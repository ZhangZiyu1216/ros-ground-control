package service

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"os"
	"os/exec"
	"ros-ground-control/agent/internal/connect"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/creack/pty"
)

// ProcessConfig 定义进程启动参数
type ProcessConfig struct {
	ID          string
	CmdStr      string
	Args        []string
	Env         []string // 额外的环境变量
	SetupScript string   // 启动脚本路径
	RawArgs     bool     // 新增：是否使用原始参数模式 (不加引号，不转义)
	Nice        int      // 新增：进程优先级 (-20 到 19，越小优先级越高)
}

// ExitCallback 回调函数
type ExitCallback func(id string, manual bool)

type MonitoredProcess struct {
	cmd         *exec.Cmd
	ptyFile     *os.File // PTY 文件句柄
	config      ProcessConfig
	manualStop  bool
	startTime   time.Time
	mu          sync.Mutex
	isExternal  bool
	externalPID int
}

type ProcessInfo struct {
	ID        string `json:"id"`
	Cmd       string `json:"cmd"`
	PID       int    `json:"pid"`
	StartTime string `json:"start_time"`
}

type ProcessManager struct {
	procs  sync.Map
	onExit ExitCallback
}

var GlobalProcManager = &ProcessManager{}

// IsRunning 检查指定 ID 的进程是否正在受管运行
func (pm *ProcessManager) IsRunning(id string) bool {
	_, ok := pm.procs.Load(id)
	return ok
}

func (pm *ProcessManager) SetExitCallback(cb ExitCallback) {
	pm.onExit = cb
}

func (pm *ProcessManager) ListProcesses() []ProcessInfo {
	list := []ProcessInfo{}
	pm.procs.Range(func(key, value interface{}) bool {
		proc := value.(*MonitoredProcess)
		pid := 0
		if proc.isExternal {
			pid = proc.externalPID
		} else if proc.cmd != nil && proc.cmd.Process != nil {
			pid = proc.cmd.Process.Pid
		}
		list = append(list, ProcessInfo{
			ID:        proc.config.ID,
			Cmd:       proc.config.CmdStr,
			PID:       pid,
			StartTime: proc.startTime.Format(time.RFC3339),
		})
		return true
	})
	return list
}

func (pm *ProcessManager) AdoptExternalProcess(id string, cmdStr string, pid int) {
	proc := &MonitoredProcess{
		config:      ProcessConfig{ID: id, CmdStr: cmdStr},
		startTime:   time.Now(),
		isExternal:  true,
		externalPID: pid,
	}
	pm.procs.Store(id, proc)
	connect.GlobalHub.Broadcast(connect.LogMessage{
		ProcessID: id,
		Stream:    "system",
		Data:      fmt.Sprintf("Process adopted from external PID: %d.", pid),
	})
}

func (pm *ProcessManager) StartProcess(cfg ProcessConfig) error {
	if _, loaded := pm.procs.Load(cfg.ID); loaded {
		return fmt.Errorf("process %s is already running", cfg.ID)
	}

	proc := &MonitoredProcess{
		config:     cfg,
		manualStop: false,
		startTime:  time.Now(),
	}
	pm.procs.Store(cfg.ID, proc)

	if err := pm.run(proc); err != nil {
		pm.procs.Delete(cfg.ID)
		return err
	}
	return nil
}

// 核心修改：run 方法
func (pm *ProcessManager) run(proc *MonitoredProcess) error {
	// 1. 准备环境变量注入指令
	var exportCmds string
	for _, e := range proc.config.Env {
		// 简单的注入过滤，防止注入无关变量
		if strings.HasPrefix(e, "ROS_") {
			// 使用 %q 自动加引号，防止变量值包含空格
			parts := strings.SplitN(e, "=", 2)
			if len(parts) == 2 {
				exportCmds += fmt.Sprintf("export %s=%q; ", parts[0], parts[1])
			}
		}
	}

	// 2. 准备 Source 和 Bash 模式
	var sourceCmd string
	setupFile := proc.config.SetupScript

	// 默认基础命令
	bashArgs := []string{"-c"}

	if setupFile == "USER_BASHRC" {
		// 交互模式：强制加载 ~/.bashrc
		// set +H: 关闭历史扩展，防止参数中的 '!' 被误解释
		sourceCmd = "set +H; source ~/.bashrc; "
		// 使用 -i 开启交互模式
		bashArgs = []string{"-i", "-c"}
	} else if setupFile != "" {
		sourceCmd = fmt.Sprintf("source %q; ", setupFile)
	} else {
		sourceCmd = "source /opt/ros/noetic/setup.bash; "
	}

	// 3. 拼接主命令 (核心修复点)
	// 使用 strings.Builder 更加高效且清晰
	var cmdBuilder strings.Builder
	cmdBuilder.WriteString(sourceCmd)
	cmdBuilder.WriteString(exportCmds)
	cmdBuilder.WriteString("exec ")
	cmdBuilder.WriteString(proc.config.CmdStr)

	// 遍历参数，使用 %q 进行安全引用
	for _, arg := range proc.config.Args {
		if proc.config.RawArgs {
			// --- 核心修复：Raw 模式 ---
			// 直接拼接字符串，只加前置空格
			// 这种模式下，前端传来的字符串里有什么引号、空格，都会原样保留给 bash
			cmdBuilder.WriteString(" " + arg)
		} else {
			// --- 原有模式：Safe Quote ---
			// 适用于 roslaunch 的文件路径，防止路径空格导致解析错误
			cmdBuilder.WriteString(fmt.Sprintf(" %q", arg))
		}
	}

	fullCmdStr := cmdBuilder.String()

	// 将拼接好的命令字符串作为 bash 的最后一个参数
	bashArgs = append(bashArgs, fullCmdStr)

	cmd := exec.Command("bash", bashArgs...)

	// 继承环境
	cmd.Env = os.Environ()
	// 虽然我们用了 exportCmds，但在 cmd.Env 里也加一份作为双重保险
	cmd.Env = append(cmd.Env, proc.config.Env...)

	fmt.Printf("[ProcManager] Executing: bash %v \"%s\"\n", bashArgs[:len(bashArgs)-1], fullCmdStr)

	// 4. 使用 PTY 启动
	// pty.Start 会处理好 TTY 的分配
	f, err := pty.Start(cmd)
	if err != nil {
		return err
	}

	if proc.config.Nice != 0 {
		// syscall.PRIO_PROCESS = 0 (表示操作进程 ID)
		// cmd.Process.Pid 是刚才启动的 bash 的 PID
		// 注意：Setpriority 需要 Agent 拥有 CAP_SYS_NICE 才能设置负数
		// 这里的 Setpriority 会修改 bash 的 nice 值
		// 随后 bash 执行 exec roslaunch，roslaunch 会继承这个 nice 值
		err := syscall.Setpriority(syscall.PRIO_PROCESS, cmd.Process.Pid, proc.config.Nice)
		if err != nil {
			// 记录警告但不阻断运行，方便排查权限问题
			fmt.Printf("[ProcManager] Warning: Failed to set nice value to %d: %v\n", proc.config.Nice, err)
		} else {
			fmt.Printf("[ProcManager] Successfully set nice value to %d for PID %d\n", proc.config.Nice, cmd.Process.Pid)
		}
	}

	proc.mu.Lock()
	proc.cmd = cmd
	proc.ptyFile = f // 保存句柄以便后续关闭
	proc.mu.Unlock()

	// 5. 启动日志转发
	// PTY 合并了 stdout 和 stderr
	go pm.streamOutput(proc.config.ID, "stdout", f)

	// 6. 监控退出
	go pm.waitProcess(proc)

	return nil
}

func (pm *ProcessManager) waitProcess(proc *MonitoredProcess) {
	// 等待进程退出
	state, err := proc.cmd.Process.Wait()

	// 进程退出后，关闭 PTY 文件句柄，这会让 streamOutput 中的 Read 返回 EOF 从而结束循环
	proc.mu.Lock()
	if proc.ptyFile != nil {
		proc.ptyFile.Close()
	}
	isManual := proc.manualStop
	procID := proc.config.ID
	proc.mu.Unlock()

	// 构造退出消息
	exitCode := -1
	if state != nil {
		exitCode = state.ExitCode()
	}

	type ExitEvent struct {
		Event    string `json:"event"`
		ID       string `json:"id"`
		Code     int    `json:"code"`
		IsManual bool   `json:"is_manual"`
		Error    string `json:"error,omitempty"`
	}

	evt := ExitEvent{Event: "exit", ID: procID, Code: exitCode, IsManual: isManual}
	if err != nil {
		evt.Error = err.Error()
	}

	dataBytes, _ := json.Marshal(evt) // 需要引入 encoding/json

	connect.GlobalHub.Broadcast(connect.LogMessage{
		ProcessID: procID,
		Stream:    "system",
		Data:      string(dataBytes),
	})

	pm.procs.Delete(procID)
	if pm.onExit != nil {
		go pm.onExit(procID, isManual)
	}
}

func (pm *ProcessManager) StopProcess(id string) error {
	val, ok := pm.procs.Load(id)
	if !ok {
		return fmt.Errorf("process not found")
	}
	proc := val.(*MonitoredProcess)

	proc.mu.Lock()
	proc.manualStop = true

	// 1. 处理外部进程 (保持原样)
	if proc.isExternal {
		proc.mu.Unlock()
		p, err := os.FindProcess(proc.externalPID)
		if err != nil {
			pm.procs.Delete(id)
			return nil
		}
		p.Signal(syscall.SIGINT)
		// 外部进程无法wait，发完信号就认为停止了
		pm.procs.Delete(id)
		return nil
	}

	cmd := proc.cmd
	proc.mu.Unlock()

	if cmd != nil && cmd.Process != nil {
		// --- 阶段 1: 发送 SIGINT (模拟 Ctrl+C) ---
		// 使用负数 PGID 向整个进程组广播信号
		log.Printf("[Proc] Stopping %s (PID: %d) with SIGINT...", id, cmd.Process.Pid)
		cmd.Process.Signal(syscall.SIGINT)
		// --- 启动协程进行“补刀” ---
		go func() {
			time.Sleep(5 * time.Second)

			if _, stillRunning := pm.procs.Load(id); !stillRunning {
				return
			}

			log.Printf("[Proc] %s still alive. Escalating to SIGTERM...", id)
			cmd.Process.Signal(syscall.SIGTERM) // 修复：使用 Signal

			time.Sleep(3 * time.Second)

			if _, stillRunning := pm.procs.Load(id); !stillRunning {
				return
			}

			log.Printf("[Proc] %s is stubborn. Escalating to SIGKILL!", id)
			cmd.Process.Signal(syscall.SIGKILL) // 修复：使用 Signal
		}()
	}
	return nil
}

// SignalProcess 向指定进程发送特定信号 (用于暂停/恢复)
func (pm *ProcessManager) SignalProcess(id string, sig syscall.Signal) error {
	val, ok := pm.procs.Load(id)
	if !ok {
		return fmt.Errorf("process not found")
	}
	proc := val.(*MonitoredProcess)

	proc.mu.Lock()
	defer proc.mu.Unlock()

	// 1. 外部进程处理
	if proc.isExternal {
		p, err := os.FindProcess(proc.externalPID)
		if err != nil {
			return err
		}
		return p.Signal(sig)
	}

	// 2. 内部进程处理
	if proc.cmd != nil && proc.cmd.Process != nil {
		// 发送给进程组，确保子进程也能收到 (虽然 rosbag play 通常是单进程)
		// 注意：syscall.Kill 的第一个参数是 pid。
		// 如果我们使用了 pty，PID == PGID，使用负数可以发给进程组。
		pid := proc.cmd.Process.Pid
		return syscall.Kill(-pid, sig)
	}

	return fmt.Errorf("process not running")
}

func (pm *ProcessManager) streamOutput(id string, streamName string, reader io.Reader) {
	// 配置常量
	const (
		readBufferSize = 8 * 1024              // 每次从 PTY 读取的最大块 (8KB)
		flushInterval  = 50 * time.Millisecond // 刷新频率 (20fps)
		maxBufferBytes = 16 * 1024             // 发送缓冲区阈值 (16KB)
	)

	// 预分配读取缓冲区
	readBuf := make([]byte, readBufferSize)

	// 发送缓冲区 (使用 bytes.Buffer 代替 strings.Builder，性能更好)
	var sendBuffer bytes.Buffer
	// 预分配一定的容量，减少扩容
	sendBuffer.Grow(maxBufferBytes)

	// 定时器
	timer := time.NewTimer(flushInterval)
	defer timer.Stop()

	// 定义刷新函数
	flush := func() {
		if sendBuffer.Len() > 0 {
			// 只有在这里才转一次 string，且是大块数据
			connect.GlobalHub.Broadcast(connect.LogMessage{
				ProcessID: id,
				Stream:    streamName,
				Data:      sendBuffer.String(),
			})
			sendBuffer.Reset()
		}
		// 重置定时器
		if !timer.Stop() {
			select {
			case <-timer.C:
			default:
			}
		}
		timer.Reset(flushInterval)
	}

	dataChan := make(chan []byte, 100)

	// 1. 读取协程 (Producer)
	go func() {
		defer close(dataChan)
		for {
			n, err := reader.Read(readBuf)
			if n > 0 {
				// 复制数据 (因为 readBuf 会被复用)
				// 这里虽然有一次 copy，但相比 scanner 的开销小得多
				chunk := make([]byte, n)
				copy(chunk, readBuf[:n])
				dataChan <- chunk
			}
			if err != nil {
				break
			}
		}
	}()

	// 2. 处理协程 (Consumer)
	for {
		select {
		case chunk, ok := <-dataChan:
			if !ok {
				flush()
				return
			}

			// 字节级替换 \n -> \r\n
			// 优化：只有当包含 \n 时才替换
			if bytes.Contains(chunk, []byte{'\n'}) {
				// Replace 返回新的切片
				chunk = bytes.ReplaceAll(chunk, []byte{'\n'}, []byte{'\r', '\n'})
			}

			sendBuffer.Write(chunk)

			if sendBuffer.Len() >= maxBufferBytes {
				flush()
			}

		case <-timer.C:
			flush()
		}
	}
}
