package service

import (
	"bufio"
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
	cmdBuilder.WriteString(proc.config.CmdStr)

	// 遍历参数，使用 %q 进行安全引用
	// 这一步解决了 foxglove 参数中正则包含特殊字符的问题
	for _, arg := range proc.config.Args {
		cmdBuilder.WriteString(fmt.Sprintf(" %q", arg))
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
		// 获取进程组 ID (PGID)
		// 因为 pty.Start 设置了 Setsid，所以 PID == PGID
		pgid := cmd.Process.Pid

		// --- 阶段 1: 发送 SIGINT (模拟 Ctrl+C) ---
		// 使用负数 PGID 向整个进程组广播信号
		log.Printf("[Proc] Stopping %s (PGID: %d) with SIGINT...", id, pgid)
		syscall.Kill(-pgid, syscall.SIGINT)

		// --- 启动协程进行“补刀” ---
		go func() {
			// 等待，看它死没死
			time.Sleep(5 * time.Second)
			// 检查是否还在 map 中 (如果 waitProcess 结束了，会从 map 中删除)
			if _, stillRunning := pm.procs.Load(id); !stillRunning {
				return // 已经正常退出了
			}
			// --- 阶段 2: 发送 SIGTERM (强制终止请求) ---
			log.Printf("[Proc] %s still alive. Escalating to SIGTERM...", id)
			syscall.Kill(-pgid, syscall.SIGTERM)
			// 再等
			time.Sleep(3 * time.Second)
			if _, stillRunning := pm.procs.Load(id); !stillRunning {
				return
			}
			// --- 阶段 3: 发送 SIGKILL ---
			log.Printf("[Proc] %s is stubborn. Escalating to SIGKILL!", id)
			syscall.Kill(-pgid, syscall.SIGKILL)
		}()
	}
	return nil
}

func (pm *ProcessManager) streamOutput(id string, streamName string, reader io.Reader) {
	scanner := bufio.NewScanner(reader)
	for scanner.Scan() {
		text := scanner.Text()
		// 替换换行符以适配 xterm
		formattedText := strings.ReplaceAll(text, "\n", "\r\n") + "\r\n"
		connect.GlobalHub.Broadcast(connect.LogMessage{
			ProcessID: id,
			Stream:    streamName,
			Data:      formattedText,
		})
	}
	// 当 PTY 关闭时，Scanner 会停止，这里不需要额外处理错误
}
