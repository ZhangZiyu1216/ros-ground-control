package service

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"os/exec"
	"ros-ground-control/agent/internal/connect"
	"strings"
	"sync"
	"syscall"
	"time"
)

// ProcessConfig 定义进程启动参数
type ProcessConfig struct {
	ID          string
	CmdStr      string
	Args        []string
	Env         []string // 额外的环境变量 "KEY=VALUE"
	SetupScript string
}

// 用于 API 返回的结构体
type ProcessInfo struct {
	ID        string `json:"id"`
	Cmd       string `json:"cmd"`
	PID       int    `json:"pid"`
	StartTime string `json:"start_time"` // ISO8601 字符串
}

// 定义回调函数类型
// id: 进程ID, manual: 是否是用户手动停止
type ExitCallback func(id string, manual bool)

type MonitoredProcess struct {
	cmd        *exec.Cmd
	config     ProcessConfig
	manualStop bool      // 标记是否是用户手动停止的
	startTime  time.Time // 记录启动时间
	mu         sync.Mutex
}

type ProcessManager struct {
	procs  sync.Map     // map[string]*MonitoredProcess
	onExit ExitCallback // 退出回调钩子
}

var GlobalProcManager = &ProcessManager{}

// 设置回调的方法
func (pm *ProcessManager) SetExitCallback(cb ExitCallback) {
	pm.onExit = cb
}

// 获取进程列表方法
func (pm *ProcessManager) ListProcesses() []ProcessInfo {
	list := []ProcessInfo{}

	pm.procs.Range(func(key, value interface{}) bool {
		proc := value.(*MonitoredProcess)

		// 获取 PID
		pid := 0
		if proc.cmd != nil && proc.cmd.Process != nil {
			pid = proc.cmd.Process.Pid
		}

		list = append(list, ProcessInfo{
			ID:        proc.config.ID,
			Cmd:       proc.config.CmdStr, // 只返回主命令，不返回冗长的 source 脚本
			PID:       pid,
			StartTime: proc.startTime.Format(time.RFC3339),
		})
		return true // 继续遍历
	})

	return list
}

// StartProcess 统一入口
func (pm *ProcessManager) StartProcess(cfg ProcessConfig) error {
	// 检查是否已存在
	if _, loaded := pm.procs.Load(cfg.ID); loaded {
		return fmt.Errorf("process %s is already running", cfg.ID)
	}

	proc := &MonitoredProcess{
		config:     cfg,
		manualStop: false,
		startTime:  time.Now(),
	}
	pm.procs.Store(cfg.ID, proc)

	// 1. 构建环境导出指令 (Pre-commands)
	var exportCmds string
	for _, e := range proc.config.Env {
		if strings.HasPrefix(e, "ROS_") {
			exportCmds += fmt.Sprintf("export %s; ", e)
		}
	}

	// 2. 构建 Source 指令
	var sourceCmd string
	setupFile := proc.config.SetupScript

	if setupFile == "USER_BASHRC" {
		sourceCmd = "export PS1=1; source ~/.bashrc; "
	} else if setupFile != "" {
		// 标准工作空间，使用 source
		sourceCmd = fmt.Sprintf("source %s; ", setupFile)
	} else {
		// 默认情况
		sourceCmd = "source /opt/ros/noetic/setup.bash; "
	}

	// 3. 构建主指令
	mainCmd := proc.config.CmdStr
	for _, arg := range proc.config.Args {
		mainCmd += " " + arg
	}

	// 4. 拼接最终的 Shell 命令串
	// 顺序：Export(注入IP) -> Source(加载环境) -> Run(执行)
	fullCmdStr := fmt.Sprintf("%s%s%s", sourceCmd, exportCmds, mainCmd)

	// Log 一下最终执行的命令，方便调试
	fmt.Printf("[ProcManager] Executing: bash -c \"%s\"\n", fullCmdStr)

	cmd := exec.Command("bash", "-c", fullCmdStr)
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}

	// 5. 依然继承 OS 的环境变量 (PATH, USER 等基础信息)
	cmd.Env = os.Environ()
	cmd.Env = append(cmd.Env, proc.config.Env...)

	stdout, _ := cmd.StdoutPipe()
	stderr, _ := cmd.StderrPipe()

	if err := cmd.Start(); err != nil {
		pm.procs.Delete(proc.config.ID)
		return err
	}

	proc.mu.Lock()
	proc.cmd = cmd
	proc.mu.Unlock()

	go pm.streamOutput(proc.config.ID, "stdout", stdout)
	go pm.streamOutput(proc.config.ID, "stderr", stderr)
	go pm.waitProcess(proc)

	return nil
}

func (pm *ProcessManager) waitProcess(proc *MonitoredProcess) {
	err := proc.cmd.Wait()

	proc.mu.Lock()
	isManual := proc.manualStop
	procID := proc.config.ID
	proc.mu.Unlock()

	// 发送日志
	exitMsg := "Process exited."
	if err != nil {
		exitMsg = fmt.Sprintf("Process exited with error: %v", err)
	}
	connect.GlobalHub.Broadcast(connect.LogMessage{
		ProcessID: procID,
		Stream:    "system",
		Data:      exitMsg,
	})

	// 清理 Map
	pm.procs.Delete(procID)

	// 触发回调，通知 ROSManager
	if pm.onExit != nil {
		// 在 Goroutine 中调用，防止阻塞 waitProcess
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
	proc.manualStop = true // 标记为手动停止，防止触发自动重启
	cmd := proc.cmd
	proc.mu.Unlock()

	if cmd != nil && cmd.Process != nil {
		// 发送 SIGINT
		return syscall.Kill(-cmd.Process.Pid, syscall.SIGINT)
	}
	return nil
}

func (pm *ProcessManager) streamOutput(id string, streamName string, reader io.Reader) {
	scanner := bufio.NewScanner(reader)
	for scanner.Scan() {
		connect.GlobalHub.Broadcast(connect.LogMessage{
			ProcessID: id,
			Stream:    streamName,
			Data:      scanner.Text(),
		})
	}
}
