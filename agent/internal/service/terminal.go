package service

import (
	"os"
	"os/exec"
	"syscall"

	"github.com/creack/pty"
)

// TerminalSession 代表一个终端会话
type TerminalSession struct {
	PtyFile *os.File // PTY 的文件描述符 (既是 Reader 也是 Writer)
	Cmd     *exec.Cmd
}

// NewTerminalSession 启动一个新的 Shell (bash)
func NewTerminalSession() (*TerminalSession, error) {
	// 默认使用 bash，如果环境里没有 bash，可以回退到 sh
	shell := os.Getenv("SHELL")
	if shell == "" {
		shell = "bash"
	}

	cmd := exec.Command(shell)

	// 关键：设置环境变量，告诉程序这是一个支持颜色的终端
	cmd.Env = append(os.Environ(), "TERM=xterm-256color")

	// pty.Start 会做以下几件事：
	// 1. 打开一个伪终端对 (master/slave)
	// 2. 将 cmd 的 Stdin/Stdout/Stderr 绑定到 slave
	// 3. 返回 master 文件句柄
	ptmx, err := pty.Start(cmd)
	if err != nil {
		return nil, err
	}

	return &TerminalSession{
		PtyFile: ptmx,
		Cmd:     cmd,
	}, nil
}

// Resize 调整终端大小
func (ts *TerminalSession) Resize(rows, cols uint16) error {
	// pty 库提供了 Setsize 方法
	return pty.Setsize(ts.PtyFile, &pty.Winsize{
		Rows: rows,
		Cols: cols,
		X:    0,
		Y:    0,
	})
}

// Close 关闭终端
func (ts *TerminalSession) Close() error {
	if ts.PtyFile != nil {
		ts.PtyFile.Close()
	}
	if ts.Cmd != nil && ts.Cmd.Process != nil {
		// 发送 SIGKILL 确保 Shell 退出
		ts.Cmd.Process.Signal(syscall.SIGKILL)
		ts.Cmd.Wait()
	}
	return nil
}

// Read/Write 直接通过 ts.PtyFile 进行，不需要额外封装
