package service

import (
	"bytes"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

type FileInfo struct {
	Name    string    `json:"name"`
	Size    int64     `json:"size"`
	ModTime time.Time `json:"modTime"`
	IsDir   bool      `json:"isDir"`
	Mode    string    `json:"mode"`
}

// InspectResult 探测结果
type InspectResult struct {
	Size   int64
	Header []byte // 前 512 字节
}

// 辅助函数：执行 sudo 命令
func execSudo(password string, command string, args ...string) error {
	cmd := exec.Command("sudo", append([]string{"-S", command}, args...)...)
	cmd.Stdin = bytes.NewBufferString(password + "\n")
	output, err := cmd.CombinedOutput()
	if err != nil {
		return fmt.Errorf("sudo %s failed: %v, output: %s", command, err, string(output))
	}
	return nil
}

// 辅助函数： InspectFile 获取文件大小和头部字节 (支持 Sudo)
func InspectFile(path string, password string) (*InspectResult, error) {
	// 1. 普通模式
	if password == "" {
		info, err := os.Stat(path)
		if err != nil {
			return nil, err
		}
		f, err := os.Open(path)
		if err != nil {
			return nil, err
		}
		defer f.Close()
		header := make([]byte, 512)
		n, _ := f.Read(header) // 忽略错误，因为文件可能小于 512 字节
		return &InspectResult{
			Size:   info.Size(),
			Header: header[:n],
		}, nil
	}

	// 2. Sudo 模式
	// 2.1 获取文件大小: sudo stat -c %s <path>
	cmdSize := exec.Command("sudo", "-S", "stat", "-c", "%s", path)
	cmdSize.Stdin = bytes.NewBufferString(password + "\n")
	outSize, err := cmdSize.CombinedOutput()
	if err != nil {
		return nil, fmt.Errorf("sudo stat failed: %v", err)
	}

	sizeStr := strings.TrimSpace(string(outSize))
	size, _ := strconv.ParseInt(sizeStr, 10, 64)

	// 2.2 获取头部字节: sudo head -c 512 <path>
	cmdHead := exec.Command("sudo", "-S", "head", "-c", "512", path)
	cmdHead.Stdin = bytes.NewBufferString(password + "\n")
	header, err := cmdHead.CombinedOutput()
	if err != nil {
		// head 命令失败可能意味着文件不可读
		return nil, fmt.Errorf("sudo head failed: %v", err)
	}

	return &InspectResult{
		Size:   size,
		Header: header,
	}, nil
}

// ListDir 获取目录列表
func ListDir(path string) ([]FileInfo, error) {
	entries, err := os.ReadDir(path)
	if err != nil {
		return nil, err
	}

	var files []FileInfo
	for _, entry := range entries {
		info, err := entry.Info()
		if err != nil {
			continue
		}
		files = append(files, FileInfo{
			Name:    entry.Name(),
			Size:    info.Size(),
			ModTime: info.ModTime(),
			IsDir:   entry.IsDir(),
		})
	}
	return files, nil
}

// ReadFile 读取文件
func ReadFile(path string, password string) (string, error) {
	// 1. 如果没有提供密码，尝试普通读取
	if password == "" {
		content, err := os.ReadFile(path)
		if err != nil {
			return "", err
		}
		return string(content), nil
	}
	// 2. 如果提供了密码，使用 sudo -S cat
	// 构造命令: sudo -S cat <path>
	cmd := exec.Command("sudo", "-S", "cat", path)
	// 设置 Stdin 用于输入密码
	cmd.Stdin = bytes.NewBufferString(password + "\n")
	// 获取输出
	output, err := cmd.CombinedOutput()
	if err != nil {
		// 尝试区分是密码错误还是文件不存在
		outStr := string(output)
		if len(outStr) > 0 {
			return "", fmt.Errorf("sudo error: %v, output: %s", err, outStr)
		}
		return "", err
	}
	return string(output), nil
}

// WriteFile 写入文件
func WriteFile(path string, content string, password string) error {
	// 1. 普通写入
	if password == "" {
		return os.WriteFile(path, []byte(content), 0644)
	}
	// 2. Sudo 写入
	// 策略：Write temp -> Sudo cp temp dest -> Remove temp
	// A. 创建临时文件
	tmpFile, err := os.CreateTemp("", "agent-edit-*")
	if err != nil {
		return fmt.Errorf("failed to create temp file: %v", err)
	}
	tmpPath := tmpFile.Name()
	defer os.Remove(tmpPath) // 确保清理
	// 写入内容到临时文件
	if _, err := tmpFile.WriteString(content); err != nil {
		tmpFile.Close()
		return err
	}
	tmpFile.Close() // 关闭以便 cp 操作
	// B. 使用 sudo cp 覆盖目标
	cmd := exec.Command("sudo", "-S", "cp", tmpPath, path)
	// 输入密码
	cmd.Stdin = bytes.NewBufferString(password + "\n")
	if output, err := cmd.CombinedOutput(); err != nil {
		return fmt.Errorf("sudo cp failed: %v, output: %s", err, string(output))
	}

	// 如果是从回收站删除，顺便清理元数据
	TryCleanupTrashInfo(path)

	return nil
}

// CreateDir 创建目录 (类似 mkdir -p)
func CreateDir(path string, password string) error {
	if password != "" {
		return execSudo(password, "mkdir", "-p", path)
	}
	return os.MkdirAll(path, 0755)
}

// DeletePath 删除文件或目录 (类似 rm -rf)
func DeletePath(path string, password string) error {
	if path == "/" || path == "." {
		return fmt.Errorf("cannot delete root directory")
	}
	if password != "" {
		// sudo rm -rf
		return execSudo(password, "rm", "-rf", path)
	}
	return os.RemoveAll(path)
}

// MovePath 移动或重命名
func MovePath(src, dst string, password string) error {
	// 1. Sudo 模式：直接调用 mv，它自动处理跨分区移动
	if password != "" {
		if err := execSudo(password, "mv", src, dst); err != nil {
			return err
		}
		TryCleanupTrashInfo(src)
		return nil
	}
	// 2. 普通模式：尝试原子重命名
	err := os.Rename(src, dst)
	if err == nil {
		TryCleanupTrashInfo(src)
		return nil
	}
	// 3. 普通模式回退：Copy + Delete
	if _, statErr := os.Stat(src); os.IsNotExist(statErr) {
		return statErr
	}
	// 注意：这里调用 CopyPath 和 DeletePath 时传空密码
	// 因为我们还在普通模式尝试阶段
	if copyErr := CopyPath(src, dst, ""); copyErr != nil {
		return fmt.Errorf("move failed: rename error (%v), fallback copy error (%v)", err, copyErr)
	}
	if delErr := DeletePath(src, ""); delErr != nil {
		// 这里就是你担心的场景：复制成功，删除失败
		// 返回具体的错误信息，让前端捕获后提示用户输入密码
		return fmt.Errorf("move incomplete: copied successfully but failed to delete source: %v", delErr)
	}
	return nil
}

// CopyPath 复制文件或目录
func CopyPath(src, dst string, password string) error {
	if password != "" {
		// sudo cp -r
		// 注意：cp -r 会保留目录结构，适合递归复制
		return execSudo(password, "cp", "-r", src, dst)
	}
	info, err := os.Stat(src)
	if err != nil {
		return err
	}
	if info.IsDir() {
		return copyDir(src, dst)
	}
	return copyFile(src, dst)
}

// --- 内部辅助函数：递归复制 ---
func copyFile(src, dst string) error {
	sourceFile, err := os.Open(src)
	if err != nil {
		return err
	}
	defer sourceFile.Close()
	// 创建目标文件 (会覆盖)
	destFile, err := os.Create(dst)
	if err != nil {
		return err
	}
	defer destFile.Close()
	// 复制内容
	if _, err := io.Copy(destFile, sourceFile); err != nil {
		return err
	}
	// 复制权限位
	sourceInfo, err := os.Stat(src)
	if err == nil {
		os.Chmod(dst, sourceInfo.Mode())
	}

	return nil
}

func copyDir(src, dst string) error {
	// 1. 获取源目录权限
	sourceInfo, err := os.Stat(src)
	if err != nil {
		return err
	}
	// 2. 创建目标目录
	if err := os.MkdirAll(dst, sourceInfo.Mode()); err != nil {
		return err
	}
	// 3. 遍历源目录
	entries, err := os.ReadDir(src)
	if err != nil {
		return err
	}
	for _, entry := range entries {
		srcPath := filepath.Join(src, entry.Name())
		dstPath := filepath.Join(dst, entry.Name())

		if entry.IsDir() {
			if err := copyDir(srcPath, dstPath); err != nil {
				return err
			}
		} else {
			if err := copyFile(srcPath, dstPath); err != nil {
				return err
			}
		}
	}
	return nil
}
