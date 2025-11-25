package service

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

// MoveToTrash 将文件移动到回收站 (符合 XDG 规范)
func MoveToTrash(srcPath string, password string) error {
	home, err := os.UserHomeDir()
	if err != nil {
		return err
	}
	trashRoot := filepath.Join(home, ".local", "share", "Trash")
	trashFiles := filepath.Join(trashRoot, "files")
	trashInfo := filepath.Join(trashRoot, "info")
	// 1. 确保回收站目录存在 (回收站通常属于当前用户，不需要 sudo)
	if err := os.MkdirAll(trashFiles, 0700); err != nil {
		return err
	}
	if err := os.MkdirAll(trashInfo, 0700); err != nil {
		return err
	}
	// 2. 计算目标文件名 (处理重名逻辑)
	baseName := filepath.Base(srcPath)
	dstPath := filepath.Join(trashFiles, baseName)
	infoPath := filepath.Join(trashInfo, baseName+".trashinfo")
	i := 2
	for {
		_, err := os.Stat(dstPath)
		if os.IsNotExist(err) {
			break
		}
		ext := filepath.Ext(baseName)
		name := baseName[:len(baseName)-len(ext)]
		newBase := fmt.Sprintf("%s.%d%s", name, i, ext)

		dstPath = filepath.Join(trashFiles, newBase)
		infoPath = filepath.Join(trashInfo, newBase+".trashinfo")
		i++
	}
	// 3. 写入 .trashinfo
	// 注意：metadata 必须写在用户的回收站目录里，通常当前用户有权写入。
	// 即使源文件是 root 的，只要回收站目录是当前用户的，这一步就不需要 sudo。
	absPath, _ := filepath.Abs(srcPath)
	now := time.Now().Format("2006-01-02T15:04:05")
	infoContent := fmt.Sprintf("[Trash Info]\nPath=%s\nDeletionDate=%s\n", absPath, now)

	if err := os.WriteFile(infoPath, []byte(infoContent), 0644); err != nil {
		return fmt.Errorf("failed to write trash info: %v", err)
	}
	// 4. 移动原文件
	// 使用我们强大的 MovePath，它支持 Sudo 和 跨分区移动
	if err := MovePath(srcPath, dstPath, password); err != nil {
		// 如果移动失败（比如密码错误，或者权限不够），清理掉刚才创建的 info 文件
		os.Remove(infoPath)
		return fmt.Errorf("failed to move file to trash: %v", err)
	}

	return nil
}

// RestoreFromTrash 从回收站还原文件
// trashFilePath: 回收站中文件的绝对路径 (例如 /home/user/.local/share/Trash/files/demo.txt)
// password: Sudo 密码 (如果还原的目标路径需要权限)
func RestoreFromTrash(trashFilePath string, password string) error {
	// 1. 推算 Info 文件路径
	// 假设结构是 .../files/filename -> .../info/filename.trashinfo
	dir := filepath.Dir(trashFilePath)       // .../Trash/files
	trashRoot := filepath.Dir(dir)           // .../Trash
	fileName := filepath.Base(trashFilePath) // demo.txt

	// 简单校验路径结构
	if filepath.Base(dir) != "files" {
		return fmt.Errorf("invalid trash file path: must be inside 'files' directory")
	}

	infoFilePath := filepath.Join(trashRoot, "info", fileName+".trashinfo")

	// 2. 读取 Info 文件获取原始路径
	infoFile, err := os.Open(infoFilePath)
	if err != nil {
		return fmt.Errorf("metadata (.trashinfo) not found: %v", err)
	}
	defer infoFile.Close()

	var originalPath string
	scanner := bufio.NewScanner(infoFile)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		// 寻找 Path=...
		if after, ok := strings.CutPrefix(line, "Path="); ok {
			originalPath = after
			// 如果之前写入时用了 URL 编码，这里需要 url.QueryUnescape
			// 根据我们之前的 MoveToTrash 实现，是直接写入的字符串，所以直接读取
			break
		}
	}

	if originalPath == "" {
		return fmt.Errorf("invalid metadata: original path not found")
	}

	// 3. 检查目标路径的父目录是否存在
	// 如果父目录没了（比如删除了整个文件夹），还原通常会失败或需要重建目录
	// MovePath 的 mv 命令通常要求父目录存在。
	// 这里做一个简单的尝试创建父目录（使用普通权限，如果失败，MovePath 里的 sudo 可能解决，或者直接报错）
	parentDir := filepath.Dir(originalPath)
	if _, err := os.Stat(parentDir); os.IsNotExist(err) {
		// 尝试创建父目录 (复用 CreateDir，支持 Sudo)
		if err := CreateDir(parentDir, password); err != nil {
			return fmt.Errorf("failed to recreate parent directory: %v", err)
		}
	}

	// 4. 执行移动 (还原)
	// 直接复用 file_manager.go 中的 MovePath，它支持跨分区和 Sudo
	if err := MovePath(trashFilePath, originalPath, password); err != nil {
		return fmt.Errorf("restore failed: %v", err)
	}

	// 5. 清理 Info 文件
	// 这一步通常不需要 sudo，因为 Trash 目录是用户拥有的
	if err := os.Remove(infoFilePath); err != nil {
		// 这是一个非致命错误，只是留下了垃圾文件
		fmt.Printf("Warning: failed to delete info file: %v\n", err)
	}

	return nil
}

// TryCleanupTrashInfo 尝试清理回收站元数据
// 当用户对回收站内的文件执行 移动(剪切) 或 永久删除 时调用
func TryCleanupTrashInfo(filePath string) {
	// 1. 检查路径是否包含 Trash/files
	// 这是一个简单的启发式检查，适配标准 XDG 路径
	if !strings.Contains(filePath, "/Trash/files/") {
		return
	}

	// 2. 推算 Info 文件路径
	dir := filepath.Dir(filePath)       // .../Trash/files
	trashRoot := filepath.Dir(dir)      // .../Trash
	fileName := filepath.Base(filePath) // a.txt

	// 再次确认父目录名为 files
	if filepath.Base(dir) != "files" {
		return
	}

	infoFilePath := filepath.Join(trashRoot, "info", fileName+".trashinfo")

	// 3. 尝试删除元数据
	// 忽略错误（因为如果文件不是通过标准方式进回收站的，可能本来就没 info）
	os.Remove(infoFilePath)
}
