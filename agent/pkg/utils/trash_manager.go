package utils

import (
	"fmt"
	"os"
	"path/filepath"
	"time"
)

// MoveToTrash 将文件移动到回收站 (符合 XDG 规范)
func MoveToTrash(srcPath string) error {
	home, err := os.UserHomeDir()
	if err != nil {
		return err
	}

	trashRoot := filepath.Join(home, ".local", "share", "Trash")
	trashFiles := filepath.Join(trashRoot, "files")
	trashInfo := filepath.Join(trashRoot, "info")

	// 1. 确保回收站目录存在
	if err := os.MkdirAll(trashFiles, 0700); err != nil {
		return err
	}
	if err := os.MkdirAll(trashInfo, 0700); err != nil {
		return err
	}

	// 2. 计算目标文件名
	baseName := filepath.Base(srcPath)
	dstPath := filepath.Join(trashFiles, baseName)
	infoPath := filepath.Join(trashInfo, baseName+".trashinfo")

	// 3. 处理重名 (如果回收站里已经有了同名文件)
	// 规范要求：如果重名，追加序号，如 a.txt -> a.2.txt
	i := 2
	for {
		_, err := os.Stat(dstPath)
		if os.IsNotExist(err) {
			break // 没重名，可以用
		}
		// 重名了，换个名字
		ext := filepath.Ext(baseName)
		name := baseName[:len(baseName)-len(ext)]
		newBase := fmt.Sprintf("%s.%d%s", name, i, ext)

		dstPath = filepath.Join(trashFiles, newBase)
		infoPath = filepath.Join(trashInfo, newBase+".trashinfo")
		i++
	}

	// 4. 准备 .trashinfo 内容
	// 格式:
	// [Trash Info]
	// Path=/home/user/file.txt
	// DeletionDate=YYYY-MM-DDThh:mm:ss
	absPath, _ := filepath.Abs(srcPath)
	now := time.Now().Format("2006-01-02T15:04:05")
	infoContent := fmt.Sprintf("[Trash Info]\nPath=%s\nDeletionDate=%s\n", absPath, now)

	// 5. 写入 info 文件
	if err := os.WriteFile(infoPath, []byte(infoContent), 0644); err != nil {
		return fmt.Errorf("failed to write trash info: %v", err)
	}

	// 6. 移动原文件
	if err := os.Rename(srcPath, dstPath); err != nil {
		// 如果移动失败，把 info 也删了
		os.Remove(infoPath)
		return fmt.Errorf("failed to move file to trash: %v", err)
	}

	return nil
}
