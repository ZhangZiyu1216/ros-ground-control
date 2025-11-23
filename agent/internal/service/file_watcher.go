package service

import (
	"encoding/json"
	"log"
	"path/filepath"
	"ros-ground-control/agent/internal/connect"
	"sync"

	"github.com/fsnotify/fsnotify"
)

type FileWatcher struct {
	watcher *fsnotify.Watcher
	mu      sync.Mutex
	// 记录当前正在监控的目录，防止重复添加
	watchingPath string
}

var GlobalFileWatcher *FileWatcher

// InitFileWatcher 初始化监控器
func InitFileWatcher() error {
	w, err := fsnotify.NewWatcher()
	if err != nil {
		return err
	}

	GlobalFileWatcher = &FileWatcher{
		watcher: w,
	}

	// 启动后台协程处理事件
	go GlobalFileWatcher.run()

	return nil
}

// SetWatchPath 设置要监控的目录 (切换目录时调用)
func (fw *FileWatcher) SetWatchPath(path string) error {
	fw.mu.Lock()
	defer fw.mu.Unlock()

	// 1. 如果之前有监控，先移除
	if fw.watchingPath != "" {
		fw.watcher.Remove(fw.watchingPath)
	}

	// 2. 添加新监控
	// Clean 清理路径 (处理 .. 等符号)
	cleanPath := filepath.Clean(path)
	if err := fw.watcher.Add(cleanPath); err != nil {
		return err
	}

	fw.watchingPath = cleanPath
	log.Printf("[FileWatcher] Watching directory: %s", cleanPath)
	return nil
}

// run 后台循环，读取 fsnotify 事件并广播到 WebSocket
func (fw *FileWatcher) run() {
	defer fw.watcher.Close()

	for {
		select {
		case event, ok := <-fw.watcher.Events:
			if !ok {
				return
			}

			// 我们只关心当前监控目录下的直接变动
			// fsnotify 的 Op 是位掩码，转换成字符串发给前端
			opStr := ""
			if event.Op&fsnotify.Create == fsnotify.Create {
				opStr = "create"
			} else if event.Op&fsnotify.Write == fsnotify.Write {
				opStr = "write"
			} else if event.Op&fsnotify.Remove == fsnotify.Remove {
				opStr = "remove"
			} else if event.Op&fsnotify.Rename == fsnotify.Rename {
				opStr = "rename"
			} else if event.Op&fsnotify.Chmod == fsnotify.Chmod {
				opStr = "chmod"
			}

			if opStr == "" {
				continue
			}

			// 构造消息载荷
			payload := map[string]string{
				"event": opStr,
				"path":  event.Name, // 文件的绝对路径
			}
			dataBytes, _ := json.Marshal(payload)

			// 通过 WebSocket 广播
			// ProcessID 使用特定标识 "sys-fs-watcher"
			connect.GlobalHub.Broadcast(connect.LogMessage{
				ProcessID: "sys-fs-watcher",
				Stream:    "fs-event",
				Data:      string(dataBytes),
			})

		case err, ok := <-fw.watcher.Errors:
			if !ok {
				return
			}
			log.Printf("[FileWatcher] Error: %v", err)
		}
	}
}
