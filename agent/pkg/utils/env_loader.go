package utils

import (
	"bufio"
	"bytes"
	"fmt"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"time"
)

var (
	userEnvCache []string
	envOnce      sync.Once
)

// GetUserEnv 强制加载用户的 .bashrc 并提取环境变量
func GetUserEnv() []string {
	envOnce.Do(func() {
		// 1. 获取用户 Home 目录
		homeDir, err := os.UserHomeDir()
		if err != nil {
			log.Printf("[EnvLoader] Failed to get home dir: %v", err)
			return
		}
		bashrcPath := filepath.Join(homeDir, ".bashrc")

		// 2. 构造命令：强制 source .bashrc 然后打印 env
		// 我们不使用 -i，而是直接 source，这样可以绕过 [ -z "$PS1" ] 的检查
		cmdStr := fmt.Sprintf("source %s && env", bashrcPath)
		cmd := exec.Command("bash", "-ic", cmdStr)

		var stdout bytes.Buffer
		cmd.Stdout = &stdout

		// 3. 执行并设置超时
		log.Println("[EnvLoader] Harvesting environment variables from .bashrc...")
		timer := time.AfterFunc(5*time.Second, func() {
			if cmd.Process != nil {
				cmd.Process.Kill()
			}
		})
		err = cmd.Run()
		timer.Stop()

		if err != nil {
			log.Printf("[EnvLoader] Error running bashrc harvest: %v", err)
			return
		}

		// 4. 解析输出
		scanner := bufio.NewScanner(&stdout)
		capturedEnv := []string{}

		// 定义我们需要“收割”的关键前缀
		prefixes := []string{
			"ROS_",
			"PYTHONPATH",
			"LD_LIBRARY_PATH",
			"CMAKE_PREFIX_PATH",
			"PATH",
			"GAZEBO_",
			"PX4_", // PX4 专用
		}

		count := 0
		for scanner.Scan() {
			line := scanner.Text()
			for _, p := range prefixes {
				if strings.HasPrefix(line, p) {
					capturedEnv = append(capturedEnv, line)
					count++
					break
				}
			}
		}

		log.Printf("[EnvLoader] Successfully harvested %d variables.", count)
		userEnvCache = capturedEnv
	})

	return userEnvCache
}
