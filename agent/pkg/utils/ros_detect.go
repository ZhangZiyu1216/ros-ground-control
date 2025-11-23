package utils

import (
	"os"
	"path/filepath"
	"strings"
)

const DefaultRosSetup = "/opt/ros/noetic/setup.bash"

// DetectWorkspaceSetup 根据 launch 文件路径，向上查找最近的工作空间 setup.bash
func DetectWorkspaceSetup(launchFilePath string) string {
	// 如果路径为空，直接返回默认
	if launchFilePath == "" {
		return DefaultRosSetup
	}

	// 获取文件的绝对路径
	absPath, err := filepath.Abs(launchFilePath)
	if err != nil {
		return DefaultRosSetup
	}

	dir := filepath.Dir(absPath)

	// 循环向上查找，直到根目录
	for {
		// 1. 检查 devel/setup.bash (catkin_make)
		develSetup := filepath.Join(dir, "devel", "setup.bash")
		if fileExists(develSetup) {
			return develSetup
		}

		// 2. 检查 install/setup.bash (catkin build / install)
		installSetup := filepath.Join(dir, "install", "setup.bash")
		if fileExists(installSetup) {
			return installSetup
		}

		// 3. 检查是否有一级目录叫 devel 或 install (防止用户传进来的路径本身就是 devel 内)
		// 这种情况比较少见，主要是为了健壮性

		// 向上移动一级
		parent := filepath.Dir(dir)
		if parent == dir {
			// 到达根目录，停止
			break
		}
		dir = parent
	}

	// 如果没找到工作空间，回退策略：
	// 尝试返回用户的 .bashrc 吗？不，这在非交互式 shell 中不可靠。
	// 直接返回全局默认
	return DefaultRosSetup
}

func fileExists(path string) bool {
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		return false
	}
	return !info.IsDir()
}

// ExtractLaunchFile 从参数列表中提取可能是 launch 文件的路径
// 简单的启发式：寻找以 .launch 结尾的参数
func ExtractLaunchFile(args []string) string {
	for _, arg := range args {
		if strings.HasSuffix(arg, ".launch") || strings.HasSuffix(arg, ".launch.xml") {
			return arg
		}
	}
	return ""
}
