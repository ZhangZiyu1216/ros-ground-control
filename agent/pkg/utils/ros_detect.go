package utils

import (
	"os"
	"path/filepath"
	"strings"
)

const DefaultRosSetup = "/opt/ros/noetic/setup.bash"

// DetectWorkspaceSetup 根据文件或目录路径，向上查找最近的工作空间 setup.bash
func DetectWorkspaceSetup(path string) string {
	if path == "" {
		return DefaultRosSetup
	}

	absPath, err := filepath.Abs(path)
	if err != nil {
		return DefaultRosSetup
	}

	// --- 核心优化：判断是文件还是目录 ---
	info, err := os.Stat(absPath)
	var dir string
	if err == nil && info.IsDir() {
		// 如果是目录，直接从该目录开始找
		dir = absPath
	} else {
		// 如果是文件（或不存在），从父目录开始找
		dir = filepath.Dir(absPath)
	}

	// 循环向上查找
	for {
		// 1. 检查 devel/setup.bash
		develSetup := filepath.Join(dir, "devel", "setup.bash")
		if fileExists(develSetup) {
			return develSetup
		}

		// 2. 检查 install/setup.bash
		installSetup := filepath.Join(dir, "install", "setup.bash")
		if fileExists(installSetup) {
			return installSetup
		}

		// 3. 检查 install_isolated/setup.bash (Cartographer 等)
		installIsoSetup := filepath.Join(dir, "install_isolated", "setup.bash")
		if fileExists(installIsoSetup) {
			return installIsoSetup
		}

		// 向上移动
		parent := filepath.Dir(dir)
		if parent == dir {
			break // 到达根目录
		}
		dir = parent
	}

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
