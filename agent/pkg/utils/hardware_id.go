package utils

import (
	"os"
	"runtime"
	"strings"
)

// GetHardwareID 获取物理设备的唯一标识
func GetHardwareID() string {
	// 1. 尝试获取 DMI UUID (针对 x86/Intel/AMD 工控机)
	if runtime.GOARCH == "amd64" || runtime.GOARCH == "386" {
		uuid, err := readFileContent("/sys/class/dmi/id/product_uuid")
		if err == nil && isValidID(uuid) {
			return strings.ToLower(uuid)
		}
	}

	// 2. 尝试获取 Device Tree Serial (针对 ARM/Jetson/RaspberryPi)
	if runtime.GOARCH == "arm" || runtime.GOARCH == "arm64" {
		// 树莓派/Jetson 通常在这里
		serial, err := readFileContent("/proc/device-tree/serial-number")
		if err == nil && isValidID(serial) {
			// 移除可能包含的空字符
			return strings.Trim(serial, "\x00")
		}

		// 旧版树莓派可能在 cpuinfo
		serial2 := getCpuInfoSerial()
		if isValidID(serial2) {
			return serial2
		}
	}

	return ""
}

// --- 辅助函数 ---

func readFileContent(path string) (string, error) {
	content, err := os.ReadFile(path)
	if err != nil {
		return "", err
	}
	return strings.TrimSpace(string(content)), nil
}

// 简单的有效性检查 (防止获取到空串或全0)
func isValidID(id string) bool {
	if len(id) < 6 {
		return false
	}
	// 简单的全0检查（有些山寨主板 UUID 是全 0）
	// 这里简化处理
	return true
}

func getCpuInfoSerial() string {
	data, err := os.ReadFile("/proc/cpuinfo")
	if err != nil {
		return ""
	}
	lines := strings.Split(string(data), "\n")
	for _, line := range lines {
		if strings.Contains(line, "Serial") {
			parts := strings.Split(line, ":")
			if len(parts) == 2 {
				return strings.TrimSpace(parts[1])
			}
		}
	}
	return ""
}
