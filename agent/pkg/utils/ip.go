package utils

import (
	"log"
	"net"
	"strings"
)

// GetIPByInterface 获取指定网卡的 IPv4 地址
func GetIPByInterface(ifaceName string) (string, error) {
	iface, err := net.InterfaceByName(ifaceName)
	if err != nil {
		return "", err
	}

	addrs, err := iface.Addrs()
	if err != nil {
		return "", err
	}

	for _, addr := range addrs {
		// 检查是否为 IP 地址 (包含 IPv4 和 IPv6)
		if ipNet, ok := addr.(*net.IPNet); ok && !ipNet.IP.IsLoopback() {
			// 仅返回 IPv4
			if ipNet.IP.To4() != nil {
				return ipNet.IP.String(), nil
			}
		}
	}
	return "", nil
}

// GetOutboundIP 获取对外 IP
// supportInterface: 指定首选网卡 (如 "wlan0"), 传 "" 或 "auto" 则自动探测
func GetOutboundIP(preferredInterface string) string {
	// 1. 尝试使用指定网卡
	if preferredInterface != "" && preferredInterface != "auto" {
		ip, err := GetIPByInterface(preferredInterface)
		if err == nil && ip != "" {
			return ip
		}
		log.Printf("[Network] Warning: Interface %s not found or no IP. Falling back to auto detection.", preferredInterface)
	}

	// 2. 自动探测 (回退逻辑)
	conn, err := net.Dial("udp", "8.8.8.8:80")
	if err != nil {
		return "127.0.0.1"
	}
	defer conn.Close()

	localAddr := conn.LocalAddr().(*net.UDPAddr)
	ip := localAddr.IP.String()

	if strings.Contains(ip, ":") {
		return "127.0.0.1"
	}
	return ip
}
