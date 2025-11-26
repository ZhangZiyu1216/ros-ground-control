package connect

import (
	"log"
	"sync"

	"github.com/grandcat/zeroconf"
)

var (
	server *zeroconf.Server
	mu     sync.Mutex
	// 缓存启动参数，以便重启时复用
	cachedHostname string
	cachedPort     int
	cachedAgentID  string
)

// StartMDNS 启动 mDNS 服务
func StartMDNS(hostname string, port int, agentID string, preferredIP string) error {
	mu.Lock()
	defer mu.Unlock()

	// 缓存参数
	cachedHostname = hostname
	cachedPort = port
	cachedAgentID = agentID

	return startInternal(preferredIP)
}

// UpdateMDNS 使用新 IP 重启 mDNS 服务
func UpdateMDNS(newIP string) {
	mu.Lock()
	defer mu.Unlock()

	log.Printf("[mDNS] Updating broadcast IP to: %s", newIP)

	if server != nil {
		server.Shutdown()
	}

	if err := startInternal(newIP); err != nil {
		log.Printf("[mDNS] Failed to restart service: %v", err)
	}
}

// 内部启动逻辑
func startInternal(ip string) error {
	meta := []string{
		"version=2.0",
		"hostname=" + cachedHostname,
		"id=" + cachedAgentID,
		"ip=" + ip, // 广播新 IP
	}

	var err error
	server, err = zeroconf.Register(
		cachedHostname,
		"_ros-agent._tcp",
		"local.",
		cachedPort,
		meta,
		nil,
	)
	return err
}

// Shutdown 关闭服务
func Shutdown() {
	mu.Lock()
	defer mu.Unlock()
	if server != nil {
		server.Shutdown()
	}
}
