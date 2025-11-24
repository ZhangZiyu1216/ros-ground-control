package connect

import (
	"log"

	"github.com/grandcat/zeroconf"
)

// StartMDNS 注册 _ros-agent._tcp 服务
func StartMDNS(hostname string, port int, agentID string, preferredIP string) (*zeroconf.Server, error) {
	// Text 记录可以携带额外信息
	meta := []string{"version=2.0", "hostname=" + hostname, "id=" + agentID, "ip=" + preferredIP}

	server, err := zeroconf.Register(
		hostname,          // Instance name
		"_ros-agent._tcp", // Service type
		"local.",          // Domain
		port,              // Port
		meta,              // TXT records
		nil,               // Interfaces (nil means all)
	)

	if err != nil {
		return nil, err
	}

	log.Printf("mDNS registered: %s (ID: %s, IP: %s)", hostname, agentID, preferredIP)
	return server, nil
}
