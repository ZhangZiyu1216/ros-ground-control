package connect

import (
	"log"

	"github.com/grandcat/zeroconf"
)

// StartMDNS 注册 _ros-agent._tcp 服务
func StartMDNS(hostname string, port int) (*zeroconf.Server, error) {
	// Text 记录可以携带额外信息
	meta := []string{"version=2.0", "hostname=" + hostname}

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

	log.Printf("mDNS service registered: %s._ros-agent._tcp local:%d", hostname, port)
	return server, nil
}
