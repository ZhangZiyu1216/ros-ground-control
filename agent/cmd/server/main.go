package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"syscall"
	"text/template"

	"ros-ground-control/agent/internal/api"
	"ros-ground-control/agent/internal/connect"
	"ros-ground-control/agent/internal/security"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"
)

// Systemd 服务模板
const serviceTemplate = `[Unit]
Description=ROS Ground Control Agent
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User={{.User}}
ExecStart={{.Path}}
WorkingDirectory={{.Dir}}
Restart=always
RestartSec=3
Environment=GIN_MODE=release

[Install]
WantedBy=multi-user.target
`

func installService() {
	exePath, _ := os.Executable()
	absPath, _ := filepath.Abs(exePath)
	dir := filepath.Dir(absPath)
	user := os.Getenv("USER")

	if user == "" {
		user = "root" // fallback
	}

	data := struct {
		User string
		Path string
		Dir  string
	}{
		User: user,
		Path: absPath,
		Dir:  dir,
	}

	// 1. 写入 /etc/systemd/system/ros-agent.service
	// 注意：这需要 sudo 权限
	f, err := os.Create("/etc/systemd/system/ros-agent.service")
	if err != nil {
		log.Fatalf("Failed to create service file (need sudo?): %v", err)
	}
	defer f.Close()

	tmpl, _ := template.New("service").Parse(serviceTemplate)
	tmpl.Execute(f, data)

	fmt.Println("[Install] Service file created at /etc/systemd/system/ros-agent.service")

	// 2. 重新加载 daemon 并启用服务
	exec.Command("systemctl", "daemon-reload").Run()
	exec.Command("systemctl", "enable", "ros-agent").Run()
	exec.Command("systemctl", "start", "ros-agent").Run()

	fmt.Println("[Install] Service enabled and started!")
	fmt.Println("[Install] Check status with: systemctl status ros-agent")
}

func main() {
	// 定义命令行参数
	installFlag := flag.Bool("install", false, "Install systemd service and enable auto-start")
	flag.Parse()

	// 如果是安装模式
	if *installFlag {
		installService()
		return // 安装完直接退出
	}

	log.Println("Starting ROS Ground Control Agent...")
	// 加载配置
	if err := config.LoadConfig(); err != nil {
		log.Fatalf("Failed to load config: %v", err)
	}
	// 生成密钥
	security.InitKeys()
	// 初始化文件监控
	if err := service.InitFileWatcher(); err != nil {
		log.Fatalf("Failed to init file watcher: %v", err)
	}

	hostname, err := os.Hostname()
	if err != nil {
		log.Printf("Warning: Could not get hostname: %v. Using default 'ros-agent'", err)
		hostname = "ros-agent"
	}
	log.Printf("[Agent] Detected hostname: %s", hostname)

	// 1. 启动 mDNS 服务 (让 Client 能发现我们)
	cfg := config.GetConfig()
	preferredIP := utils.GetOutboundIP(cfg.NetworkInterface)
	log.Printf("[Agent] Detected hostname: %s, Preferred IP: %s", hostname, preferredIP)

	mdnsService, err := connect.StartMDNS(hostname, 8080, cfg.AgentID, preferredIP)
	if err != nil {
		log.Fatalf("Failed to start mDNS: %v", err)
	}
	defer mdnsService.Shutdown()

	// 启动 WebSocket Hub 广播循环
	go connect.GlobalHub.Run()
	// 启动硬件监控
	service.GlobalMonitor.StartMonitor()
	// 启动网络监控
	service.GlobalROSManager.StartNetworkMonitor()
	// 现在不自动启动 ROS 基础设施
	//service.GlobalROSManager.StartDefaultServices()
	log.Println("[Agent] ROS Stack is in standby mode. Waiting for client request.")

	// 2. 初始化 HTTP 路由
	r := api.NewRouter()

	// 3. 启动 Web Server (在一个 Goroutine 中运行，防止阻塞)
	go func() {
		if err := r.Run(":8080"); err != nil {
			log.Fatalf("Server startup failed: %v", err)
		}
	}()

	// 4. 优雅退出处理 (捕获 Ctrl+C)
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down agent...")
}
