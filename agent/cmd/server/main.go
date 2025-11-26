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

	"ros-ground-control/agent/internal/api"
	"ros-ground-control/agent/internal/connect"
	"ros-ground-control/agent/internal/security"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"
)

func installService() {
	// 1. 获取当前运行的用户 (Sudo 执行时，SUDO_USER 是实际用户，USER 是 root)
	realUser := os.Getenv("SUDO_USER")
	if realUser == "" {
		log.Fatal("Error: Please run with sudo (e.g., sudo ./ros-agent -install)")
	}

	// 2. 确定安装目标路径
	const installDir = "/usr/local/bin"
	const binaryName = "ros-ground-control" // 统一安装后的名字
	targetPath := filepath.Join(installDir, binaryName)

	// 3. 自我复制 (Copy Self)
	selfPath, _ := os.Executable()

	fmt.Printf("[Install] Installing binary to %s...\n", targetPath)

	// 读取自己
	input, err := os.ReadFile(selfPath)
	if err != nil {
		log.Fatalf("Failed to read self: %v", err)
	}
	// 写入系统目录
	if err := os.WriteFile(targetPath, input, 0755); err != nil {
		log.Fatalf("Failed to copy binary to %s: %v", targetPath, err)
	}

	// 4. 生成 Systemd 文件
	// 注意：ExecStart 指向新的系统路径
	// 注意：User 设置为 realUser，确保以普通用户身份运行 (读取 ~/.config 和 ROS 环境)
	const serviceContent = `[Unit]
Description=ROS Ground Control Agent
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=%s
ExecStart=%s
Restart=always
RestartSec=3
Environment=GIN_MODE=release

[Install]
WantedBy=multi-user.target
`
	serviceFile := fmt.Sprintf(serviceContent, realUser, targetPath)

	if err := os.WriteFile("/etc/systemd/system/ros-agent.service", []byte(serviceFile), 0644); err != nil {
		log.Fatalf("Failed to write service file: %v", err)
	}

	// 5. 启用服务
	fmt.Println("[Install] Enabling systemd service...")
	exec.Command("systemctl", "daemon-reload").Run()
	exec.Command("systemctl", "enable", "ros-agent").Run()
	exec.Command("systemctl", "restart", "ros-agent").Run()

	fmt.Println("[Install] ✅ Success! Agent is running.")
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

	if err := connect.StartMDNS(hostname, 8080, cfg.AgentID, preferredIP); err != nil {
		log.Printf("Failed to start mDNS: %v", err)
	}
	// 退出时清理
	defer connect.Shutdown()

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
