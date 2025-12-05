package main

import (
	"flag"
	"fmt"
	"log"
	"net"
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
Environment=GOGC=200
OOMScoreAdjust=-1000

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

// 获取单例锁
// 返回值: unlock 函数 (用于退出时释放), error
func acquireInstanceLock() (func(), error) {
	// 修改路径：使用系统临时目录，实现跨用户互斥
	// 也就是 /tmp/ros-ground-control.lock
	lockPath := filepath.Join(os.TempDir(), "ros-ground-control.lock")

	// 1. 尝试打开文件
	// 使用 O_RDWR (读写) 模式，这是 Flock 所需的
	// 使用 0666 权限尝试创建 (rw-rw-rw-)，这样不同用户之间理论上可以互相 check 锁
	// 但受限于 umask，实际创建出来的可能是 644 或 664
	file, err := os.OpenFile(lockPath, os.O_CREATE|os.O_RDWR, 0666)

	if err != nil {
		// 情况 A: 另一个用户已经创建了该文件，且当前用户没有写权限
		// 比如 User A (root) 创建了锁，User B (ubuntu) 尝试打开
		if os.IsPermission(err) {
			return nil, fmt.Errorf("another instance is running (owned by another user)")
		}
		return nil, fmt.Errorf("failed to open lock file: %v", err)
	}

	// 尝试修改权限，确保其他用户也能打开它以便检查锁状态
	// 这一步可能会失败（如果文件不是我创建的），忽略错误
	os.Chmod(lockPath, 0666)

	// 2. 尝试获取独占锁
	err = syscall.Flock(int(file.Fd()), syscall.LOCK_EX|syscall.LOCK_NB)
	if err != nil {
		file.Close()
		// 情况 B: 同一个用户启动了两个实例
		if err == syscall.EWOULDBLOCK {
			return nil, fmt.Errorf("another instance is running (locked)")
		}
		return nil, fmt.Errorf("flock failed: %v", err)
	}

	// 3. 写入 PID
	file.Truncate(0)
	fmt.Fprintf(file, "%d", os.Getpid())

	unlock := func() {
		syscall.Flock(int(file.Fd()), syscall.LOCK_UN)
		file.Close()
		// 注意：公共目录下的锁文件最好不要删除
		// 因为删除操作可能存在竞态条件，且如果其他用户正在等待锁，删除会导致诡异行为
		// 留在 /tmp 下让系统自动清理即可
	}
	return unlock, nil
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

	// 检查单例锁
	// 注意：必须在 LoadConfig 之前或之后立即执行，要在启动任何服务之前
	unlock, err := acquireInstanceLock()
	if err != nil {
		// 打印红色错误信息并退出
		log.Fatalf("\n\n[ERROR] Start Failed: %v\nRun 'systemctl status ros-agent' to check if daemon is active.\n\n", err)
	}
	// 程序退出时解锁
	defer unlock()

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

	// 1. 启动 mDNS 服务 (让 Client 能发现我们)
	cfg := config.GetConfig()

	// 1. 创建监听器 (Listener)
	// 如果 cfg.Port 是 0，系统会自动分配一个空闲端口
	listener, err := net.Listen("tcp", fmt.Sprintf(":%d", cfg.Port))
	if err != nil {
		log.Fatalf("Failed to listen on port %d: %v", cfg.Port, err)
	}

	// 2. 获取实际分配的端口
	// 这一步对于 Port=0 至关重要
	realPort := listener.Addr().(*net.TCPAddr).Port
	log.Printf("[Agent] Listening on port: %d", realPort)

	// 3. 获取 Hostname
	hostname, err := os.Hostname()
	if err != nil {
		hostname = "ros-agent"
	}
	log.Printf("[Agent] Detected hostname: %s", hostname)

	// 4. 获取 IP
	preferredIP := utils.GetOutboundIP(cfg.NetworkInterface)

	log.Printf("[Agent] Discovery: %s @ %s:%d", hostname, preferredIP, realPort)

	// 5. 启动 mDNS (传入 真实端口 realPort)
	// 这样前端扫描到的 Service 就会包含正确的动态端口
	if err := connect.StartMDNS(hostname, realPort, cfg.AgentID, preferredIP); err != nil {
		log.Printf("Failed to start mDNS: %v", err)
	}
	defer connect.Shutdown()

	// 启动 WebSocket Hub 广播循环
	go connect.GlobalHub.Run()
	// 启动硬件监控
	service.GlobalMonitor.StartMonitor()
	// 启动网络监控
	service.GlobalROSManager.StartNetworkMonitor()
	// 启动 Roscore 巡检
	service.GlobalROSManager.StartCoreMonitor()
	// 现在不自动启动 ROS 基础设施
	//service.GlobalROSManager.StartDefaultServices()
	log.Println("[Agent] ROS Stack is in standby mode. Waiting for client request.")

	// 2. 初始化 HTTP 路由
	r := api.NewRouter()

	// 3. 启动 Web Server (在一个 Goroutine 中运行，防止阻塞)
	go func() {
		if err := r.RunListener(listener); err != nil {
			log.Fatalf("Server startup failed: %v", err)
		}
	}()

	// 4. 优雅退出处理 (捕获 Ctrl+C)
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down agent...")
}
