package service

import (
	"fmt"
	"log"
	"net"
	"os/exec"
	"ros-ground-control/agent/internal/connect"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"
	"strconv"
	"strings"
	"sync"
	"time"
)

// 定义系统服务的固定 ID
const (
	IDRosCore  = "sys-roscore"
	IDFoxglove = "sys-foxglove"
)

type ROSManager struct {
	mu           sync.RWMutex // 互斥锁保护状态位
	isRestarting bool
	lastKnownIP  string
}

var GlobalROSManager = &ROSManager{}

// --- 网络监控器 ---
func (rm *ROSManager) StartNetworkMonitor() {
	cfg := config.GetConfig()
	rm.lastKnownIP = utils.GetOutboundIP(cfg.NetworkInterface)
	log.Printf("[NetMon] Started. Interface: %s, IP: %s", cfg.NetworkInterface, rm.lastKnownIP)

	go func() {
		ticker := time.NewTicker(3 * time.Second) // 每 3 秒检查一次
		defer ticker.Stop()

		for range ticker.C {
			currentCfg := config.GetConfig()
			currentIP := utils.GetOutboundIP(currentCfg.NetworkInterface)

			// 忽略 127.0.0.1 (可能是网络切换中间状态，网卡掉线瞬间)
			if currentIP == "127.0.0.1" {
				continue
			}

			// 如果 IP 发生了变化
			if currentIP != rm.lastKnownIP {
				log.Printf("[NetMon] ⚠️ IP Change detected! %s -> %s", rm.lastKnownIP, currentIP)

				rm.lastKnownIP = currentIP

				// 通知前端（可选，让前端弹出“网络切换中...”的提示）
				connect.GlobalHub.Broadcast(connect.LogMessage{
					ProcessID: "system",
					Stream:    "system",
					Data:      fmt.Sprintf("Network change detected (%s). Rebooting ROS system...", currentIP),
				})

				// 触发全系统重启
				// 注意：这里复用 handleCoreCrash 的逻辑，因为它本质上就是一次全清理+全重建
				go rm.handleCoreCrash()
			}
		}
	}()
}

// --- 状态管理方法 ---
func (rm *ROSManager) setRestarting(state bool) {
	rm.mu.Lock()
	defer rm.mu.Unlock()
	rm.isRestarting = state
}

func (rm *ROSManager) isSystemRestarting() bool {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.isRestarting
}

// 辅助函数：清理僵尸进程
func cleanupZombie(procName string) {
	// 使用 pkill -f 模糊匹配命令行，确保杀掉之前的残留进程
	// 忽略错误，因为如果没有找到进程，pkill 会返回 error，这对我们来说是好事
	exec.Command("pkill", "-f", procName).Run()
}

// 辅助函数：查找进程 PID
func findPidByName(name string) (int, error) {
	// 使用 pgrep -n (newest) 查找最近启动的匹配进程
	// roscore 实际上是 python 脚本，真正占用端口的核心进程是 "rosmaster"
	cmd := exec.Command("pgrep", "-n", "-f", name)
	output, err := cmd.Output()
	if err != nil {
		return 0, err
	}
	pidStr := strings.TrimSpace(string(output))
	return strconv.Atoi(pidStr)
}

// 辅助函数：统一生成配置
// 根据 ID 生成对应的 ProcessConfig，并注入当前最新的 IP
func (rm *ROSManager) generateServiceConfig(id string) (ProcessConfig, error) {
	// 读取配置
	cfg := config.GetConfig()
	// 1. 传入指定的网卡名称
	hostIP := utils.GetOutboundIP(cfg.NetworkInterface)
	// 2. 构造 ROS 基础环境变量
	commonEnv := []string{
		fmt.Sprintf("ROS_IP=%s", hostIP),
		fmt.Sprintf("ROS_HOSTNAME=%s", hostIP),
		"ROS_MASTER_URI=http://localhost:11311",
	}
	// 3. 基础模板
	baseCfg := ProcessConfig{
		ID:          id,
		Env:         commonEnv,
		SetupScript: "/opt/ros/noetic/setup.bash",
	}
	switch id {
	case IDRosCore:
		baseCfg.CmdStr = "roscore"
		baseCfg.Args = []string{}
	case IDFoxglove:
		baseCfg.CmdStr = "roslaunch"
		baseCfg.Args = []string{
			"foxglove_bridge",
			"foxglove_bridge.launch",
			"port:=8765",
			"address:=0.0.0.0",
		}
	default:
		return ProcessConfig{}, fmt.Errorf("unknown service id: %s", id)
	}
	return baseCfg, nil
}

// 初始化：注册回调函数
func init() {
	GlobalProcManager.SetExitCallback(func(id string, manual bool) {
		// 如果是用户手动停止的，什么都不做
		if manual {
			return
		}

		// 检查是否处于全局重启状态
		if GlobalROSManager.isSystemRestarting() {
			log.Printf("[ROS-Watchdog] System is restarting. Ignoring cascade exit of: %s", id)
			return
		}

		log.Printf("[ROS-Watchdog] Process %s exited unexpectedly!", id)

		// 根据死掉的进程 ID 决定策略
		switch id {
		case IDRosCore:
			// 只有 roscore 挂了才触发“全家桶重启”
			// 使用 Goroutine 执行，防止阻塞 Watchdog 回调线程
			go GlobalROSManager.handleCoreCrash()
		case IDFoxglove:
			// 只有在非全局重启模式下，才允许单点重启
			time.Sleep(2 * time.Second)
			GlobalROSManager.RestartService(id)
		default:
			// 其他普通 ROS 进程（如 rosbag, temporary node）不自动重启
			log.Printf("[ROS-Watchdog] Ignoring non-system process: %s", id)
		}
	})
}

// 核心崩溃处理逻辑 (级联重启)
func (rm *ROSManager) handleCoreCrash() {
	// 双重检查：防止并发调用导致多次重启
	if rm.isSystemRestarting() {
		return
	}

	log.Println("[ROS-Watchdog] CRITICAL: roscore crashed. Initiating full system restart...")

	// 进入维护模式
	// 任何由此引发的子进程崩溃都会被 Watchdog 忽略
	rm.setRestarting(true)
	defer func() {
		log.Println("[ROS-Watchdog] ✅ Maintenance finished. System is responding.")
		rm.setRestarting(false)

		// 恢复服务 (StartDefaultServices 内部是异步的，所以这里可以直接调)
		rm.StartDefaultServices()
	}()

	// 3. 强制清理所有子服务
	// 使用并发清理以加快速度
	var wg sync.WaitGroup
	wg.Add(2)
	go func() { defer wg.Done(); rm.StopService(IDFoxglove) }()
	wg.Wait()
	// 停止自动压缩监控
	GlobalCompressor.Stop()

	// 4. 等待端口释放
	log.Println("[ROS-Watchdog] Waiting for ports to clear...")
	time.Sleep(3 * time.Second) // 增加到 5s 确保 TIME_WAIT 结束
}

// StartDefaultServices 启动默认服务
func (rm *ROSManager) StartDefaultServices() {
	go func() {
		log.Println("[ROS] Initializing default services...")

		// 1. 检查 roscore 是否存在
		if rm.isPortOpen("localhost", "11311") {
			log.Println("[ROS] roscore is already running (external).")
			// --- 尝试接管 ---
			// 查找 rosmaster 的 PID (因为 roscore 只是脚本，rosmaster 才是本体)
			pid, err := findPidByName("rosmaster")
			if err == nil && pid > 0 {
				log.Printf("[ROS] Adopting existing rosmaster (PID: %d)", pid)
				// 将其加入 ProcessManager，ID 设为 sys-roscore
				GlobalProcManager.AdoptExternalProcess(IDRosCore, "roscore (external)", pid)
			} else {
				log.Println("[ROS] Warning: Port 11311 open but could not find rosmaster PID.")
			}
		} else {
			// 端口未占用，执行清理并启动新的
			cleanupZombie("roscore")
			cleanupZombie("rosmaster")
			cfg, _ := rm.generateServiceConfig(IDRosCore)
			if err := GlobalProcManager.StartProcess(cfg); err != nil {
				log.Printf("[ROS] Failed to start roscore: %v", err)
				return
			}
			log.Println("[ROS] Waiting for roscore...")
			if !rm.waitForPort("localhost", "11311", 10*time.Second) {
				log.Println("[ROS] Error: roscore timeout!")
				return
			}
		}
		log.Println("[ROS] roscore is ready. Starting bridges...")
		// 2. 清理并启动 Foxglove Bridge
		cleanupZombie("foxglove_bridge")
		// 稍微等待操作系统释放端口资源
		time.Sleep(1 * time.Second)
		// 启动 Foxglove Bridge
		if cfg, err := rm.generateServiceConfig(IDFoxglove); err == nil {
			GlobalProcManager.StartProcess(cfg)
		}
		// 3. 启动/重置自动压缩管理器
		GlobalCompressor.Reset() // 防止重启时的残留
		GlobalCompressor.Start() // 启动后台扫描协程
		log.Println("[ROS] ros is ready.")
	}()
}

// StopService 停止服务
func (rm *ROSManager) StopService(id string) error {
	return GlobalProcManager.StopProcess(id)
}

// RestartService 重启服务
func (rm *ROSManager) RestartService(id string) error {
	// 1. 先停止
	log.Printf("[ROS] Restarting service: %s", id)
	rm.StopService(id) // 忽略错误，可能本来就没运行
	// 2. 等待资源释放
	time.Sleep(1 * time.Second)
	// 3. 重新生成配置 (这会重新获取 IP)
	cfg, err := rm.generateServiceConfig(id)
	if err != nil {
		return err
	}
	// 4. 再次启动
	return GlobalProcManager.StartProcess(cfg)
}

// --- 端口检测工具保持不变 ---
func (rm *ROSManager) isPortOpen(host, port string) bool {
	timeout := time.Second
	conn, err := net.DialTimeout("tcp", net.JoinHostPort(host, port), timeout)
	if err != nil {
		return false
	}
	if conn != nil {
		conn.Close()
		return true
	}
	return false
}

func (rm *ROSManager) waitForPort(host, port string, timeout time.Duration) bool {
	start := time.Now()
	for {
		if rm.isPortOpen(host, port) {
			return true
		}
		if time.Since(start) > timeout {
			return false
		}
		time.Sleep(500 * time.Millisecond)
	}
}
