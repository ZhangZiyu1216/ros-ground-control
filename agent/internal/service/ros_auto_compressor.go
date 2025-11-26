package service

import (
	"fmt"
	"log"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"
	"strings"
	"sync"
	"time"
)

const (
	ImageMsgType      = "sensor_msgs/Image"
	CompressedMsgType = "sensor_msgs/CompressedImage"
)

type AutoCompressor struct {
	runningCompressors map[string]string // map[TopicName]ProcessID
	mu                 sync.Mutex
	stopChan           chan struct{}
}

var GlobalCompressor = &AutoCompressor{
	runningCompressors: make(map[string]string),
	stopChan:           make(chan struct{}),
}

func (ac *AutoCompressor) Start() {
	go ac.loop()
}

// 增加 Shutdown 方法
func (ac *AutoCompressor) Shutdown() {
	// 1. 停止循环
	select {
	case <-ac.stopChan:
		// 已经关闭了，不做处理
	default:
		close(ac.stopChan)
	}
	ac.mu.Lock()
	defer ac.mu.Unlock()
	// 2. 杀死所有由它启动的进程
	for topic, procID := range ac.runningCompressors {
		log.Printf("[AutoCompressor] Shutdown: stopping %s (%s)", procID, topic)
		GlobalProcManager.StopProcess(procID)
	}
	// 3. 清空列表
	ac.runningCompressors = make(map[string]string)
}

func (ac *AutoCompressor) loop() {
	log.Println("[AutoCompressor] Started. Monitoring for Raw Image topics...")
	for {
		// 1. 获取当前配置的间隔时间
		cfg := config.GetConfig()
		intervalMs := max(cfg.CompressorPollMs, 100)
		duration := time.Duration(intervalMs) * time.Millisecond

		// 2. 创建一个 Timer (一次性定时器)
		timer := time.NewTimer(duration)

		select {
		case <-ac.stopChan:
			// 退出前停止 timer，虽然这里已经不重要了
			timer.Stop()
			return

		case <-timer.C:
			// 时间到了，执行扫描
			ac.scanAndProcess()
			// 循环会回到开头，重新读取配置，重新创建 Timer
		}
	}
}

func (ac *AutoCompressor) scanAndProcess() {
	// 1. 连接 Master
	// 假设 Master 在本地，端口固定。如果在多机环境，这里应该从 Env 读取 ROS_MASTER_URI
	client, err := utils.NewROSMasterClient("http://localhost:11311")
	if err != nil {
		// Master 可能还没起，忽略
		return
	}
	// 2. 获取所有话题类型
	topicTypes, err := client.GetTopicTypes()
	if err != nil {
		log.Printf("[AutoCompressor] Failed to get topic types: %v", err)
		return
	}
	// 3. 获取有发布者的话题 (关键修复！)
	// 只有在这个列表里的话题，才是真正“活着”的源数据
	publishedTopics, err := client.GetPublishedTopics()
	if err != nil {
		return
	}
	// 4. 找出需要压缩的话题
	// 条件: (Type == Image) AND (Has Publisher) AND (No Compressed Topic)
	activeRawTopics := make(map[string]bool)

	for topic, typ := range topicTypes {
		// 必须是 Image 类型，且必须有活跃的发布者
		if typ == ImageMsgType && publishedTopics[topic] {
			activeRawTopics[topic] = true
			// 检查是否已有压缩版
			compressedTopic := topic + "/compressed"
			// 注意：我们要检查 compressedTopic 是否有发布者，而不仅仅是类型存在
			// 如果 master 里有类型记录但没发布者，说明之前的压缩进程挂了，我们需要重启它
			if publishedTopics[compressedTopic] {
				continue
			}
			// 启动压缩
			ac.startCompressor(topic)
		}
	}

	// 5. 清理无效进程
	ac.mu.Lock()
	defer ac.mu.Unlock()

	for topic, procID := range ac.runningCompressors {
		// 如果话题不再活跃（没有发布者了），杀掉压缩进程
		if !activeRawTopics[topic] {
			log.Printf("[AutoCompressor] Topic %s lost publisher. Stopping compressor %s", topic, procID)
			GlobalProcManager.StopProcess(procID)
			delete(ac.runningCompressors, topic)
		}
	}
}

func (ac *AutoCompressor) startCompressor(topic string) {
	ac.mu.Lock()
	defer ac.mu.Unlock()

	// 双重检查
	if _, exists := ac.runningCompressors[topic]; exists {
		return
	}

	// 生成唯一 ID
	// 将话题名中的 / 替换为 _ 以便作为 ID
	safeName := strings.ReplaceAll(strings.TrimPrefix(topic, "/"), "/", "_")
	procID := fmt.Sprintf("auto-comp-%s", safeName)

	// 构造命令
	// rosrun image_transport republish raw in:=<topic> compressed out:=<topic>
	cmdStr := "rosrun"
	args := []string{
		"image_transport",
		"republish",
		"raw",
		fmt.Sprintf("in:=%s", topic),
		"compressed",
		fmt.Sprintf("out:=%s", topic),
	}

	// 获取环境
	cfg, _ := GlobalROSManager.GenerateConfigStub(IDRosCore) // 借用 roscore 的环境配置

	procCfg := ProcessConfig{
		ID:          procID,
		CmdStr:      cmdStr,
		Args:        args,
		Env:         cfg.Env,
		SetupScript: cfg.SetupScript, // 使用系统或 Vendor 环境
	}

	log.Printf("[AutoCompressor] Detected raw image: %s. Starting compressor...", topic)
	if err := GlobalProcManager.StartProcess(procCfg); err != nil {
		log.Printf("[AutoCompressor] Failed to start compressor for %s: %v", topic, err)
	} else {
		ac.runningCompressors[topic] = procID
	}
}

// Reset 清空状态 (当 roscore 重启时调用)
func (ac *AutoCompressor) Reset() {
	ac.mu.Lock()
	defer ac.mu.Unlock()
	// 注意：ProcessManager 会自己处理 handleCoreCrash 时的进程清理
	// 我们只需要清空 map
	ac.runningCompressors = make(map[string]string)
}
