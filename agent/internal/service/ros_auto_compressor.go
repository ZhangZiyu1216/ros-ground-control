package service

import (
	"fmt"
	"log"
	"os"
	"ros-ground-control/agent/pkg/utils"
	"strings"
	"sync"
	"time"
)

// 移除硬编码的 ImageMsgType，使用变量以便调试
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

// ... Start, Stop, Shutdown, Reset 保持不变 ...
// 这里为了篇幅省略，请保留原有的 Shutdown, Reset, Start 方法代码
// 仅需确保 Start 调用的是新的 loop

func (ac *AutoCompressor) Start() {
	// 防止重复启动
	select {
	case <-ac.stopChan:
		ac.stopChan = make(chan struct{})
	default:
	}
	go ac.loop()
}

func (ac *AutoCompressor) Shutdown() {
	select {
	case <-ac.stopChan:
	default:
		close(ac.stopChan)
	}

	ac.mu.Lock()
	defer ac.mu.Unlock()

	for topic, procID := range ac.runningCompressors {
		log.Printf("[AutoCompressor] Shutdown: stopping %s (%s)", procID, topic)
		GlobalProcManager.StopProcess(procID)
	}
	ac.runningCompressors = make(map[string]string)
}

func (ac *AutoCompressor) Reset() {
	ac.mu.Lock()
	defer ac.mu.Unlock()
	ac.runningCompressors = make(map[string]string)
}

// ---------------------------------------------------------

func (ac *AutoCompressor) loop() {
	// 简单的重试延迟
	log.Println("[AutoCompressor] Started. Monitoring for Raw Image topics...")

	// 这里的间隔其实建议从 config 读取，为了调试先硬编码 3秒
	ticker := time.NewTicker(3 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ac.stopChan:
			return
		case <-ticker.C:
			ac.scanAndProcess()
		}
	}
}

func (ac *AutoCompressor) scanAndProcess() {
	// 1. 确定 Master URI
	// 优先读取环境变量，因为 roscore 启动时可能绑定了非 localhost IP
	masterURI := os.Getenv("ROS_MASTER_URI")
	if masterURI == "" {
		masterURI = "http://localhost:11311"
	}

	// 2. 连接 Master
	client, err := utils.NewROSMasterClient(masterURI)
	if err != nil {
		// 这里虽然静默，但在调试阶段最好打印一下，确认是否连不上
		// log.Printf("[AutoCompressor] DEBUG: Connect master failed: %v", err)
		return
	}

	// 3. 获取所有话题类型
	topicTypes, err := client.GetTopicTypes()
	if err != nil {
		// 只有在真的出错时打印，连接拒绝通常不打印
		return
	}

	// 4. 获取活跃发布者
	publishedTopics, err := client.GetPublishedTopics()
	if err != nil {
		return
	}

	// --- 调试日志：每隔几次打印一次发现的图像话题，或者只打印新发现的 ---
	// 实际生产中可以去掉
	foundImageCount := 0

	activeRawTopics := make(map[string]bool)

	for topic, typ := range topicTypes {
		// 必须是 sensor_msgs/Image
		if typ == ImageMsgType {
			foundImageCount++

			// 必须有发布者
			if !publishedTopics[topic] {
				// log.Printf("[AutoCompressor] Ignored %s (No publisher)", topic)
				continue
			}

			activeRawTopics[topic] = true

			// 检查是否已有压缩版
			compressedTopic := topic + "/compressed"

			// 关键修正：检查压缩版是否有发布者，而不仅仅是类型存在
			if publishedTopics[compressedTopic] {
				// 已经有压缩版在跑了
				continue
			}

			// 启动压缩
			ac.startCompressor(topic)
		}
	}

	if foundImageCount > 0 {
		// 这是一个极其有用的调试信息，证明逻辑跑到了这里
		// log.Printf("[AutoCompressor] DEBUG: Found %d raw image topics", foundImageCount)
	}

	// 5. 清理无效进程
	ac.mu.Lock()
	defer ac.mu.Unlock()

	for topic, procID := range ac.runningCompressors {
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

	if _, exists := ac.runningCompressors[topic]; exists {
		return
	}

	safeName := strings.ReplaceAll(strings.TrimPrefix(topic, "/"), "/", "_")
	procID := fmt.Sprintf("auto-comp-%s", safeName)

	// 构造命令
	// 显式指定 namespace，防止重名干扰
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
	cfg, _ := GlobalROSManager.GenerateConfigStub(IDRosCore)

	procCfg := ProcessConfig{
		ID:          procID,
		CmdStr:      cmdStr,
		Args:        args,
		Env:         cfg.Env,
		SetupScript: cfg.SetupScript,
	}

	log.Printf("[AutoCompressor] 📸 Starting compressor for: %s -> %s", topic, procID)

	if err := GlobalProcManager.StartProcess(procCfg); err != nil {
		log.Printf("[AutoCompressor] ❌ Failed to start compressor: %v", err)
	} else {
		ac.runningCompressors[topic] = procID
	}
}
