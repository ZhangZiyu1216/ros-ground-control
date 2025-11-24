package config

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"

	"ros-ground-control/agent/pkg/utils"

	"github.com/google/uuid"
)

// AgentConfig 定义配置结构
type AgentConfig struct {
	AgentID string `json:"agent_id"`
	// NetworkInterface 指定网卡名称，如 "wlan0", "eth0"。
	// 如果为空或 "auto"，则使用自动探测
	NetworkInterface string `json:"network_interface"`
	CompressorPollMs int    `json:"compressor_poll_ms"`
}

var (
	GlobalConfig *AgentConfig
	mu           sync.RWMutex
)

// getConfigPath 获取标准配置文件路径
func getConfigPath() (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	// ~/.config/ros-ground-control/
	configDir := filepath.Join(home, ".config", "ros-ground-control")

	// 确保目录存在
	if err := os.MkdirAll(configDir, 0755); err != nil {
		return "", err
	}

	return filepath.Join(configDir, "config.json"), nil
}

// LoadConfig 加载配置，如果文件不存在则创建默认配置
func LoadConfig() error {
	mu.Lock()
	defer mu.Unlock()
	path, err := getConfigPath()
	if err != nil {
		return err
	}
	// 1. 初始化默认配置
	GlobalConfig = &AgentConfig{
		NetworkInterface: "auto",
		CompressorPollMs: 5000,
		AgentID:          "", // 先置空
	}
	// 2. 尝试读取并解析文件 (如果文件存在)
	// 我们先读文件，这样能获取到 network_interface 等其他用户配置
	fileExisted := false
	data, err := os.ReadFile(path)
	if err == nil {
		fileExisted = true
		if err := json.Unmarshal(data, GlobalConfig); err != nil {
		}
	}
	// 3. 获取当前的物理硬件 ID (Source of Truth)
	hwID := utils.GetHardwareID()
	// 判断硬件 ID 是否有效 (排除我们在 utils 里定义的错误字符串)
	isHwValid := hwID != "" && hwID != "no-physical-mac" && hwID != "unknown-mac-error"
	// 标记是否需要回写磁盘
	needSave := !fileExisted
	// 4. 核心逻辑：强制覆盖 (Enforce Hardware ID)
	if isHwValid {
		// 只要硬件 ID 有效，就以它为准
		// 如果文件里的 ID 和硬件 ID 不一致（比如拷贝了别人的配置，或者 ID 为空），强制覆盖
		if GlobalConfig.AgentID != hwID {
			GlobalConfig.AgentID = hwID
			needSave = true
		}
	} else {
		// 5. 兜底逻辑：无法获取硬件 ID
		// 只有在硬件 ID 获取失败，且配置文件里也没有 ID 的情况下，才生成随机 UUID
		if GlobalConfig.AgentID == "" {
			GlobalConfig.AgentID = uuid.New().String()
			needSave = true
		}
		// 如果配置文件里已有 ID (虽然不是硬件ID)，则保持不变，防止随机 UUID 每次重启都变
	}
	// 6. 如果数据有变动（或文件本就不存在），立即持久化
	if needSave {
		return saveConfigInternal(path)
	}
	return nil
}

// GetConfig 获取当前配置的副本
func GetConfig() AgentConfig {
	mu.RLock()
	defer mu.RUnlock()
	return *GlobalConfig
}

// UpdateConfig 更新配置并持久化
func UpdateConfig(newCfg AgentConfig) error {
	mu.Lock()
	defer mu.Unlock()
	GlobalConfig = &newCfg

	path, err := getConfigPath()
	if err != nil {
		return err
	}
	return saveConfigInternal(path)
}

func saveConfigInternal(path string) error {
	data, err := json.MarshalIndent(GlobalConfig, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}
