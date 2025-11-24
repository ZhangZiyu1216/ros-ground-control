package config

import (
	"encoding/json"
	"os"
	"sync"
)

const ConfigFileName = "config.json"

// AgentConfig 定义配置结构
type AgentConfig struct {
	// NetworkInterface 指定网卡名称，如 "wlan0", "eth0"。
	// 如果为空或 "auto"，则使用自动探测
	NetworkInterface string `json:"network_interface"`
	CompressorPollMs int    `json:"compressor_poll_ms"`
}

var (
	GlobalConfig *AgentConfig
	mu           sync.RWMutex
)

// LoadConfig 加载配置，如果文件不存在则创建默认配置
func LoadConfig() error {
	mu.Lock()
	defer mu.Unlock()

	// 默认配置
	GlobalConfig = &AgentConfig{
		NetworkInterface: "auto",
		CompressorPollMs: 3000, // 默认 3 秒
	}

	data, err := os.ReadFile(ConfigFileName)
	if os.IsNotExist(err) {
		// 文件不存在，保存默认配置
		return saveConfigInternal()
	} else if err != nil {
		return err
	}

	return json.Unmarshal(data, GlobalConfig)
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
	return saveConfigInternal()
}

func saveConfigInternal() error {
	data, err := json.MarshalIndent(GlobalConfig, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(ConfigFileName, data, 0644)
}
