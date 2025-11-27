package config

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"
)

// SequenceStep 定义序列中的单个步骤
type SequenceStep struct {
	Type    string `json:"type"`    // "node" | "delay"
	Content string `json:"content"` // 节点ID 或 延时毫秒数
}

// LaunchSequence 定义启动序列
type LaunchSequence struct {
	ID          string         `json:"id"`          // 唯一标识
	Name        string         `json:"name"`        // 序列名称
	Description string         `json:"description"` // 描述
	Steps       []SequenceStep `json:"steps"`       // 步骤列表
}

var (
	seqList   []LaunchSequence
	seqListMu sync.RWMutex
)

// getSequenceListPath 获取 sequences.json 的路径
func getSequenceListPath() (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	// 保持与其他配置在同一目录下
	dir := filepath.Join(home, ".config", "ros-ground-control")
	os.MkdirAll(dir, 0755)
	return filepath.Join(dir, "sequences.json"), nil
}

// LoadSequenceList 加载序列列表
func LoadSequenceList() ([]LaunchSequence, error) {
	seqListMu.Lock()
	defer seqListMu.Unlock()

	path, err := getSequenceListPath()
	if err != nil {
		return nil, err
	}

	data, err := os.ReadFile(path)
	if os.IsNotExist(err) {
		seqList = []LaunchSequence{}
		return seqList, nil
	} else if err != nil {
		return nil, err
	}

	if err := json.Unmarshal(data, &seqList); err != nil {
		return nil, err
	}
	return seqList, nil
}

// SaveSequence 添加或更新序列 (Upsert)
func SaveSequence(seq LaunchSequence) error {
	seqListMu.Lock()
	defer seqListMu.Unlock()

	// 查找并更新，如果没找到则追加
	found := false
	for i, s := range seqList {
		if s.ID == seq.ID {
			seqList[i] = seq
			found = true
			break
		}
	}
	if !found {
		seqList = append(seqList, seq)
	}

	return persistSequenceList()
}

// DeleteSequence 删除序列
func DeleteSequence(id string) error {
	seqListMu.Lock()
	defer seqListMu.Unlock()

	newLists := []LaunchSequence{}
	for _, s := range seqList {
		if s.ID != id {
			newLists = append(newLists, s)
		}
	}
	seqList = newLists

	return persistSequenceList()
}

// persistSequenceList 内部持久化函数
func persistSequenceList() error {
	path, _ := getSequenceListPath()
	data, err := json.MarshalIndent(seqList, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}
