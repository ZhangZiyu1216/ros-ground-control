package config

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// ParamFile 定义单个参数文件的引用
type ParamFile struct {
	Name string `json:"name"` // 显示名称，例如 "雷达内参"
	Path string `json:"path"` // 文件绝对路径，例如 "/home/user/lidar.yaml"
}

// QuickNode 定义“常用节点”的数据结构
type QuickNode struct {
	ID          string      `json:"id"`          // 唯一标识 (UUID 或 时间戳)
	Name        string      `json:"name"`        // 显示名称 (如 "激光雷达")
	Cmd         string      `json:"cmd"`         // roslaunch / rosrun
	Args        []string    `json:"args"`        // ["my_pkg", "lidar.launch"]
	Description string      `json:"description"` // 备注
	Params      []ParamFile `json:"params"`      // 关联的参数文件列表
	Nice        int         `json:"nice"`        // 保存优先级设置
}

var (
	nodeList   []QuickNode
	nodeListMu sync.RWMutex
)

// getNodeListPath 获取 nodes.json 的路径
func getNodeListPath() (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	dir := filepath.Join(home, ".config", "ros-ground-control")
	os.MkdirAll(dir, 0755)
	return filepath.Join(dir, "nodes.json"), nil
}

// LoadNodeList 加载节点列表
func LoadNodeList() ([]QuickNode, error) {
	nodeListMu.Lock()
	defer nodeListMu.Unlock()

	path, err := getNodeListPath()
	if err != nil {
		return nil, err
	}

	data, err := os.ReadFile(path)
	if os.IsNotExist(err) {
		// 如果不存在，返回空列表
		nodeList = []QuickNode{}
		return nodeList, nil
	} else if err != nil {
		return nil, err
	}

	if err := json.Unmarshal(data, &nodeList); err != nil {
		return nil, err
	}
	return nodeList, nil
}

// SaveNode 添加或更新节点
func SaveNode(node QuickNode) error {
	nodeListMu.Lock()
	defer nodeListMu.Unlock()

	// 如果没有 ID，生成一个 (简单用纳秒时间戳即可，或者前端传 UUID)
	if node.ID == "" {
		node.ID = fmt.Sprintf("node-%d", time.Now().UnixNano())
	}

	// 查找并更新，如果没找到则追加
	found := false
	for i, n := range nodeList {
		if n.ID == node.ID {
			nodeList[i] = node
			found = true
			break
		}
	}
	if !found {
		nodeList = append(nodeList, node)
	}

	return persistNodeList()
}

// DeleteNode 删除节点
func DeleteNode(id string) error {
	nodeListMu.Lock()
	defer nodeListMu.Unlock()

	newLists := []QuickNode{}
	for _, n := range nodeList {
		if n.ID != id {
			newLists = append(newLists, n)
		}
	}
	nodeList = newLists

	return persistNodeList()
}

// persistNodeList 内部持久化函数
func persistNodeList() error {
	path, _ := getNodeListPath()
	data, err := json.MarshalIndent(nodeList, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}
