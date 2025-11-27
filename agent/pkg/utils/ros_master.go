package utils

import (
	"fmt"

	"github.com/kolo/xmlrpc"
)

// ROSMasterClient 封装与 ROS Master 的通信
type ROSMasterClient struct {
	client *xmlrpc.Client
	ID     string
}

// GetPublishedTopics 获取当前有发布者的话题列表
// 返回 map[TopicName]bool
func (c *ROSMasterClient) GetPublishedTopics() (map[string]bool, error) {
	// API: getSystemState(caller_id)
	// Return: [code, status, [publishers, subscribers, services]]
	// publishers: [ [topic, [node1, node2...]], ... ]

	var result []any
	err := c.client.Call("getSystemState", c.ID, &result)
	if err != nil {
		return nil, err
	}

	// 解析返回值
	// [code, statusMessage, [publishers, subscribers, services]]
	if len(result) < 3 {
		return nil, fmt.Errorf("invalid response length")
	}

	stateList, ok := result[2].([]interface{})
	if !ok || len(stateList) < 1 {
		return nil, fmt.Errorf("system state body error")
	}

	// stateList[0] 是 publishers
	pubs, ok := stateList[0].([]interface{})
	if !ok {
		return nil, fmt.Errorf("publishers list error")
	}

	activeTopics := make(map[string]bool)
	for _, item := range pubs {
		// item 应该是 [topicName, [node1, node2...]]
		pair, ok := item.([]interface{})
		if !ok || len(pair) < 2 {
			continue
		}

		topic, ok1 := pair[0].(string)
		nodes, ok2 := pair[1].([]interface{})

		if ok1 && ok2 && len(nodes) > 0 {
			activeTopics[topic] = true
		}
	}

	return activeTopics, nil
}

// NewROSMasterClient 创建客户端
// uri 例如: "http://localhost:11311"
func NewROSMasterClient(uri string) (*ROSMasterClient, error) {
	client, err := xmlrpc.NewClient(uri, nil)
	if err != nil {
		return nil, err
	}
	return &ROSMasterClient{
		client: client,
		ID:     "/ros_ground_control_agent", // Agent 在 Master 眼里的名字
	}, nil
}

// GetTopicTypes 获取所有话题及其类型
// 返回 map[TopicName]TopicType
func (c *ROSMasterClient) GetTopicTypes() (map[string]string, error) {
	// ROS API: getTopicTypes(caller_id)
	// Response: [code, statusMessage, [[topic, type], ...]]
	var result []interface{}

	// 设置超时，防止 Master 卡死导致 Agent 阻塞
	// xmlrpc 库本身没有直接的超时设置，这里假设网络是本地的很快
	// 生产环境可以考虑自行封装 http.Client

	err := c.client.Call("getTopicTypes", c.ID, &result)
	if err != nil {
		return nil, err
	}

	if len(result) < 3 {
		return nil, fmt.Errorf("invalid response format from master")
	}

	// 解析返回值
	// code := result[0].(int64) // 1 表示成功
	// status := result[1].(string)
	topicList, ok := result[2].([]interface{})
	if !ok {
		return nil, fmt.Errorf("topic list format error")
	}

	topicTypes := make(map[string]string)
	for _, t := range topicList {
		pair, ok := t.([]interface{})
		if !ok || len(pair) < 2 {
			continue
		}
		name := pair[0].(string)
		typ := pair[1].(string)
		topicTypes[name] = typ
	}

	return topicTypes, nil
}

// IsMasterRunning 简单检查 Master 是否在线
func IsMasterRunning(uri string) bool {
	client, err := NewROSMasterClient(uri)
	if err != nil {
		return false
	}
	// 尝试调用 getUri 来测试连接
	var res []interface{}
	err = client.client.Call("getUri", "/test", &res)
	return err == nil
}
