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
