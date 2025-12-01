package session

import (
	"sync"

	"github.com/google/uuid"
)

var (
	// currentToken 存储当前的会话令牌
	currentToken string
	// mu 保证并发安全
	mu sync.RWMutex
)

// TryCreateSession 尝试创建一个新会话
// 如果当前已有会话，返回 ("", false)
// 如果成功，返回 (token, true)
func TryCreateSession() (string, bool) {
	mu.Lock()
	defer mu.Unlock()

	if currentToken != "" {
		return "", false // 已有客户端连接
	}

	// 生成新 Token
	token := uuid.New().String()
	currentToken = token
	return token, true
}

// ClearSession 清除当前会话 (当 WebSocket 断开时调用)
func ClearSession() {
	mu.Lock()
	defer mu.Unlock()
	currentToken = ""
}

// IsValid 校验 Token 是否合法
func IsValid(token string) bool {
	mu.RLock()
	defer mu.RUnlock()

	// 如果当前没连接，或者是空 token，都算非法
	if currentToken == "" || token == "" {
		return false
	}
	return currentToken == token
}

// GetCurrentToken 获取当前 Token (仅供内部调试或特殊用途)
func GetCurrentToken() string {
	mu.RLock()
	defer mu.RUnlock()
	return currentToken
}
