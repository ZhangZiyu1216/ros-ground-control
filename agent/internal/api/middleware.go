package api

import (
	"net/http"
	"ros-ground-control/agent/internal/session"

	"github.com/gin-gonic/gin"
)

// AuthMiddleware 鉴权中间件
func AuthMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		// 1. 从 Header 获取 Token
		clientToken := c.GetHeader("X-Session-Token")

		// 2. 校验 Token 是否有效 (是否匹配当前 WebSocket 那个 Session)
		if !session.IsValid(clientToken) {
			c.JSON(http.StatusForbidden, gin.H{
				"error": "Unauthorized: Invalid or missing session token. Please reconnect WebSocket.",
			})
			// 拦截，不再向下执行 Handler
			c.Abort()
			return
		}

		// 3. 通过，继续执行
		c.Next()
	}
}
