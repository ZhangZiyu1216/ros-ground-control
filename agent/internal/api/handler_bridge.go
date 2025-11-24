package api

import (
	"net/http"
	"ros-ground-control/agent/internal/service"

	"github.com/gin-gonic/gin"
)

// BridgeReq 定义请求体格式
type BridgeReq struct {
	Service string `json:"service"` // "foxglove"
	Action  string `json:"action"`  // "start", "stop", "restart"
}

func RegisterBridgeRoutes(rg *gin.RouterGroup) {
	rg.POST("/action", func(c *gin.Context) {
		var req BridgeReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 1. 将前端的服务名映射到内部 ID
		var targetID string
		switch req.Service {
		case "foxglove":
			targetID = service.IDFoxglove
		default:
			c.JSON(http.StatusBadRequest, gin.H{"error": "Unknown service name"})
			return
		}

		// 2. 执行动作
		var err error
		switch req.Action {
		case "start":
			// 需要重新获取配置来启动，这里简化处理，假设 Manager 内部能处理重启动逻辑
			// 由于我们在 ServiceConfig 里定义了 cmd，建议在 ROSManager 里增加一个 GetConfigByID 方法，
			// 或者简单地只支持 restart/stop。
			// 这里演示 Restart 的逻辑，Start 逻辑类似。
			err = service.GlobalROSManager.RestartService(targetID)
		case "stop":
			err = service.GlobalROSManager.StopService(targetID)
		case "restart":
			err = service.GlobalROSManager.RestartService(targetID)
		default:
			c.JSON(http.StatusBadRequest, gin.H{"error": "Unknown action"})
			return
		}

		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"status": "ok", "service": req.Service, "action": req.Action})
	})
}
