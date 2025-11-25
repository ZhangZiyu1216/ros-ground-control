package api

import (
	"net/http"
	"ros-ground-control/agent/internal/service"
	"time"

	"github.com/gin-gonic/gin"
)

// BridgeReq 定义请求体格式
type BridgeReq struct {
	Service string `json:"service"` // "foxglove"
	Action  string `json:"action"`  // "start", "stop", "restart"
}

func RegisterRosRoutes(rg *gin.RouterGroup) {
	rg.POST("/action", func(c *gin.Context) {
		var req BridgeReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 1. 将前端的服务名映射到内部 ID
		var targetID string
		switch req.Service {
		// 启动全部服务
		case "stack":
			targetID = service.IDStack
		// 启动roscore
		case "roscore":
			targetID = service.IDRosCore
		// 启动rosbridge
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
			err = service.GlobalROSManager.StartService(targetID)
		case "stop":
			if targetID == service.IDStack {
				service.GlobalROSManager.StopDefaultServices()
			} else {
				err = service.GlobalROSManager.StopService(targetID)
			}
		case "restart":
			if targetID == service.IDStack {
				service.GlobalROSManager.StopDefaultServices()
				// 异步重启
				go func() {
					time.Sleep(2 * time.Second)
					service.GlobalROSManager.StartService(service.IDStack)
				}()
			} else {
				err = service.GlobalROSManager.RestartService(targetID)
			}

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
