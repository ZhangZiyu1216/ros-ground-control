package api

import (
	"ros-ground-control/agent/internal/connect"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
)

func NewRouter() *gin.Engine {
	r := gin.Default()

	// 1. 配置跨域 (CORS)
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true
	config.AllowHeaders = []string{"Origin", "Content-Length", "Content-Type"}
	r.Use(cors.New(config))

	// 2. 配置 WebSocket 路由
	// WebSocket 也是通过 HTTP 协议握手升级的，所以也是一个 GET 请求
	r.GET("/ws/logs", func(c *gin.Context) {
		connect.HandleWebsocket(c)
	})
	r.GET("/ws/terminal", handleTerminal)

	// 3. 配置 API 路由组
	apiGroup := r.Group("/api")
	{
		// System 接口
		apiGroup.GET("/sys/ping", handlePing)
		apiGroup.GET("/sys/info", handleSysInfo)
		apiGroup.GET("/sys/config", handleGetConfig)
		apiGroup.POST("/sys/config", handleUpdateConfig)
		apiGroup.GET("/sys/interfaces", handleListInterfaces) // 获取网卡列表
		apiGroup.GET("/sys/pubkey", handleGetPubKey)

		// File System 接口
		// RegisterFSRoutes 定义在 handler_fs.go 中 (同 package api，直接调用)
		fsGroup := apiGroup.Group("/fs")
		RegisterFSRoutes(fsGroup)

		// Process 接口
		// RegisterProcRoutes 定义在 handler_proc.go 中 (同 package api，直接调用)
		procGroup := apiGroup.Group("/proc")
		RegisterProcRoutes(procGroup)

		// Bridge 接口
		bridgeGroup := apiGroup.Group("/bridge")
		RegisterBridgeRoutes(bridgeGroup)

		// 常用节点配置接口
		nodeGroup := apiGroup.Group("/nodes")
		RegisterNodeListRoutes(nodeGroup)
	}

	return r
}
