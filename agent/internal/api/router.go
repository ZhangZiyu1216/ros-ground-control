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

		// --- 2. 受保护接口 (应用 AuthMiddleware) ---
		// 创建一个子路由组，应用中间件
		protected := apiGroup.Group("/")
		protected.Use(AuthMiddleware())
		{
			protected.GET("/sys/info", handleSysInfo)
			protected.GET("/sys/config", handleGetConfig)
			protected.POST("/sys/config", handleUpdateConfig)
			protected.GET("/sys/interfaces", handleListInterfaces) // 获取网卡列表
			protected.GET("/sys/pubkey", handleGetPubKey)
			// File System 接口
			// RegisterFSRoutes 定义在 handler_fs.go 中 (同 package api，直接调用)
			fsGroup := protected.Group("/fs")
			RegisterFSRoutes(fsGroup)

			// Process 接口
			// RegisterProcRoutes 定义在 handler_proc.go 中 (同 package api，直接调用)
			procGroup := protected.Group("/proc")
			RegisterProcRoutes(procGroup)

			// ros 服务接口
			rosGroup := protected.Group("/ros")
			RegisterRosRoutes(rosGroup)

			// 常用节点配置接口
			nodeGroup := protected.Group("/nodes")
			RegisterNodeListRoutes(nodeGroup)

			// 启动序列接口
			seqGroup := protected.Group("/sequences")
			RegisterSequenceRoutes(seqGroup)

			// ros 包录制接口
			bagGroup := protected.Group("/bag")
			RegisterBagRoutes(bagGroup)
		}
	}

	return r
}
