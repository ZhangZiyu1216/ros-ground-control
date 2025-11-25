package api

import (
	"fmt"
	"net/http"
	"path/filepath"
	"ros-ground-control/agent/internal/service"
	"time"

	"github.com/gin-gonic/gin"
)

type BagStartReq struct {
	Topics []string `json:"topics"` // 要录制的话题列表，空则录制所有
	Name   string   `json:"name"`   // 文件名前缀，如 "test_data"
	Path   string   `json:"path"`   // 保存目录，如 "/home/user/bags"
	Split  bool     `json:"split"`  // 是否按大小分割
	Size   int      `json:"size"`   // 分割大小 (MB)
}

func RegisterBagRoutes(rg *gin.RouterGroup) {
	// 开始录制
	rg.POST("/start", func(c *gin.Context) {
		var req BagStartReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 1. 构造 ID
		// 这是一个普通进程，我们生成一个特定前缀的 ID
		timestamp := time.Now().Format("20060102-150405")
		procID := fmt.Sprintf("bag-%s", timestamp)

		// 2. 构造命令参数
		// 基础命令
		args := []string{"record"}

		// 文件名: /path/name_timestamp.bag
		filename := fmt.Sprintf("%s_%s", req.Name, timestamp)
		fullPath := filepath.Join(req.Path, filename)
		args = append(args, "-O", fullPath)

		// 选项
		if req.Split {
			args = append(args, "--split", "--size", fmt.Sprintf("%d", req.Size))
		}

		// 话题
		if len(req.Topics) == 0 {
			args = append(args, "-a") // 录制所有
		} else {
			args = append(args, req.Topics...)
		}

		// 3. 构造配置
		// 借用 roscore 的环境配置 (因为 rosbag 需要 ROS 环境)
		envCfg, _ := service.GlobalROSManager.GenerateConfigStub(service.IDRosCore)

		procCfg := service.ProcessConfig{
			ID:          procID,
			CmdStr:      "rosbag",
			Args:        args,
			Env:         envCfg.Env,
			SetupScript: envCfg.SetupScript,
		}

		// 4. 启动进程 (交给 ProcessManager 管理)
		if err := service.GlobalProcManager.StartProcess(procCfg); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{
			"status": "recording",
			"id":     procID, // 返回 ID，前端用于停止
			"file":   fullPath,
		})
	})

	// 停止录制
	rg.POST("/stop", func(c *gin.Context) {
		var req struct {
			ID string `json:"id"`
		}
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 关键：rosbag record 需要收到 SIGINT (Ctrl+C) 才能正确写入文件尾部索引。
		// ProcessManager.StopProcess 默认发送的就是 SIGINT，所以直接调用即可。
		if err := service.GlobalProcManager.StopProcess(req.ID); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"status": "stopped"})
	})
}
