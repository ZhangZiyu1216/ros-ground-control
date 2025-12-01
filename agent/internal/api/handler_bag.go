package api

import (
	"fmt"
	"net/http"
	"path/filepath"
	"ros-ground-control/agent/internal/service"
	"syscall"
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

// BagPlayReq 播放请求参数
type BagPlayReq struct {
	Path      string   `json:"path"`       // bag 文件绝对路径
	Topics    []string `json:"topics"`     // 指定播放的话题 (可选)
	Rate      float64  `json:"rate"`       // 播放倍速 (默认 1.0)
	StartTime float64  `json:"start_time"` // 从第几秒开始 (可选)
	Loop      bool     `json:"loop"`       // 是否循环
	Clock     bool     `json:"clock"`      // 是否发布 /clock (用于仿真)
}

// BagPauseReq 暂停请求参数
type BagPauseReq struct {
	ID     string `json:"id"`
	Action string `json:"action"` // "pause" 或 "resume"
}

func RegisterBagRoutes(rg *gin.RouterGroup) {
	// --- 录制接口 ---
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

	// --- 播放接口 ---
	// 1. 开始播放
	rg.POST("/play/start", func(c *gin.Context) {
		var req BagPlayReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 构造 ID
		timestamp := time.Now().Format("150405") // 时分秒
		procID := fmt.Sprintf("play-%s", timestamp)

		// 构造参数
		args := []string{"play", req.Path}

		if req.Rate > 0 && req.Rate != 1.0 {
			args = append(args, "--rate", fmt.Sprintf("%f", req.Rate))
		}
		if req.StartTime > 0 {
			args = append(args, "--start", fmt.Sprintf("%f", req.StartTime))
		}
		if req.Loop {
			args = append(args, "--loop")
		}
		if req.Clock {
			args = append(args, "--clock")
		}
		// 指定话题
		if len(req.Topics) > 0 {
			args = append(args, "--topics")
			args = append(args, req.Topics...)
		}

		// 获取 ROS 环境
		envCfg, _ := service.GlobalROSManager.GenerateConfigStub(service.IDRosCore)

		procCfg := service.ProcessConfig{
			ID:          procID,
			CmdStr:      "rosbag",
			Args:        args,
			Env:         envCfg.Env,
			SetupScript: envCfg.SetupScript,
		}

		if err := service.GlobalProcManager.StartProcess(procCfg); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"status": "playing", "id": procID})
	})

	// 2. 暂停/继续播放
	rg.POST("/play/pause", func(c *gin.Context) {
		var req BagPauseReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		var sig syscall.Signal
		var status string

		switch req.Action {
		case "pause":
			sig = syscall.SIGSTOP // 挂起进程
			status = "paused"
		case "resume":
			sig = syscall.SIGCONT // 恢复进程
			status = "playing"
		default:
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid action (use 'pause' or 'resume')"})
			return
		}

		if err := service.GlobalProcManager.SignalProcess(req.ID, sig); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"status": status})
	})

	// 3. 停止播放
	// 实际上可以直接复用通用的 /proc/stop 或 /api/bag/stop (它们都是调 StopProcess)
	// 但为了 API 结构的一致性，这里再提供一个入口
	rg.POST("/play/stop", func(c *gin.Context) {
		var req struct {
			ID string `json:"id"`
		}
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}
		if err := service.GlobalProcManager.StopProcess(req.ID); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "stopped"})
	})
}
