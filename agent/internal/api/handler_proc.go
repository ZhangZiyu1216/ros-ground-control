package api

import (
	"fmt"
	"log"
	"net/http"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"

	"github.com/gin-gonic/gin"
)

type StartProcReq struct {
	ID   string   `json:"id"`
	Cmd  string   `json:"cmd"`
	Args []string `json:"args"`
}

func RegisterProcRoutes(rg *gin.RouterGroup) {
	rg.POST("/start", func(c *gin.Context) {
		var req StartProcReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}

		// 1. 准备 ROS_IP 等关键变量
		cfg := config.GetConfig()
		hostIP := utils.GetOutboundIP(cfg.NetworkInterface)

		// 这些变量会被 ProcManager 提取并转换成 "export KEY=VAL;" 放到命令最前面
		baseEnv := []string{
			fmt.Sprintf("ROS_IP=%s", hostIP),
			fmt.Sprintf("ROS_HOSTNAME=%s", hostIP),
			"ROS_MASTER_URI=http://localhost:11311",
		}

		setupScript := utils.DefaultRosSetup // 默认 /opt/ros/noetic/setup.bash

		// 2. 探测逻辑
		if req.Cmd == "roslaunch" {
			launchFile := utils.ExtractLaunchFile(req.Args)
			log.Printf("[Proc] Request to launch: %s", launchFile)

			if launchFile != "" {
				detected := utils.DetectWorkspaceSetup(launchFile)

				if detected != utils.DefaultRosSetup {
					// 场景 A: 找到了标准工作空间
					log.Printf("[Proc] Detected workspace: %s", detected)
					setupScript = detected
				} else {
					// 场景 B: 没找到 -> 这是一个系统包(apt安装) 或者 像PX4那样的非标包
					// 策略：直接使用用户的 .bashrc，这是最稳妥的 fallback
					log.Printf("[Proc] No workspace detected. Falling back to ~/.bashrc")
					setupScript = "USER_BASHRC"
				}
			}
		} else {
			// 对于 rosrun，通常也直接用 bashrc
			setupScript = "USER_BASHRC"
		}

		procConfig := service.ProcessConfig{
			ID:          req.ID,
			CmdStr:      req.Cmd,
			Args:        req.Args,
			Env:         baseEnv,     // 传入需要 export 的变量
			SetupScript: setupScript, // 传入 source 策略
		}

		if err := service.GlobalProcManager.StartProcess(procConfig); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{
			"status": "started",
			"id":     req.ID,
			"setup":  setupScript,
		})
	})

	rg.POST("/stop", func(c *gin.Context) {
		var req struct {
			ID string `json:"id"`
		}
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}

		if err := service.GlobalProcManager.StopProcess(req.ID); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "stopped"})
	})

	rg.GET("/list", func(c *gin.Context) {
		// 直接调用 Manager 获取列表
		list := service.GlobalProcManager.ListProcesses()

		// 返回 JSON
		c.JSON(http.StatusOK, gin.H{
			"count":     len(list),
			"processes": list,
		})
	})
}
