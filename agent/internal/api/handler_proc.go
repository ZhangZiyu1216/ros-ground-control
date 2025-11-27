package api

import (
	"fmt"
	"log"
	"net/http"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/pkg/config"
	"ros-ground-control/agent/pkg/utils"
	"strings"

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

		setupScript := "" // 初始化为空
		// 2. 探测逻辑
		if req.Cmd == "roslaunch" {
			launchFile := utils.ExtractLaunchFile(req.Args)

			if launchFile != "" {
				detected := utils.DetectWorkspaceSetup(launchFile)

				// 逻辑修正：
				if detected != utils.DefaultRosSetup {
					// 1. 找到了明确的工作空间 (情况 2)
					setupScript = detected
					log.Printf("[Proc] Detected workspace: %s", detected)
				} else if strings.HasPrefix(launchFile, "/opt/ros") {
					// 2. 是系统自带的包 (情况 1) -> 强制使用系统环境
					setupScript = utils.DefaultRosSetup
					log.Printf("[Proc] System package detected. Using default: %s", setupScript)
				} else {
					// 3. 既不是工作空间，也不是系统包 (情况 3: PX4 等非标包)
					// 这时候才回退到用户 bashrc
					setupScript = "USER_BASHRC"
					log.Printf("[Proc] Non-standard package. Fallback to ~/.bashrc")
				}
			} else {
				// 没找到 launch 文件路径，默认用 bashrc 碰运气
				setupScript = "USER_BASHRC"
			}
		} else {
			// rosrun 等其他命令，默认用 bashrc
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
