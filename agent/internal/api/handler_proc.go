package api

import (
	"fmt"
	"log"
	"net/http"
	"os"
	"strings"

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

		// [Debug] 打印原始请求
		log.Printf("[Proc-Debug] Received Start: Cmd='%s', Args=%v (len=%d)", req.Cmd, req.Args, len(req.Args))

		cfg := config.GetConfig()
		hostIP := utils.GetOutboundIP(cfg.NetworkInterface)
		baseEnv := []string{
			fmt.Sprintf("ROS_IP=%s", hostIP),
			fmt.Sprintf("ROS_HOSTNAME=%s", hostIP),
			"ROS_MASTER_URI=http://localhost:11311",
		}

		setupScript := ""
		finalCmd := req.Cmd
		finalArgs := []string{}

		switch req.Cmd {
		case "roslaunch":
			// roslaunch 保持原样 (前端负责拆分)
			finalArgs = req.Args
			launchFile := utils.ExtractLaunchFile(req.Args)
			if launchFile != "" {
				detected := utils.DetectWorkspaceSetup(launchFile)
				if detected != utils.DefaultRosSetup {
					setupScript = detected
					log.Printf("[Proc] Detected workspace: %s", detected)
				} else if strings.HasPrefix(launchFile, "/opt/ros") {
					setupScript = utils.DefaultRosSetup
				} else {
					setupScript = "USER_BASHRC"
				}
			} else {
				setupScript = "USER_BASHRC"
			}

		case "rosrun", "rostopic", "rosservice":
			// [Debug] 进入 ROS 命令逻辑
			if len(req.Args) >= 2 {
				argsStr := req.Args[0]
				contextPath := req.Args[1]

				// 1. 探测环境
				log.Printf("[Proc-Debug] Detecting env from path: %s", contextPath)
				detected := utils.DetectWorkspaceSetup(contextPath)
				if detected != utils.DefaultRosSetup {
					setupScript = detected
					log.Printf("[Proc] Detected workspace: %s", detected)
				} else {
					setupScript = "USER_BASHRC"
					log.Printf("[Proc] Workspace not found, using bashrc")
				}

				// 2. 解析参数 (关键！)
				// strings.Fields 会自动按空格分割字符串，处理多个空格的情况
				finalArgs = strings.Fields(argsStr)

			} else if len(req.Args) == 1 {
				// 容错：只传了命令，没传路径
				setupScript = "USER_BASHRC"
				finalArgs = strings.Fields(req.Args[0])
			} else {
				// 没参数？
				setupScript = "USER_BASHRC"
			}

		case "bash":
			if len(req.Args) >= 2 {
				fullCommand := req.Args[0]
				contextPath := req.Args[1]

				detected := utils.DetectWorkspaceSetup(contextPath)
				if detected != utils.DefaultRosSetup {
					setupScript = detected
				} else {
					setupScript = "USER_BASHRC"
				}

				finalCmd = "bash"
				// bash -c "..." 这里的命令字符串需要作为一个整体参数
				finalArgs = []string{"-c", fullCommand}
			} else {
				setupScript = "USER_BASHRC"
				if len(req.Args) > 0 {
					finalCmd = "bash"
					finalArgs = []string{"-c", req.Args[0]}
				}
			}

		default:
			log.Printf("[Proc-Debug] Unknown command '%s', hitting default case", req.Cmd)
			setupScript = "USER_BASHRC"
			finalArgs = req.Args
		}

		// [Debug] 打印最终生成的参数
		log.Printf("[Proc-Debug] Final Decision: Setup=%s, Cmd=%s, Args=%v", setupScript, finalCmd, finalArgs)

		// 构造配置
		finalEnv := os.Environ()
		finalEnv = append(finalEnv, baseEnv...)

		procConfig := service.ProcessConfig{
			ID:          req.ID,
			CmdStr:      finalCmd,
			Args:        finalArgs,
			Env:         finalEnv,
			SetupScript: setupScript,
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

	// ... Stop, List 接口保持不变
	rg.POST("/stop", func(c *gin.Context) {
		var req struct {
			ID string `json:"id"`
		}
		if err := c.BindJSON(&req); err != nil {
			c.JSON(400, gin.H{"error": err.Error()})
			return
		}
		if err := service.GlobalProcManager.StopProcess(req.ID); err != nil {
			c.JSON(500, gin.H{"error": err.Error()})
			return
		}
		c.JSON(200, gin.H{"status": "stopped"})
	})

	rg.GET("/list", func(c *gin.Context) {
		list := service.GlobalProcManager.ListProcesses()
		c.JSON(200, gin.H{"count": len(list), "processes": list})
	})
}
