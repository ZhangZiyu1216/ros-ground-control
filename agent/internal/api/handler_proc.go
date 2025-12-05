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
		isRaw := false

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
			// --- 核心修复：开启 Raw 模式 ---
			isRaw = true

			if len(req.Args) >= 2 {
				// Index 0: 命令参数字符串
				// Index 1: 上下文路径 (仅用于探测)

				argsStr := req.Args[0] // 获取原始字符串
				contextPath := req.Args[1]

				// 探测环境 ... (保持不变)
				detected := utils.DetectWorkspaceSetup(contextPath)
				if detected != utils.DefaultRosSetup {
					setupScript = detected
				} else {
					setupScript = "USER_BASHRC"
				}

				// 关键点：不要 strings.Fields(argsStr)！
				// 直接把这整个长字符串作为一个元素放入 slice
				finalArgs = []string{argsStr}

			} else if len(req.Args) == 1 {
				// 容错分支
				setupScript = "USER_BASHRC"
				// 同样不要分割
				finalArgs = []string{req.Args[0]}
			} else {
				setupScript = "USER_BASHRC"
			}

			// 如果是 bash 命令，特殊处理一下 cmdStr
			if req.Cmd == "bash" {
				// 为了简化前端，这里做个自动适配：
				if req.Cmd == "bash" && len(finalArgs) > 0 {
					rawCmd := finalArgs[0]
					finalArgs = []string{"-c", fmt.Sprintf("%q", rawCmd)}
				}
			}

		case "bash":
			// 约定: Args[0] = 纯命令字符串 (如 "ls -la"), Args[1] = 环境路径
			// 开启 Raw 模式，让 proc_manager 仅仅做字符串拼接
			isRaw = true
			var userCommand string

			if len(req.Args) >= 2 {
				userCommand = req.Args[0]
				contextPath := req.Args[1]
				// 1. 探测环境
				detected := utils.DetectWorkspaceSetup(contextPath)
				if detected != utils.DefaultRosSetup {
					setupScript = detected
					log.Printf("[Proc] Detected workspace: %s", detected)
				} else {
					setupScript = "USER_BASHRC"
				}
			} else if len(req.Args) == 1 {
				// 容错: 没传路径
				userCommand = req.Args[0]
				setupScript = "USER_BASHRC"
			} else {
				// 空命令
				setupScript = "USER_BASHRC"
				userCommand = "echo 'No command provided'"
			}
			// 2. 构造 bash -c 指令
			// 我们需要执行: bash -c "用户输入的命令"
			// 因为 proc_manager 开启了 RawArgs，它会直接拼接字符串。
			// 所以我们需要在这里手动用 %q 给 userCommand 加上双引号并转义内部字符。
			finalCmd = "bash"
			// 最终 args 数组变成: ["-c", "\"ls -la | grep x\""]
			// 拼接后效果: bash -c "ls -la | grep x"
			finalArgs = []string{"-c", fmt.Sprintf("%q", userCommand)}

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
			RawArgs:     isRaw,
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
