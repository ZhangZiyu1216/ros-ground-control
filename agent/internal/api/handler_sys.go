package api

import (
	"net"
	"net/http"
	"ros-ground-control/agent/pkg/config"
	"runtime"

	"github.com/gin-gonic/gin"
)

// 获取当前配置
func handleGetConfig(c *gin.Context) {
	c.JSON(http.StatusOK, config.GetConfig())
}

// 更新配置
func handleUpdateConfig(c *gin.Context) {
	var newCfg config.AgentConfig
	if err := c.BindJSON(&newCfg); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
		return
	}

	if err := config.UpdateConfig(newCfg); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to save config"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"status": "ok", "msg": "Config updated. System network monitor will handle the restart."})
}

// 获取网卡列表
func handleListInterfaces(c *gin.Context) {
	ifaces, err := net.Interfaces()
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}

	var names []string
	names = append(names, "auto") // 默认选项
	for _, i := range ifaces {
		// 过滤掉 loopback 和 down 的网卡
		if i.Flags&net.FlagUp == 0 || i.Flags&net.FlagLoopback != 0 {
			continue
		}
		names = append(names, i.Name)
	}
	c.JSON(http.StatusOK, names)
}

func handlePing(c *gin.Context) {
	c.JSON(200, gin.H{
		"status": "ok",
		"agent":  "v2.0",
	})
}

func handleSysInfo(c *gin.Context) {
	c.JSON(200, gin.H{
		"os":         runtime.GOOS,
		"arch":       runtime.GOARCH,
		"num_cpu":    runtime.NumCPU(),
		"ros_distro": "noetic", // 这里后续应从环境变量读取
	})
}
