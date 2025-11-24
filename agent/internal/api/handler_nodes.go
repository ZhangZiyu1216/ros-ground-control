package api

import (
	"net/http"
	"ros-ground-control/agent/pkg/config"

	"github.com/gin-gonic/gin"
)

func RegisterNodeListRoutes(rg *gin.RouterGroup) {
	// 1. 获取列表
	rg.GET("", func(c *gin.Context) {
		list, err := config.LoadNodeList()
		if err != nil {
			// 如果读取失败，返回空列表而不是 500，体验更好
			c.JSON(http.StatusOK, []config.QuickNode{})
			return
		}
		c.JSON(http.StatusOK, list)
	})

	// 2. 保存/更新节点
	rg.POST("", func(c *gin.Context) {
		var req config.QuickNode
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 简单的校验
		if req.Name == "" || req.Cmd == "" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Name and Cmd are required"})
			return
		}

		if err := config.SaveNode(req); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "saved"})
	})

	// 3. 删除节点
	rg.DELETE("", func(c *gin.Context) {
		id := c.Query("id")
		if id == "" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Missing id parameter"})
			return
		}

		if err := config.DeleteNode(id); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "deleted"})
	})
}
