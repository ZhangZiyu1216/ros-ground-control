package api

import (
	"net/http"
	"ros-ground-control/agent/pkg/config"

	"github.com/gin-gonic/gin"
)

func RegisterSequenceRoutes(rg *gin.RouterGroup) {
	// 1. 获取所有序列
	rg.GET("", func(c *gin.Context) {
		list, err := config.LoadSequenceList()
		if err != nil {
			// 读取失败返回空列表，不阻断前端
			c.JSON(http.StatusOK, []config.LaunchSequence{})
			return
		}
		c.JSON(http.StatusOK, list)
	})

	// 2. 保存/更新序列
	rg.POST("", func(c *gin.Context) {
		var req config.LaunchSequence
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}

		// 基础校验
		if req.ID == "" || req.Name == "" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "ID and Name are required"})
			return
		}

		if err := config.SaveSequence(req); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "saved"})
	})

	// 3. 删除序列
	rg.DELETE("", func(c *gin.Context) {
		id := c.Query("id")
		if id == "" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Missing id parameter"})
			return
		}

		if err := config.DeleteSequence(id); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "deleted"})
	})
}
