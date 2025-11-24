package api

import (
	"net/http"
	"os"
	"path/filepath"
	"ros-ground-control/agent/internal/security"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/pkg/utils"
	"slices"
	"strings"
	"unicode/utf8"

	"github.com/gin-gonic/gin"
)

// 定义请求体结构
type FileActionReq struct {
	Path string `json:"path"`
}
type FileMoveCopyReq struct {
	Src string `json:"src"`
	Dst string `json:"dst"`
}
type FileWriteReq struct {
	Path    string `json:"path"`
	Content string `json:"content"`
}
type BookmarkReq struct {
	Path   string `json:"path"`
	Action string `json:"action"` // "add" or "remove"
}

// handleGetPubKey 返回 RSA 公钥
func handleGetPubKey(c *gin.Context) {
	pubKey := security.GetPublicKeyPEM()
	c.JSON(http.StatusOK, gin.H{"public_key": pubKey})
}

// 辅助函数：判断是否为二进制，并返回 MIME
func detectFileType(header []byte) (bool, string) {
	mime := http.DetectContentType(header)
	// 1. 核心判断：检查是否有 NULL 字节 (0x00)
	if slices.Contains(header, 0x00) {
		return true, mime
	}
	// 2. 辅助判断：明确的二进制媒体类型黑名单
	// 虽然没有 0x00，但如果是压缩包、图片等，也不应该当文本打开
	if strings.HasPrefix(mime, "image/") ||
		strings.HasPrefix(mime, "video/") ||
		strings.HasPrefix(mime, "audio/") ||
		strings.HasPrefix(mime, "application/zip") ||
		strings.HasPrefix(mime, "application/x-gzip") ||
		strings.HasPrefix(mime, "application/pdf") {
		return true, mime
	}
	// 3. 辅助判断：检查是否为有效的 UTF-8 序列
	if !utf8.Valid(header) {
		return true, mime
	}
	// 4. 剩下的全部视为文本
	// 无论是 text/plain, application/json, application/xml, application/javascript
	// 只要没有 0x00 且不是媒体文件，我们都允许前端尝试编辑
	return false, mime
}

// 辅助函数：处理密码提取和解密
func getSudoPassword(c *gin.Context) (string, error) {
	encrypted := c.GetHeader("X-Sudo-Password")
	if encrypted == "" {
		return "", nil // 无密码
	}
	// 尝试解密
	return security.DecryptPassword(encrypted)
}

func RegisterFSRoutes(rg *gin.RouterGroup) {
	// 1. List
	rg.GET("/list", func(c *gin.Context) {
		path := c.Query("path")
		if path == "" {
			path = "." // 默认当前目录
		}
		files, err := service.ListDir(path)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, files)
	})
	// 2. Read
	rg.GET("/read", func(c *gin.Context) {
		path := c.Query("path")
		// 1. 获取解密后的密码
		pwd, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		// 2. 调用 InspectFile
		inspect, err := service.InspectFile(path, pwd)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		// 3. 判断大小 (大于 2MB 就不直接返回文本了，防止卡顿)
		if inspect.Size > 2*1024*1024 {
			c.JSON(http.StatusOK, gin.H{
				"is_binary": true,
				"reason":    "too_large",
				"size":      inspect.Size,
				// 即使太大，我们也试着通过 Header 猜一下类型，方便前端显示图标
				"mime": http.DetectContentType(inspect.Header),
			})
			return
		}
		// 4. 判断是否为二进制
		isBin, mime := detectFileType(inspect.Header)
		if isBin {
			c.JSON(http.StatusOK, gin.H{
				"is_binary": true,
				"mime":      mime,
				"size":      inspect.Size,
			})
			return
		}
		// 5. 确认是小文本文件，调用全量读取 (支持 Sudo)
		// 这里复用我们之前写好的 service.ReadFile
		content, err := service.ReadFile(path, pwd)
		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{
			"is_binary": false,
			"mime":      mime,
			"content":   content,
		})
	})
	// 3. Write
	rg.POST("/write", func(c *gin.Context) {
		var req FileWriteReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Invalid JSON"})
			return
		}
		// 获取并解密密码
		password, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Security check failed: " + err.Error()})
			return
		}
		if err := service.WriteFile(req.Path, req.Content, password); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "ok"})
	})
	// 4. Mkdir (新建文件夹)
	rg.POST("/mkdir", func(c *gin.Context) {
		var req FileActionReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		pwd, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		if err := service.CreateDir(req.Path, pwd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "created"})
	})
	// 5. Delete (删除)
	rg.POST("/delete", func(c *gin.Context) {
		var req FileActionReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		// 调用 MoveToTrash
		if err := utils.MoveToTrash(req.Path); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "trashed"})
	})
	// 6. Delete/permanent (永久删除)
	rg.POST("/delete/permanent", func(c *gin.Context) {
		var req FileActionReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		pwd, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		if err := service.DeletePath(req.Path, pwd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "deleted"})
	})
	// 7. Rename (重命名/移动)
	rg.POST("/rename", func(c *gin.Context) {
		var req FileMoveCopyReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		pwd, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		if err := service.MovePath(req.Src, req.Dst, pwd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "moved"})
	})
	// 8. Copy (复制)
	rg.POST("/copy", func(c *gin.Context) {
		var req FileMoveCopyReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		pwd, err := getSudoPassword(c)
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		if err := service.CopyPath(req.Src, req.Dst, pwd); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "copied"})
	})
	// 9. Watch 接口
	rg.POST("/watch", func(c *gin.Context) {
		var req struct {
			Path string `json:"path"`
		}
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}

		if err := service.GlobalFileWatcher.SetWatchPath(req.Path); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}

		c.JSON(http.StatusOK, gin.H{"status": "watching", "path": req.Path})
	})
	// 10. 获取所有“位置” (常用目录 + 书签)
	rg.GET("/places", func(c *gin.Context) {
		common := utils.GetCommonDirs()
		bookmarks, err := utils.GetBookmarks()
		if err != nil {
			// 书签读取失败不影响常用目录
			bookmarks = []utils.Bookmark{}
		}

		c.JSON(http.StatusOK, gin.H{
			"common":    common,
			"bookmarks": bookmarks,
		})
	})
	// 11. 书签管理
	rg.POST("/bookmark", func(c *gin.Context) {
		var req BookmarkReq
		if err := c.BindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}

		var err error
		switch req.Action {
		case "add":
			err = utils.AddBookmark(req.Path)
		case "remove":
			err = utils.RemoveBookmark(req.Path)
		default:
			c.JSON(http.StatusBadRequest, gin.H{"error": "invalid action"})
			return
		}

		if err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "ok"})
	})
	// 12. 下载/预览接口 (利用 Gin 的 File 方法)
	rg.GET("/download", func(c *gin.Context) {
		path := c.Query("path")
		c.File(path)
	})
	// 13. 上传接口（流式）
	rg.POST("/upload", func(c *gin.Context) {
		// 1. 获取参数
		dstPath := c.PostForm("path")
		if dstPath == "" {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Missing 'path' field"})
			return
		}
		// 2. 获取文件流
		fileHeader, err := c.FormFile("file")
		if err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "Missing 'file' field"})
			return
		}
		// 3. 保存到临时目录 (/tmp)
		// Gin 的 SaveUploadedFile 使用 io.Copy，内存占用小
		tmpPath := filepath.Join(os.TempDir(), "agent-upload-"+filepath.Base(fileHeader.Filename))
		if err := c.SaveUploadedFile(fileHeader, tmpPath); err != nil {
			c.JSON(http.StatusInternalServerError, gin.H{"error": "Upload failed: " + err.Error()})
			return
		}
		// 4. 获取 Sudo 密码 (如果有)
		pwd, err := getSudoPassword(c)
		if err != nil {
			os.Remove(tmpPath) // 清理
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		// 5. 移动到目标位置 (复用 MovePath，它支持 Sudo 和跨分区移动)
		// 如果目标路径需要 Root 权限，MovePath 会使用 sudo mv
		if err := service.MovePath(tmpPath, dstPath, pwd); err != nil {
			os.Remove(tmpPath) // 确保清理
			c.JSON(http.StatusInternalServerError, gin.H{"error": "Move failed: " + err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"status": "uploaded", "size": fileHeader.Size})
	})
}
