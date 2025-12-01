package api

import (
	"encoding/json"
	"io"
	"log"
	"net/http"
	"ros-ground-control/agent/internal/service"
	"ros-ground-control/agent/internal/session"
	"sync"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var termUpgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// Client 发送的消息结构
type TermMessage struct {
	Type string `json:"type"` // "input" 或 "resize"
	Data string `json:"data"` // 对于 input 是键盘字符
	Rows uint16 `json:"rows"` // 对于 resize
	Cols uint16 `json:"cols"` // 对于 resize
}

func handleTerminal(c *gin.Context) {
	// --- 鉴权逻辑 ---

	// 1. 从 URL Query 参数中获取 Token
	// 例如: ws://ip:port/ws/terminal?token=uuid-string
	token := c.Query("token")

	// 2. 校验 Token 是否有效
	// 这会检查 token 是否不为空，且是否等于当前持有锁的 session token
	if !session.IsValid(token) {
		// 如果无效，直接拒绝升级，返回 403 Forbidden
		c.JSON(http.StatusForbidden, gin.H{"error": "Unauthorized: Invalid token"})
		return
	}

	// --- 鉴权通过，继续原有逻辑 ---
	// 1. 升级 WebSocket
	ws, err := termUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Printf("[Terminal] Upgrade failed: %v", err)
		return
	}
	defer ws.Close()

	// 2. 启动 PTY 会话
	session, err := service.NewTerminalSession()
	if err != nil {
		log.Printf("[Terminal] Failed to start pty: %v", err)
		ws.WriteMessage(websocket.TextMessage, []byte("Error starting shell: "+err.Error()))
		return
	}
	defer session.Close()

	// 3. 定义退出机制
	// 使用 WaitGroup 确保两个 Goroutine 都退出后再清理
	var wg sync.WaitGroup
	wg.Add(2) // Reader 和 Writer

	// --- 协程 A: PTY -> WebSocket (输出流) ---
	go func() {
		defer wg.Done()
		buffer := make([]byte, 4096) // 4KB 缓冲
		for {
			// 从 PTY 读取 Shell 的输出
			n, err := session.PtyFile.Read(buffer)
			if err != nil {
				// PTY 关闭 (比如用户输入了 exit)
				if err != io.EOF {
					log.Printf("[Terminal] PTY read error: %v", err)
				}
				break
			}

			// 发送二进制数据给前端 xterm.js
			// 使用 BinaryMessage 可以避免 UTF-8 校验问题，且效率更高
			err = ws.WriteMessage(websocket.BinaryMessage, buffer[:n])
			if err != nil {
				log.Printf("[Terminal] WS write error: %v", err)
				break
			}
		}
		// PTY 结束了，关闭 WS 连接以通知另一端
		ws.Close()
	}()

	// --- 协程 B: WebSocket -> PTY (输入流) ---
	go func() {
		defer wg.Done()
		for {
			// 读取前端发来的 JSON
			_, msgBytes, err := ws.ReadMessage()
			if err != nil {
				// WS 断开
				break
			}

			var msg TermMessage
			if err := json.Unmarshal(msgBytes, &msg); err != nil {
				log.Printf("[Terminal] Invalid JSON: %v", err)
				continue
			}

			switch msg.Type {
			case "input":
				// 写入 PTY (键盘输入)
				session.PtyFile.Write([]byte(msg.Data))
			case "resize":
				// 调整窗口大小
				session.Resize(msg.Rows, msg.Cols)
			}
		}
		// WS 断开了，关闭 PTY 没什么问题
		session.Close()
	}()

	// 等待直到连接断开
	wg.Wait()
	log.Println("[Terminal] Session closed")
}
