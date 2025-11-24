package connect

import (
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

// 配置常量
const (
	writeWait      = 10 * time.Second    // 写超时
	pongWait       = 60 * time.Second    // 读 Pong 超时
	pingPeriod     = (pongWait * 9) / 10 // 发送 Ping 的周期 (必须小于 pongWait)
	maxMessageSize = 512                 // 前端发来的消息限制 (心跳包很小)
	historySize    = 100                 // 日志回放：保留最近 100 条
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

// LogMessage 日志结构
type LogMessage struct {
	ProcessID string `json:"process_id"`
	Stream    string `json:"stream"`
	Data      string `json:"data"`
}

// Client 代表一个 WebSocket 连接
type Client struct {
	hub  *Hub
	conn *websocket.Conn
	send chan LogMessage // 待发送消息通道
}

// Hub 管理所有连接和日志广播
type Hub struct {
	clients    map[*Client]bool
	broadcast  chan LogMessage
	register   chan *Client
	unregister chan *Client

	// 日志回放缓冲区 (Ring Buffer)
	history   []LogMessage
	historyMu sync.RWMutex
}

var GlobalHub = NewHub()

func NewHub() *Hub {
	return &Hub{
		broadcast:  make(chan LogMessage),
		register:   make(chan *Client),
		unregister: make(chan *Client),
		clients:    make(map[*Client]bool),
		history:    make([]LogMessage, 0, historySize),
	}
}

func (h *Hub) Run() {
	for {
		select {
		case client := <-h.register:
			h.clients[client] = true
			// 连接建立时，立即发送历史日志
			h.pushHistory(client)

		case client := <-h.unregister:
			if _, ok := h.clients[client]; ok {
				delete(h.clients, client)
				close(client.send)
			}

		case msg := <-h.broadcast:
			// 1. 存入历史记录
			h.saveToHistory(msg)

			// 2. 广播给所有客户端
			for client := range h.clients {
				select {
				case client.send <- msg:
				default:
					// 发送阻塞，直接踢掉
					close(client.send)
					delete(h.clients, client)
				}
			}
		}
	}
}

// Broadcast 供外部调用
func (h *Hub) Broadcast(msg LogMessage) {
	h.broadcast <- msg
}

// saveToHistory 存入环形缓冲区
func (h *Hub) saveToHistory(msg LogMessage) {
	h.historyMu.Lock()
	defer h.historyMu.Unlock()

	// 简单的切片追加与截断
	if len(h.history) >= historySize {
		// 删除第一条，追加新的一条
		h.history = h.history[1:]
	}
	h.history = append(h.history, msg)
}

// pushHistory 将缓存发送给新连接
func (h *Hub) pushHistory(client *Client) {
	h.historyMu.RLock()
	defer h.historyMu.RUnlock()

	for _, msg := range h.history {
		select {
		case client.send <- msg:
		default:
		}
	}
}

// readPump 处理 WebSocket 读取 (主要用于处理 Pong 和关闭)
func (c *Client) readPump() {
	defer func() {
		c.hub.unregister <- c
		c.conn.Close()
	}()

	c.conn.SetReadLimit(maxMessageSize)
	// 设置读超时，如果多久没收到 Pong 就断开
	c.conn.SetReadDeadline(time.Now().Add(pongWait))

	// 设置 Pong 处理函数：收到 Pong 后重置超时时间
	c.conn.SetPongHandler(func(string) error {
		c.conn.SetReadDeadline(time.Now().Add(pongWait))
		return nil
	})

	for {
		// 我们主要通过 WS 发送数据，不怎么读数据，但必须这里读才能处理 Control Frame (Ping/Pong/Close)
		_, _, err := c.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("WS error: %v", err)
			}
			break
		}
	}
}

// writePump 处理 WebSocket 写入 (发送日志 + 发送 Ping)
// 关键：Gorilla WebSocket 要求同一时间只能有一个 Goroutine 写入同一个 Conn
func (c *Client) writePump() {
	ticker := time.NewTicker(pingPeriod)
	defer func() {
		ticker.Stop()
		c.conn.Close()
	}()

	for {
		select {
		case msg, ok := <-c.send:
			// 设置写超时
			c.conn.SetWriteDeadline(time.Now().Add(writeWait))
			if !ok {
				// Channel closed
				c.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}

			// 发送 JSON 日志
			if err := c.conn.WriteJSON(msg); err != nil {
				return
			}

		case <-ticker.C:
			// 发送 Ping 心跳
			c.conn.SetWriteDeadline(time.Now().Add(writeWait))
			if err := c.conn.WriteMessage(websocket.PingMessage, nil); err != nil {
				return
			}
		}
	}
}

// HandleWebsocket Gin Handler
func HandleWebsocket(c *gin.Context) {
	conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Println(err)
		return
	}

	client := &Client{hub: GlobalHub, conn: conn, send: make(chan LogMessage, 256)}
	client.hub.register <- client

	// 启动读写泵
	go client.writePump()
	go client.readPump()
}
