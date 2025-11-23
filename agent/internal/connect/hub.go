package connect

import (
	"log"
	"net/http"
	"sync"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

type LogMessage struct {
	ProcessID string `json:"process_id"`
	Stream    string `json:"stream"` // stdout, stderr
	Data      string `json:"data"`
}

type Hub struct {
	clients   map[*websocket.Conn]bool
	broadcast chan LogMessage
	mu        sync.Mutex
}

var GlobalHub = &Hub{
	clients:   make(map[*websocket.Conn]bool),
	broadcast: make(chan LogMessage),
}

func (h *Hub) Run() {
	for {
		msg := <-h.broadcast
		h.mu.Lock()
		for client := range h.clients {
			err := client.WriteJSON(msg)
			if err != nil {
				client.Close()
				delete(h.clients, client)
			}
		}
		h.mu.Unlock()
	}
}

func (h *Hub) Broadcast(msg LogMessage) {
	h.broadcast <- msg
}

// HandleWebsocket Gin Handler
func HandleWebsocket(c *gin.Context) {
	ws, err := upgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Println(err)
		return
	}
	GlobalHub.mu.Lock()
	GlobalHub.clients[ws] = true
	GlobalHub.mu.Unlock()
}
