package service

import (
	"encoding/json"
	"log"
	"math"
	"ros-ground-control/agent/internal/connect"
	"time"

	"github.com/shirou/gopsutil/v3/cpu"
	"github.com/shirou/gopsutil/v3/disk"
	"github.com/shirou/gopsutil/v3/host"
	"github.com/shirou/gopsutil/v3/mem"
	"github.com/shirou/gopsutil/v3/net"
)

// SystemStats 定义发送给前端的数据结构
type SystemStats struct {
	CPUUsage    float64 `json:"cpu_usage"`   // CPU 总使用率 %
	MemUsage    float64 `json:"mem_usage"`   // 内存使用率 %
	MemUsed     uint64  `json:"mem_used"`    // 已用内存 (Bytes)
	MemTotal    uint64  `json:"mem_total"`   // 总内存 (Bytes)
	DiskUsage   float64 `json:"disk_usage"`  // 根目录磁盘使用率 %
	NetRxRate   float64 `json:"net_rx_rate"` // 下载速率 (Bytes/s)
	NetTxRate   float64 `json:"net_tx_rate"` // 上传速率 (Bytes/s)
	Temperature float64 `json:"temperature"` // 核心温度 (℃)
	Uptime      uint64  `json:"uptime"`      // 系统运行时间 (秒)
}

type SystemMonitor struct {
	lastNetStat *net.IOCountersStat
	lastTime    time.Time
}

var GlobalMonitor = &SystemMonitor{}

// StartMonitor 启动监控循环
func (sm *SystemMonitor) StartMonitor() {
	// 预热：第一次获取网络数据作为基准
	netStats, _ := net.IOCounters(false)
	if len(netStats) > 0 {
		sm.lastNetStat = &netStats[0]
		sm.lastTime = time.Now()
	}

	// 启动 Ticker (每 2 秒采集一次)
	// 2秒是一个比较好的平衡，既实时又不占用过多资源
	go func() {
		ticker := time.NewTicker(2 * time.Second)
		defer ticker.Stop()

		for range ticker.C {
			stats := sm.collect()

			// 序列化
			data, _ := json.Marshal(stats)

			// 通过 WebSocket 广播
			// Stream 使用 "system-stats"
			connect.GlobalHub.Broadcast(connect.LogMessage{
				ProcessID: "system",
				Stream:    "system-stats",
				Data:      string(data),
			})
		}
	}()

	log.Println("[Monitor] Hardware resource monitor started.")
}

func (sm *SystemMonitor) collect() SystemStats {
	stats := SystemStats{}

	// 1. CPU
	// 传入 0 表示不阻塞等待，直接返回距上次调用以来的计算结果
	cpuPercents, err := cpu.Percent(0, false)
	if err == nil && len(cpuPercents) > 0 {
		stats.CPUUsage = round(cpuPercents[0])
	}

	// 2. Memory
	vMem, err := mem.VirtualMemory()
	if err == nil {
		stats.MemUsage = round(vMem.UsedPercent)
		stats.MemUsed = vMem.Used
		stats.MemTotal = vMem.Total
	}

	// 3. Disk (Root Path)
	diskInfo, err := disk.Usage("/")
	if err == nil {
		stats.DiskUsage = round(diskInfo.UsedPercent)
	}

	// 4. Network Rate (Backend Calculation)
	currentNetStats, err := net.IOCounters(false)
	if err == nil && len(currentNetStats) > 0 {
		current := currentNetStats[0]
		now := time.Now()

		if sm.lastNetStat != nil {
			duration := now.Sub(sm.lastTime).Seconds()
			if duration > 0 {
				// 计算速率 Bytes/s
				rxBytes := float64(current.BytesRecv - sm.lastNetStat.BytesRecv)
				txBytes := float64(current.BytesSent - sm.lastNetStat.BytesSent)

				stats.NetRxRate = round(rxBytes / duration)
				stats.NetTxRate = round(txBytes / duration)
			}
		}
		// 更新状态
		sm.lastNetStat = &current
		sm.lastTime = now
	}

	// 5. Uptime
	hostInfo, err := host.Info()
	if err == nil {
		stats.Uptime = hostInfo.Uptime
	}

	// 6. Temperature (Best Effort)
	// 尝试获取温度，优先找 core, thermal_zone 等关键字
	temps, err := host.SensorsTemperatures()
	if err == nil {
		maxTemp := 0.0
		for _, t := range temps {
			// 简单的启发式策略：取所有传感器中温度最高的（通常是 CPU 核心或 GPU）
			// 忽略 0 度或异常值
			if t.Temperature > maxTemp && t.Temperature < 150 {
				// 有些传感器可能叫 "acpitz", "coretemp", "thermal_zone0" (Jetson)
				// 我们可以根据名字过滤，也可以简单粗暴取最大值
				// 这里为了兼容性取最大值
				maxTemp = t.Temperature
			}
		}
		stats.Temperature = maxTemp
	}

	return stats
}

// 辅助：保留两位小数
func round(val float64) float64 {
	return math.Round(val*100) / 100
}
