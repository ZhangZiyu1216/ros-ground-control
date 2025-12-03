<template>
  <div class="overview-container">
    <!-- Left Column: Host Command Center -->
    <div class="layout-col left-col">
      <div class="glass-panel host-panel">
        <div class="panel-header">
          <div class="header-title">
            <el-icon><Monitor /></el-icon> COMMAND CENTER
          </div>
        </div>

        <div class="host-content">
          <!-- Identity -->
          <div class="host-identity">
            <div class="id-row">
              <h2>{{ hostInfo.hostname }}</h2>
              <span class="platform-tag">{{ hostInfo.platform }}</span>
            </div>
            <div class="uptime-row">
              <el-icon><Timer /></el-icon> Uptime: {{ formatUptime(hostInfo.uptime) }}
            </div>
          </div>

          <div class="divider"></div>

          <!-- Circular Metrics -->
          <div class="compact-metrics">
            <div class="c-metric">
              <el-progress
                type="dashboard"
                :percentage="hostCpuUsage"
                :width="100"
                :stroke-width="8"
                :color="colors.cpu"
              >
                <template #default="{ percentage }">
                  <span class="cm-val">{{ percentage }}%</span>
                  <span class="cm-label">CPU</span>
                </template>
              </el-progress>
            </div>
            <div class="c-metric">
              <el-progress
                type="dashboard"
                :percentage="hostMemUsage"
                :width="100"
                :stroke-width="8"
                :color="colors.mem"
              >
                <template #default="{ percentage }">
                  <span class="cm-val">{{ percentage }}%</span>
                  <span class="cm-label">RAM</span>
                </template>
              </el-progress>
            </div>
          </div>

          <!-- Specs Grid -->
          <div class="spec-grid">
            <div class="spec-cell full">
              <label>CPU Model</label>
              <span :title="hostInfo.cpuModel">{{ hostInfo.cpuModel }}</span>
            </div>
            <div class="spec-cell half">
              <label>Threads</label>
              <span>{{ hostInfo.cpuCores || '-' }}</span>
            </div>
            <div class="spec-cell half">
              <label>Frequency</label>
              <span>{{ (hostInfo.cpuSpeed / 1000).toFixed(2) }} GHz</span>
            </div>
            <div class="spec-cell full">
              <label>Total Memory</label>
              <span>{{ formatBytes(hostInfo.totalmem) }}</span>
            </div>
          </div>

          <div class="divider"></div>

          <!-- Network -->
          <div class="net-section">
            <div class="section-sub-title">NETWORK INTERFACES</div>
            <el-scrollbar height="100%">
              <div class="net-list">
                <div v-for="net in hostInfo.networks" :key="net.ip" class="net-interface">
                  <div class="iface-icon">
                    <el-icon v-if="isWifi(net.name)"><Iphone /></el-icon>
                    <el-icon v-else><Connection /></el-icon>
                  </div>
                  <div class="iface-info">
                    <span class="iface-name">{{ net.name }}</span>
                    <span class="iface-ip">{{ net.ip }}</span>
                  </div>
                  <div class="active-dot"></div>
                </div>
                <div v-if="!hostInfo.networks?.length" class="no-net">
                  No active IPv4 interfaces
                </div>
              </div>
            </el-scrollbar>
          </div>
        </div>

        <div class="panel-watermark">
          <el-icon><Monitor /></el-icon> HOST
        </div>
      </div>
    </div>

    <!-- Right Column: Robot Fleet -->
    <div class="layout-col right-col">
      <div class="glass-panel fleet-panel">
        <div class="panel-header">
          <div class="header-title">
            <el-icon><Connection /></el-icon> ROBOT FLEET
          </div>
          <div class="header-stat">
            <span class="stat-num active">{{ activeCount }}</span>
            <span class="stat-divider">/</span>
            <span class="stat-total">{{ allAgents.length }}</span>
            <span class="stat-label">ONLINE</span>
          </div>
        </div>

        <el-scrollbar height="100%" view-style="padding: 10px;">
          <div class="agent-grid">
            <div
              v-for="agent in allAgents"
              :key="agent.uniqueKey"
              class="flip-container"
              :class="{ 'is-flipped': flippedCards[agent.uniqueKey] }"
            >
              <div class="flipper">
                <!-- Front Side -->
                <!-- [修改] 只有 ready 才能翻转，否则触发连接 -->
                <div
                  class="card-face front"
                  :class="{
                    'status-online': agent.status === 'ready',
                    'status-offline': agent.status !== 'ready'
                  }"
                  @click="
                    agent.status === 'ready' ? flipCard(agent.uniqueKey) : handleCardClick(agent)
                  "
                >
                  <div class="card-header">
                    <div class="agent-identity">
                      <div class="status-indicator" :class="agent.status"></div>
                      <span class="agent-name">{{ agent.name }}</span>
                    </div>
                    <!-- 删除按钮 -->
                    <div class="header-actions">
                      <el-tag v-if="agent.status === 'ready'" size="small" effect="plain" round
                        >Ready</el-tag
                      >
                      <el-button
                        v-if="agent.status !== 'ready'"
                        link
                        type="danger"
                        size="small"
                        @click.stop="handleDelete(agent.originalIndex)"
                      >
                        <el-icon><Delete /></el-icon>
                      </el-button>
                    </div>
                  </div>

                  <div class="card-body">
                    <template v-if="agent.status === 'ready'">
                      <div class="info-row">
                        <span class="cpu-chip">{{
                          agent.sysInfo?.cpu_model || 'Unknown CPU'
                        }}</span>
                        <span class="arch-badge">{{ agent.sysInfo?.arch || 'UNK' }}</span>
                      </div>

                      <div class="metrics-row">
                        <div class="metric-item">
                          <el-progress
                            type="circle"
                            :percentage="agent.stats?.cpu_usage || 0"
                            :width="50"
                            :stroke-width="4"
                            :color="colors.cpu"
                            :show-text="false"
                          />
                          <span>CPU</span>
                        </div>
                        <div class="metric-item">
                          <el-progress
                            type="circle"
                            :percentage="agent.stats?.mem_usage || 0"
                            :width="50"
                            :stroke-width="4"
                            :color="colors.mem"
                            :show-text="false"
                          />
                          <span>RAM</span>
                        </div>
                        <div class="net-temp-box">
                          <div class="nt-row">
                            <el-icon><Download /></el-icon>
                            {{ formatSpeed(agent.stats?.net_rx_rate) }}
                          </div>
                          <div class="nt-row">
                            <el-icon><Upload /></el-icon>
                            {{ formatSpeed(agent.stats?.net_tx_rate) }}
                          </div>
                          <div
                            class="nt-row temp"
                            :class="{ hot: (agent.stats?.temperature || 0) > 70 }"
                          >
                            <el-icon><Odometer /></el-icon>
                            {{ (agent.stats?.temperature || 0).toFixed(0) }}°C
                          </div>
                        </div>
                      </div>
                    </template>

                    <template v-else>
                      <div class="offline-state">
                        <el-icon
                          class="state-icon"
                          :class="{ spin: agent.status === 'setting_up' }"
                        >
                          <component :is="agent.status === 'setting_up' ? Loading : SwitchButton" />
                        </el-icon>
                        <span class="state-text">{{ getStatusText(agent.status) }}</span>
                        <div class="agent-ip">{{ agent.ip }}</div>
                      </div>
                    </template>
                  </div>

                  <div class="card-footer" @click.stop>
                    <template v-if="agent.status === 'ready'">
                      <!-- [修改] 点击这里跳转 -->
                      <el-button class="footer-btn primary" text bg @click="handleCardClick(agent)">
                        <el-icon><Monitor /></el-icon> 进入控制台
                      </el-button>
                      <el-button class="footer-btn danger" text bg @click="handleDisconnect(agent)">
                        <el-icon><SwitchButton /></el-icon>
                      </el-button>
                    </template>
                    <template v-else>
                      <!-- 离线状态下按钮不可点击，点击卡片主体即可连接 -->
                      <el-button class="footer-btn" text bg disabled>
                        {{ agent.status === 'setting_up' ? '连接中...' : '点击卡片连接' }}
                      </el-button>
                    </template>
                  </div>
                </div>

                <!-- Back Side -->
                <div class="card-face back" @click="flipCard(agent.uniqueKey)">
                  <div class="card-header back-header">
                    <span>Quick Sequences</span>
                    <el-icon><Back /></el-icon>
                  </div>

                  <div class="card-body scroll-body" @click.stop>
                    <template v-if="agent.sequences && agent.sequences.length > 0">
                      <div
                        v-for="seq in agent.sequences"
                        :key="seq.id"
                        class="seq-list-item"
                        @click="runSequence(agent.id, seq)"
                      >
                        <div class="seq-icon">
                          <el-icon><VideoPlay /></el-icon>
                        </div>
                        <div class="seq-info">
                          <div class="seq-title">{{ seq.name }}</div>
                          <div class="seq-meta">{{ seq.steps.length }} steps</div>
                        </div>
                        <el-icon class="seq-play"><VideoPlay /></el-icon>
                      </div>
                    </template>
                    <div v-else class="empty-seq">
                      <span>无一键启动序列</span>
                      <el-button link type="primary" size="small" @click="handleCardClick(agent)"
                        >去创建</el-button
                      >
                    </div>
                  </div>

                  <div class="card-footer" @click.stop>
                    <el-button class="footer-btn primary" text bg @click="handleCardClick(agent)">
                      <el-icon><Monitor /></el-icon> 进入控制台
                    </el-button>
                  </div>
                </div>
              </div>
            </div>

            <div class="flip-container add-container">
              <div class="add-card">
                <div class="add-option" @click="$emit('add')">
                  <div class="add-icon-circle">
                    <el-icon><Plus /></el-icon>
                  </div>
                  <span class="add-title">新建连接</span>
                  <span class="add-desc">手动输入 IP</span>
                </div>
                <div class="add-divider-h"></div>
                <div class="add-option orange" @click="$emit('first-setup')">
                  <div class="add-icon-circle">
                    <el-icon><MagicStick /></el-icon>
                  </div>
                  <span class="add-title">首次部署</span>
                  <span class="add-desc">SSH 自动安装</span>
                </div>
              </div>
            </div>
          </div>
        </el-scrollbar>

        <div class="panel-watermark">
          <el-icon><Connection /></el-icon> FLEET
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, computed, reactive } from 'vue'
import {
  Monitor,
  Connection,
  VideoPlay,
  SwitchButton,
  Loading,
  Download,
  Upload,
  Odometer,
  Back,
  Timer,
  Iphone,
  Plus,
  MagicStick,
  Delete
} from '@element-plus/icons-vue'
import { useRobotStore } from '../store/robot'
import { ElMessage } from 'element-plus'

const emit = defineEmits(['switch-view', 'add', 'first-setup', 'delete'])
const robotStore = useRobotStore()
const props = defineProps({
  savedBackends: { type: Array, default: () => [] }
})

// 颜色配置 (用于 Progress Bar)
const colors = {
  cpu: [
    { color: '#67c23a', percentage: 40 },
    { color: '#e6a23c', percentage: 70 },
    { color: '#f56c6c', percentage: 100 }
  ],
  mem: '#409eff'
}

// 主机信息
const hostInfo = ref({
  hostname: 'Loading...',
  platform: '',
  arch: '',
  cpuModel: '',
  cpuCores: 0,
  cpuSpeed: 0,
  totalmem: 0,
  freemem: 0,
  uptime: 0,
  networks: []
})
const hostCpuUsage = ref(0)
let refreshTimer = null

// 计算 Host CPU Usage
const refreshHostStats = async () => {
  try {
    // 调用 IPC 获取真实数据
    const data = await window.api.getHostInfo()
    hostInfo.value = {
      ...data,
      cpuModel: data.cpuModel
    }
    hostCpuUsage.value = data.cpuUsage || 0
  } catch (e) {
    console.error('Host info error', e)
  }
}

// ------------------- 核心逻辑 -------------------

const allAgents = computed(() => {
  return props.savedBackends.map((item, index) => {
    // 匹配活跃连接
    const activeClient = Object.values(robotStore.clients).find(
      (c) => c.ip === item.settings.ip || (item.settings.id && c.id === item.settings.id)
    )
    if (activeClient) {
      return {
        ...activeClient,
        name: item.name,
        uniqueKey: activeClient.id,
        status: activeClient.status,
        settings: item.settings,
        stats: activeClient.stats || {},
        originalIndex: index,
        sequences: activeClient.sequences || [] // [补充] 确保 sequences 存在
      }
    } else {
      return {
        id: item.settings.id,
        ip: item.settings.ip,
        name: item.name,
        status: 'disconnected',
        uniqueKey: item.settings.ip,
        settings: item.settings,
        stats: {},
        originalIndex: index,
        sequences: []
      }
    }
  })
})

const activeCount = computed(() => allAgents.value.filter((a) => a.status === 'ready').length)

// ------------------- 工具函数 -------------------

const formatBytes = (bytes) => {
  if (!bytes) return '0 B'
  const k = 1024
  const sizes = ['B', 'KB', 'MB', 'GB', 'TB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))
  return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i]
}

const formatSpeed = (bytesPerSec) => {
  if (!bytesPerSec) return '0 KB/s'
  return formatBytes(bytesPerSec) + '/s'
}

const formatUptime = (seconds) => {
  const h = Math.floor(seconds / 3600)
  const m = Math.floor((seconds % 3600) / 60)
  return `${h}h ${m}m`
}

const hostMemUsage = computed(() => {
  if (!hostInfo.value.totalmem) return 0
  return Math.round(
    ((hostInfo.value.totalmem - hostInfo.value.freemem) / hostInfo.value.totalmem) * 100
  )
})

const getStatusText = (status) => {
  const map = {
    ready: 'Online',
    setting_up: 'Connecting...',
    disconnected: 'Click to Connect',
    failed: 'Connection Failed'
  }
  return map[status] || status
}

// 辅助判断 Wifi 图标
const isWifi = (name) => {
  if (!name) return false
  const lower = name.toLowerCase()
  return lower.includes('wi') || lower.includes('wl')
}

// ------------------- 交互 -------------------

// 翻转状态管理
const flippedCards = reactive({})
const flipCard = (key) => {
  flippedCards[key] = !flippedCards[key]
}

const runSequence = async (agentId, seq) => {
  try {
    ElMessage.info(`Executing: ${seq.name}`)
    await robotStore.runSequence(agentId, seq)
    ElMessage.success('Sent')
  } catch (e) {
    ElMessage.error(e.message)
  }
}

const handleDisconnect = (agent) => {
  emit('switch-view', { index: agent.originalIndex, action: 'disconnect' })
}

const handleCardClick = (agent) => {
  if (agent.status === 'ready') {
    emit('switch-view', { index: agent.originalIndex, action: 'jump' })
  } else {
    emit('switch-view', { index: agent.originalIndex, action: 'connect' })
  }
}

const handleDelete = (index) => {
  emit('delete', index)
}

// ------------------- 生命周期 -------------------

onMounted(() => {
  refreshHostStats()
  refreshTimer = setInterval(refreshHostStats, 3000)
})

onUnmounted(() => {
  if (refreshTimer) clearInterval(refreshTimer)
})
</script>
<style scoped>
/* ============================================
   1. 布局容器
   ============================================ */
.overview-container {
  height: 100%;
  padding: 12px;
  gap: 12px;
  display: flex;
  flex-direction: row;
  box-sizing: border-box;
  background-color: var(--bg-color);
  /* [修复3] 强制无衬线字体 */
  font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
}

.layout-col {
  display: flex;
  flex-direction: column;
  min-width: 0;
}
.left-col {
  width: 320px;
  flex-shrink: 0;
}
.right-col {
  flex: 1;
}

/* 玻璃面板通用 */
.glass-panel {
  flex: 1;
  display: flex;
  flex-direction: column;
  background: var(--panel-bg-color);
  backdrop-filter: blur(12px);
  border: 1px solid var(--panel-border-color);
  border-radius: var(--panel-radius);
  box-shadow: var(--panel-shadow);
  position: relative;
  overflow: hidden;
}

.panel-header {
  padding: 15px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(0, 0, 0, 0.03);
}
.header-title {
  font-size: 14px;
  font-weight: 800;
  letter-spacing: 1px;
  color: var(--text-secondary);
  display: flex;
  align-items: center;
  gap: 8px;
}

/* Watermark */
.panel-watermark {
  position: absolute;
  bottom: 10px;
  right: 15px;
  font-family: 'Segoe UI', Impact, sans-serif;
  font-weight: 900;
  font-size: 20px;
  letter-spacing: 2px;
  color: var(--text-secondary);
  opacity: 0.12;
  display: flex;
  align-items: center;
  gap: 8px;
  pointer-events: none;
  z-index: 0;
}

/* ============================================
   2. Host Panel (Left)
   ============================================ */
.host-content {
  flex: 1;
  overflow-y: auto;
  padding: 0;
  display: flex;
  flex-direction: column;
  z-index: 1;
}

.host-identity {
  padding: 20px;
}
.id-row {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 5px;
}
.host-identity h2 {
  margin: 0;
  font-size: 18px;
  color: var(--text-primary);
  word-break: break-all;
  line-height: 1.2;
}
.platform-tag {
  font-size: 10px;
  color: #409eff;
  background: rgba(64, 158, 255, 0.1);
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: 700;
  text-transform: uppercase;
  white-space: nowrap;
}
.uptime-row {
  font-size: 12px;
  color: var(--text-secondary);
  display: flex;
  align-items: center;
  gap: 5px;
}

.divider {
  height: 1px;
  background: var(--divider-color);
  margin: 5px 20px;
  opacity: 0.5;
}

.compact-metrics {
  display: flex;
  justify-content: space-around;
  padding: 15px 10px;
}
.c-metric {
  display: flex;
  flex-direction: column;
  align-items: center;
}
.cm-val {
  font-size: 16px;
  font-weight: 700;
  color: var(--text-primary);
}
.cm-label {
  font-size: 11px;
  color: var(--text-secondary);
  margin-top: -5px;
}

.spec-grid {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  padding: 10px 20px;
}
.spec-cell {
  display: flex;
  flex-direction: column;
}
.spec-cell.full {
  width: 100%;
}
.spec-cell.half {
  width: calc(50% - 5px);
}
.spec-cell label {
  font-size: 10px;
  color: var(--text-secondary);
  text-transform: uppercase;
  margin-bottom: 2px;
}
/* [修复3] 修正等宽字体名称 */
.spec-cell span {
  font-size: 12px;
  font-weight: 600;
  color: var(--text-primary);
  font-family: 'Consolas', monospace;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.net-section {
  flex: 1;
  display: flex;
  flex-direction: column;
  background: rgba(128, 128, 128, 0.03);
  padding: 15px 20px;
  border-top: 1px solid var(--divider-color);
}
.section-sub-title {
  font-size: 10px;
  font-weight: 800;
  color: var(--text-secondary);
  margin-bottom: 10px;
  letter-spacing: 1px;
}
.net-list {
  display: flex;
  flex-direction: column;
  gap: 8px;
}
.net-interface {
  display: flex;
  align-items: center;
  gap: 10px;
  background: var(--card-inner-bg);
  padding: 8px 12px;
  border-radius: 8px;
  border: 1px solid transparent;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.05);
}
.iface-icon {
  width: 32px;
  height: 32px;
  border-radius: 6px;
  background: rgba(64, 158, 255, 0.1);
  color: #409eff;
  display: flex;
  align-items: center;
  justify-content: center;
}
.iface-info {
  flex: 1;
  display: flex;
  flex-direction: column;
}
.iface-name {
  font-size: 11px;
  color: var(--text-secondary);
  font-weight: 700;
  text-transform: uppercase;
}
.iface-ip {
  font-size: 13px;
  color: var(--text-primary);
  font-family: 'Consolas', monospace;
  font-weight: 600;
}
.active-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #67c23a;
  box-shadow: 0 0 4px #67c23a;
}

/* ============================================
   3. Fleet Panel (Right)
   ============================================ */
.header-stat {
  display: flex;
  align-items: baseline;
  gap: 4px;
  font-family: 'Consolas', monospace;
}
.stat-num {
  font-size: 20px;
  font-weight: 700;
  color: var(--text-secondary);
}
.stat-num.active {
  color: #67c23a;
}
.stat-divider {
  font-size: 16px;
  color: var(--divider-color);
}
.stat-total {
  font-size: 16px;
  color: var(--text-secondary);
}
.stat-label {
  font-size: 10px;
  font-weight: 700;
  color: var(--text-secondary);
  margin-left: 4px;
}

.agent-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
  gap: 15px;
  padding-bottom: 20px;
}

/* --- Flip Card Styles --- */
.flip-container {
  perspective: 1000px;
  height: 220px;
  z-index: 1;
}
.flipper {
  position: relative;
  width: 100%;
  height: 100%;
  transition: transform 0.6s cubic-bezier(0.4, 0, 0.2, 1);
  transform-style: preserve-3d;
}
.flip-container.is-flipped .flipper {
  transform: rotateY(180deg);
}

.card-face {
  position: absolute;
  width: 100%;
  height: 100%;
  backface-visibility: hidden;
  background: var(--card-inner-bg);
  border: 1px solid var(--panel-border-color);
  border-radius: 16px;
  box-shadow: var(--card-shadow);
  display: flex;
  flex-direction: column;
  overflow: hidden;
  transition: all 0.3s;
  cursor: pointer; /* 确保所有状态都有手型 */
}
.card-face.back {
  transform: rotateY(180deg);
  background: var(--bg-color);
}

/* Front Hover Effect */
.card-face.front:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.12);
  border-color: rgba(64, 158, 255, 0.3);
}
/* Online Glow */
.status-online {
  border-left: 3px solid #67c23a;
}
.status-offline {
  border-left: 3px solid var(--divider-color);
  opacity: 0.9;
}

/* Card Content */
.card-header {
  padding: 12px 15px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid var(--divider-color);
  background: rgba(128, 128, 128, 0.03);
}
.agent-identity {
  display: flex;
  align-items: center;
  gap: 8px;
}
.agent-name {
  font-weight: 700;
  font-size: 14px;
  color: var(--text-primary);
}
.status-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #909399;
}
.status-indicator.ready {
  background: #67c23a;
  box-shadow: 0 0 6px #67c23a;
}
.status-indicator.setting_up {
  background: #e6a23c;
  animation: blink 1s infinite;
}

.card-body {
  flex: 1;
  padding: 15px;
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.cpu-chip {
  font-size: 12px;
  font-weight: 600;
  color: var(--text-primary);
  max-width: 140px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.arch-badge {
  font-size: 10px;
  background: rgba(128, 128, 128, 0.1);
  padding: 1px 6px;
  border-radius: 4px;
  color: var(--text-secondary);
  font-weight: 700;
}

.metrics-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-top: auto;
}
.metric-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
}
.metric-item span {
  font-size: 10px;
  color: var(--text-secondary);
  font-weight: 600;
}

.net-temp-box {
  display: flex;
  flex-direction: column;
  gap: 2px;
  align-items: flex-end;
}
.nt-row {
  font-size: 11px;
  color: var(--text-secondary);
  display: flex;
  align-items: center;
  gap: 4px;
  font-family: 'Consolas', monospace;
}
.nt-row.temp.hot {
  color: #f56c6c;
  font-weight: 700;
}

/* Offline State */
.offline-state {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 8px;
  color: var(--text-secondary);
  opacity: 0.7;
}
.state-icon {
  font-size: 32px;
}
.state-icon.spin {
  animation: spin 1s linear infinite;
  color: #e6a23c;
}
.agent-ip {
  font-family: 'Consolas', monospace;
  font-size: 12px;
  background: rgba(128, 128, 128, 0.1);
  padding: 2px 6px;
  border-radius: 4px;
  margin-top: 5px;
}

/* Footer */
.card-footer {
  height: 40px;
  border-top: 1px solid var(--divider-color);
  display: flex;
}
.footer-btn {
  flex: 1;
  border-radius: 0;
  font-size: 12px;
  font-weight: 600;
  height: 100%;
  border: none;
}
.footer-btn.primary {
  color: #409eff;
}
.footer-btn.primary:hover {
  background: rgba(64, 158, 255, 0.1);
}
.footer-btn.danger {
  color: #f56c6c;
  max-width: 50px;
  border-left: 1px solid var(--divider-color);
}
.footer-btn.danger:hover {
  background: rgba(245, 108, 108, 0.1);
}

/* Back Side Content */
.back-header {
  background: transparent;
  color: var(--text-primary);
  font-weight: 700;
  font-size: 13px;
}
.scroll-body {
  overflow-y: auto;
  padding: 10px;
}
.seq-list-item {
  display: flex;
  align-items: center;
  gap: 10px;
  background: rgba(128, 128, 128, 0.05);
  padding: 8px 10px;
  border-radius: 8px;
  margin-bottom: 8px;
  cursor: pointer;
  transition: all 0.2s;
}
.seq-list-item:hover {
  background: rgba(64, 158, 255, 0.1);
  color: #409eff;
}
.seq-icon {
  color: var(--text-secondary);
  font-size: 16px;
}
.seq-info {
  flex: 1;
}
.seq-title {
  font-size: 13px;
  font-weight: 600;
}
.seq-meta {
  font-size: 10px;
  color: var(--text-secondary);
}
.seq-play {
  font-size: 16px;
  opacity: 0;
  transition: opacity 0.2s;
}
.seq-list-item:hover .seq-play {
  opacity: 1;
}
.empty-seq {
  text-align: center;
  color: var(--text-secondary);
  font-size: 12px;
  margin-top: 20px;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

/* Add New Container */
.add-container {
  height: 220px;
}
.add-card {
  height: 100%;
  width: 100%;
  background: rgba(128, 128, 128, 0.03);
  border: 2px dashed var(--divider-color);
  border-radius: 16px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  transition: all 0.3s;
}
.add-card:hover {
  border-color: #409eff;
  background: rgba(64, 158, 255, 0.02);
}

.add-option {
  flex: 1;
  width: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  transition: background 0.2s;
}
.add-option:first-child {
  border-radius: 14px 14px 0 0;
}
.add-option:last-child {
  border-radius: 0 0 14px 14px;
}
.add-option:hover {
  background: rgba(128, 128, 128, 0.05);
}
.add-option.orange:hover .add-icon-circle {
  background: #e6a23c;
  color: white;
  transform: scale(1.1);
}
.add-option:hover .add-icon-circle {
  background: #409eff;
  color: white;
  transform: scale(1.1);
}

.add-icon-circle {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  background: var(--bg-color);
  border: 1px solid var(--divider-color);
  color: var(--text-secondary);
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 5px;
  transition: all 0.3s;
}
.add-title {
  font-size: 13px;
  font-weight: 700;
  color: var(--text-primary);
}
.add-desc {
  font-size: 10px;
  color: var(--text-secondary);
}
.add-divider-h {
  width: 80%;
  height: 1px;
  background: var(--divider-color);
}

/* Animations */
@keyframes spin {
  to {
    transform: rotate(360deg);
  }
}
@keyframes blink {
  0%,
  100% {
    opacity: 1;
  }
  50% {
    opacity: 0.4;
  }
}
</style>
