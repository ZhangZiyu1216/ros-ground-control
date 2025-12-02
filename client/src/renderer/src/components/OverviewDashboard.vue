<template>
  <div class="overview-container-row">
    <!-- Left Column: Host Command Center (保持不变) -->
    <div class="host-column">
      <div class="panel-header">
        <el-icon><Monitor /></el-icon> COMMAND CENTER
      </div>

      <div class="host-card glass-panel">
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

        <div class="compact-metrics">
          <div class="c-metric">
            <el-progress
              type="dashboard"
              :percentage="hostCpuUsage"
              :width="90"
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
              :width="90"
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

        <div class="spec-grid">
          <div class="spec-cell">
            <label>Model</label>
            <span :title="hostInfo.cpuModel">{{ hostInfo.cpuModel }}</span>
          </div>
          <div class="spec-cell half">
            <label>Cores</label>
            <span>{{ hostInfo.cpuCores || '-' }} Threads</span>
          </div>
          <div class="spec-cell half">
            <label>Freq</label>
            <span>{{ (hostInfo.cpuSpeed / 1000).toFixed(2) }} GHz</span>
          </div>
          <div class="spec-cell">
            <label>Total Memory</label>
            <span>{{ formatBytes(hostInfo.totalmem) }}</span>
          </div>
        </div>

        <div class="divider"></div>

        <div class="net-section">
          <div class="section-sub-title">NETWORK INTERFACES</div>
          <el-scrollbar max-height="150px">
            <div class="net-list">
              <div v-for="net in hostInfo.networks" :key="net.ip" class="net-interface">
                <div class="iface-icon">
                  <el-icon
                    v-if="
                      net.name.toLowerCase().includes('wi') || net.name.toLowerCase().includes('wl')
                    "
                    ><Iphone
                  /></el-icon>
                  <el-icon v-else><Connection /></el-icon>
                </div>
                <div class="iface-info">
                  <span class="iface-name">{{ net.name }}</span>
                  <span class="iface-ip">{{ net.ip }}</span>
                </div>
                <div class="active-dot"></div>
              </div>
              <div v-if="!hostInfo.networks?.length" class="no-net">No active IPv4 interfaces</div>
            </div>
          </el-scrollbar>
        </div>
      </div>
    </div>

    <!-- Right Column: Robot Fleet -->
    <div class="fleet-column">
      <div class="panel-header">
        <el-icon><Connection /></el-icon> ROBOT FLEET
        <span class="badge">{{ activeCount }} / {{ allAgents.length }} ONLINE</span>
      </div>

      <div class="agent-grid">
        <!-- 1. 正常的机器人卡片 -->
        <div
          v-for="agent in allAgents"
          :key="agent.uniqueKey"
          class="flip-container"
          :class="{ 'is-flipped': flippedCards[agent.uniqueKey] }"
        >
          <div class="flipper">
            <!-- Front Side -->
            <div
              class="front agent-card glass-panel"
              :class="{
                'is-online': agent.status === 'ready',
                'is-offline': agent.status !== 'ready'
              }"
              @click="handleCardClick(agent)"
            >
              <div class="card-header">
                <div class="agent-identity">
                  <div class="status-indicator" :class="agent.status"></div>
                  <span class="agent-name">{{ agent.name }}</span>
                </div>
                <!-- [新增] 删除按钮 (仅离线可用，阻止冒泡) -->
                <div class="header-actions">
                  <span class="agent-ip">{{ agent.ip }}</span>
                  <el-button
                    v-if="agent.status !== 'ready'"
                    link
                    type="danger"
                    class="del-btn"
                    @click.stop="handleDelete(agent.originalIndex)"
                  >
                    <el-icon><Delete /></el-icon>
                  </el-button>
                </div>
              </div>

              <div class="card-body">
                <template v-if="agent.status === 'ready'">
                  <div class="static-info-row">
                    <div class="info-group">
                      <el-tooltip
                        :content="agent.sysInfo?.cpu_model || 'Unknown CPU'"
                        placement="top"
                        :show-after="500"
                      >
                        <span class="cpu-model">{{
                          agent.sysInfo?.cpu_model || 'Unknown CPU'
                        }}</span>
                      </el-tooltip>
                      <span v-if="agent.sysInfo?.num_cpu" class="separator">|</span>
                      <span v-if="agent.sysInfo?.num_cpu" class="cpu-cores"
                        >{{ agent.sysInfo.num_cpu }}C</span
                      >
                    </div>
                    <span class="arch-tag">{{ agent.sysInfo?.arch || 'UNK' }}</span>
                  </div>

                  <div class="stats-row">
                    <div class="stat-mini">
                      <span class="s-label">CPU</span>
                      <el-progress
                        :percentage="agent.stats?.cpu_usage || 0"
                        :stroke-width="6"
                        :show-text="false"
                        :color="colors.cpu"
                      />
                    </div>
                    <div class="stat-mini">
                      <span class="s-label">RAM</span>
                      <el-progress
                        :percentage="agent.stats?.mem_usage || 0"
                        :stroke-width="6"
                        :show-text="false"
                        :color="colors.mem"
                      />
                    </div>
                  </div>

                  <div class="net-grid">
                    <div class="net-cell">
                      <el-icon><Download /></el-icon> {{ formatSpeed(agent.stats?.net_rx_rate) }}
                    </div>
                    <div class="net-cell">
                      <el-icon><Upload /></el-icon> {{ formatSpeed(agent.stats?.net_tx_rate) }}
                    </div>
                    <div
                      class="net-cell temp"
                      :class="{ hot: (agent.stats?.temperature || 0) > 70 }"
                    >
                      <el-icon><Odometer /></el-icon>
                      {{ (agent.stats?.temperature || 0).toFixed(1) }}°C
                    </div>
                  </div>
                </template>

                <template v-else>
                  <div class="offline-placeholder">
                    <el-icon
                      class="placeholder-icon"
                      :class="{ 'is-loading': agent.status === 'setting_up' }"
                    >
                      <component :is="agent.status === 'setting_up' ? Loading : SwitchButton" />
                    </el-icon>
                    <span>{{ getStatusText(agent.status) }}</span>
                  </div>
                </template>
              </div>

              <div v-if="agent.status === 'ready'" class="card-footer" @click.stop>
                <div class="footer-btn action" @click="flipCard(agent.uniqueKey)">
                  <el-icon><VideoPlay /></el-icon> 启动序列
                </div>
                <div class="footer-divider"></div>
                <div class="footer-btn danger" @click="handleDisconnect(agent)">
                  <el-icon><SwitchButton /></el-icon> 断开连接
                </div>
              </div>
            </div>

            <!-- Back Side -->
            <div class="back agent-card glass-panel" @click="flipCard(agent.uniqueKey)">
              <div class="card-header back-header">
                <span>Select Sequence</span>
                <el-icon><Back /></el-icon>
              </div>
              <div class="card-body scrollable">
                <template v-if="agent.sequences && agent.sequences.length > 0">
                  <div
                    v-for="seq in agent.sequences"
                    :key="seq.id"
                    class="seq-item"
                    @click.stop="(runSequence(agent.id, seq), flipCard(agent.uniqueKey))"
                  >
                    <el-icon><VideoPlay /></el-icon>
                    <span class="seq-name">{{ seq.name }}</span>
                    <span class="seq-steps">{{ seq.steps.length }} steps</span>
                  </div>
                </template>
                <div v-else class="empty-seq">
                  <el-empty description="No sequences found" :image-size="60" />
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- 2. [新增] 添加新连接卡片 -->
        <div class="flip-container add-new-container">
          <div class="add-card glass-panel">
            <div class="add-btn-group">
              <div class="add-btn-item primary" @click="$emit('add')">
                <el-icon class="add-icon"><Plus /></el-icon>
                <span>新建连接</span>
                <span class="sub-text">手动配置 IP</span>
              </div>
              <div class="add-divider"></div>
              <div class="add-btn-item orange" @click="$emit('first-setup')">
                <el-icon class="add-icon"><MagicStick /></el-icon>
                <span>首次部署</span>
                <span class="sub-text">SSH 安装 Agent</span>
              </div>
            </div>
          </div>
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
const hostCpuUsage = ref(0) // 模拟计算
let refreshTimer = null
// 计算 Host CPU Usage
const refreshHostStats = async () => {
  try {
    // 调用 IPC 获取真实数据
    const data = await window.api.getHostInfo()
    hostInfo.value = {
      ...data,
      cpuModel: data.cpuModel // 后端现在已经处理好这个字段了
    }
    // 注意：第一次调用可能因为没有时间差而返回 0，这是正常的，第二次刷新就会有数据
    hostCpuUsage.value = data.cpuUsage || 0
  } catch (e) {
    console.error('Host info error', e)
  }
}

// ------------------- 核心逻辑 -------------------

// [修改] 基于 Props 计算 allAgents
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
        originalIndex: index // [新增] 保存原始索引用于删除
      }
    } else {
      return {
        id: item.settings.id,
        ip: item.settings.ip,
        name: item.name,
        status: 'disconnected',
        uniqueKey: item.settings.ip, // 使用 IP 作为离线 Key
        settings: item.settings,
        stats: {},
        originalIndex: index // [新增]
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

// 速率转换 (B/s -> KB/s, MB/s)
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

// [新增] 处理删除请求
const handleDelete = (index) => {
  emit('delete', index)
}
// ------------------- 生命周期 -------------------

onMounted(() => {
  // [修改] 不再获取 saved_backends，依赖 props
  refreshHostStats()
  refreshTimer = setInterval(refreshHostStats, 3000)
})

onUnmounted(() => {
  if (refreshTimer) clearInterval(refreshTimer)
})
</script>

<style scoped>
/* ... Host Column 样式保持不变 ... */
.overview-container-row {
  display: flex;
  height: 100%;
  padding: 20px;
  box-sizing: border-box;
  gap: 20px;
  overflow: hidden;
}
.panel-header {
  font-size: 14px;
  font-weight: 800;
  letter-spacing: 1px;
  color: var(--text-secondary);
  margin-bottom: 15px;
  display: flex;
  align-items: center;
  gap: 8px;
}
.host-column {
  width: 300px;
  flex-shrink: 0;
  display: flex;
  flex-direction: column;
}
.host-card {
  flex: 1;
  display: flex;
  flex-direction: column;
  padding: 0;
  background: var(--panel-bg-color);
  border: 1px solid var(--panel-border-color);
  border-radius: var(--panel-radius);
  box-shadow: var(--panel-shadow);
  backdrop-filter: blur(10px);
  overflow: hidden;
}
.host-identity {
  padding: 20px 20px 10px;
}
.id-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 5px;
}
.host-identity h2 {
  margin: 0;
  font-size: 18px;
  color: var(--text-primary);
  max-width: 180px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.platform-tag {
  font-size: 10px;
  color: #409eff;
  background: rgba(64, 158, 255, 0.1);
  padding: 2px 6px;
  border-radius: 4px;
  text-transform: uppercase;
  font-weight: 700;
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
  margin: 10px 20px;
  opacity: 0.5;
}
.compact-metrics {
  display: flex;
  justify-content: space-around;
  padding: 5px 10px;
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
.spec-cell span {
  font-size: 12px;
  font-weight: 600;
  color: var(--text-primary);
  font-family: 'Consolas';
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
  min-height: 0;
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
  border: 1px solid var(--glass-border);
  transition: all 0.2s;
}
.net-interface:hover {
  border-color: #409eff;
  transform: translateX(2px);
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
  font-size: 16px;
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
  font-family: 'Consolas';
  font-weight: 600;
}
.active-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #67c23a;
  box-shadow: 0 0 4px #67c23a;
}
.no-net {
  font-size: 12px;
  color: var(--text-secondary);
  font-style: italic;
  text-align: center;
  margin-top: 20px;
}

/* ... Right Column & Flip Card 样式 ... */
.fleet-column {
  flex: 1;
  display: flex;
  flex-direction: column;
  min-width: 0;
}
.badge {
  margin-left: auto;
  background: rgba(103, 194, 58, 0.1);
  color: #67c23a;
  padding: 2px 8px;
  border-radius: 4px;
  font-size: 12px;
}
.agent-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
  gap: 20px;
  overflow-y: auto;
  padding-right: 5px;
}
.flip-container {
  perspective: 1000px;
  height: 240px;
}
.flipper {
  position: relative;
  width: 100%;
  height: 100%;
  transition: transform 0.6s;
  transform-style: preserve-3d;
}
.flip-container.is-flipped .flipper {
  transform: rotateY(180deg);
}
.front,
.back {
  position: absolute;
  width: 100%;
  height: 100%;
  backface-visibility: hidden;
  background: var(--card-inner-bg);
  border: 1px solid var(--glass-border);
  border-radius: 16px;
  box-shadow: var(--card-shadow);
  display: flex;
  flex-direction: column;
  box-sizing: border-box;
}
.back {
  transform: rotateY(180deg);
  background: var(--bg-color);
}
.is-online {
  animation: shadow-flow-green-soft 3s infinite;
}

/* Header Actions */
.card-header {
  padding: 15px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid var(--divider-color);
  background: rgba(128, 128, 128, 0.02);
}
.agent-identity {
  display: flex;
  align-items: center;
  gap: 8px;
}
.agent-name {
  font-weight: 700;
  font-size: 15px;
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
/* [新增] Header 右侧操作区 */
.header-actions {
  display: flex;
  align-items: center;
  gap: 8px;
}
.agent-ip {
  font-family: 'Consolas', monospace;
  font-size: 12px;
  color: var(--text-secondary);
}
.del-btn {
  padding: 0;
  height: auto;
}

/* Body Info */
.card-body {
  padding: 15px;
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 12px;
}
.static-info-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
  padding-bottom: 8px;
  border-bottom: 1px dashed var(--divider-color);
  gap: 10px;
}
.info-group {
  display: flex;
  align-items: center;
  gap: 6px;
  flex: 1;
  min-width: 0;
}
.cpu-model {
  font-size: 11px;
  font-weight: 700;
  color: var(--text-primary);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.separator {
  color: var(--divider-color);
  font-size: 10px;
  flex-shrink: 0;
}
.cpu-cores {
  font-size: 11px;
  color: var(--text-secondary);
  font-family: 'Consolas', monospace;
  flex-shrink: 0;
  font-weight: 600;
}
.arch-tag {
  font-size: 10px;
  background: rgba(0, 0, 0, 0.05);
  padding: 1px 5px;
  border-radius: 4px;
  color: var(--text-primary);
  font-weight: 600;
  text-transform: uppercase;
  flex-shrink: 0;
}
.stats-row {
  display: flex;
  justify-content: space-between;
  gap: 10px;
}
.stat-mini {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 4px;
}
.s-label {
  font-size: 10px;
  color: var(--text-secondary);
  font-weight: 700;
}
.net-grid {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 5px;
  background: rgba(128, 128, 128, 0.05);
  padding: 6px;
  border-radius: 6px;
  margin-top: auto;
}
.net-cell {
  display: flex;
  flex-direction: column;
  align-items: center;
  font-size: 11px;
  color: var(--text-secondary);
  gap: 2px;
}
.net-cell .el-icon {
  font-size: 14px;
  color: #409eff;
}
.net-cell.temp .el-icon {
  color: #e6a23c;
}
.net-cell.temp.hot {
  color: #f56c6c;
  font-weight: bold;
}
.offline-placeholder {
  height: 100px;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  color: var(--text-secondary);
  opacity: 0.6;
}
.placeholder-icon {
  font-size: 32px;
  margin-bottom: 8px;
}

/* Footer */
.card-footer {
  display: flex;
  align-items: center;
  border-top: 1px solid var(--divider-color);
  height: 40px;
}
.footer-btn {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  font-size: 12px;
  font-weight: 600;
  color: var(--text-secondary);
  cursor: pointer;
  height: 100%;
  transition: background 0.2s;
}
.footer-btn:hover {
  background: rgba(128, 128, 128, 0.05);
  color: var(--text-primary);
}
.footer-btn.action:hover {
  color: #409eff;
  background: rgba(64, 158, 255, 0.05);
}
.footer-btn.danger:hover {
  color: #f56c6c;
  background: rgba(245, 108, 108, 0.05);
}
.footer-divider {
  width: 1px;
  height: 20px;
  background: var(--divider-color);
}

/* Back Side */
.back-header {
  padding: 15px;
  border-bottom: 1px solid var(--divider-color);
  font-weight: 700;
  font-size: 14px;
  color: var(--text-primary);
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.scrollable {
  overflow-y: auto;
  padding: 10px;
}
.seq-item {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 10px;
  border-radius: 8px;
  background: rgba(128, 128, 128, 0.05);
  margin-bottom: 8px;
  cursor: pointer;
  transition: all 0.2s;
}
.seq-item:hover {
  background: rgba(64, 158, 255, 0.1);
  color: #409eff;
}
.seq-name {
  flex: 1;
  font-weight: 600;
  font-size: 13px;
}
.seq-steps {
  font-size: 11px;
  color: var(--text-secondary);
}

/* [新增] Add Card Styles */
.add-card {
  height: 100%;
  width: 100%;
  border: 1px dashed var(--divider-color);
  background: rgba(128, 128, 128, 0.02);
  border-radius: 16px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.add-btn-group {
  display: flex;
  flex-direction: column;
  gap: 15px;
  width: 70%;
}
.add-btn-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 15px;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  background: var(--card-inner-bg);
  box-shadow: var(--card-shadow);
  border: 1px solid transparent;
}
.add-btn-item:hover {
  transform: translateY(-2px);
}
.add-btn-item.primary:hover {
  border-color: #409eff;
  color: #409eff;
}
.add-btn-item.orange:hover {
  border-color: #e6a23c;
  color: #e6a23c;
}

.add-icon {
  font-size: 24px;
  margin-bottom: 5px;
}
.add-btn-item span {
  font-weight: 700;
  font-size: 13px;
}
.sub-text {
  font-size: 10px;
  opacity: 0.7;
  font-weight: 400 !important;
}
.add-divider {
  height: 1px;
  background: var(--divider-color);
  opacity: 0.5;
}

/* Animations */
@keyframes shadow-flow-green-soft {
  0%,
  100% {
    box-shadow: 0 4px 12px rgba(103, 194, 58, 0.1);
    border-color: rgba(103, 194, 58, 0.3);
  }
  50% {
    box-shadow: 0 8px 20px rgba(103, 194, 58, 0.25);
    border-color: rgba(103, 194, 58, 0.6);
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
