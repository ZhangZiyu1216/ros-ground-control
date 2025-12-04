<template>
  <div class="overview-container">
    <!-- Left Column: Host Command Center -->
    <div class="layout-col left-col">
      <div class="glass-panel host-panel">
        <div class="panel-header">
          <div class="header-title">
            <el-icon><Monitor /></el-icon> 当前主机
          </div>
        </div>

        <div class="host-content">
          <!-- [修改] 使用统一的滚动条包裹整个左侧内容 -->
          <el-scrollbar height="100%">
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
                  class="enhanced-progress"
                  type="dashboard"
                  :percentage="hostCpuUsage"
                  :width="90"
                  :stroke-width="8"
                  :color="colors.cpu"
                >
                  <template #default="{ percentage }"
                    ><span class="cm-val">{{ percentage }}%</span></template
                  >
                </el-progress>
                <span class="cm-label">CPU Load</span>
              </div>
              <div class="c-metric">
                <el-progress
                  class="enhanced-progress"
                  type="dashboard"
                  :percentage="hostMemUsage"
                  :width="90"
                  :stroke-width="8"
                  :color="colors.mem"
                >
                  <template #default="{ percentage }"
                    ><span class="cm-val">{{ percentage }}%</span></template
                  >
                </el-progress>
                <span class="cm-label">RAM Usage</span>
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

            <!-- Network (移除内部滚动条，改为自然流) -->
            <div class="net-section">
              <div class="section-sub-title">NETWORK INTERFACES</div>
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
            </div>
          </el-scrollbar>
        </div>

        <div class="panel-watermark">
          <el-icon><Monitor /></el-icon> Client
        </div>
      </div>
    </div>

    <!-- Right Column: Robot Fleet -->
    <div class="layout-col right-col">
      <div class="glass-panel fleet-panel">
        <div class="panel-header">
          <div class="header-title">
            <el-icon><Connection /></el-icon> 远程主机
          </div>
          <div class="header-stat">
            <span class="stat-num active">{{ activeCount }}</span>
            <span class="stat-divider">/</span>
            <span class="stat-total">{{ allAgents.length }}</span>
            <span class="stat-label">在线</span>
          </div>
        </div>

        <div class="fleet-list-wrapper">
          <el-scrollbar height="100%">
            <div class="agent-grid">
              <div
                v-for="agent in allAgents"
                :key="agent.uniqueKey"
                class="flip-container"
                :class="{ 'is-flipped': flippedCards[agent.uniqueKey] }"
              >
                <div class="flipper">
                  <!-- Front Side -->
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
                      <div class="header-left">
                        <div class="status-indicator" :class="agent.status"></div>
                        <div class="id-text-group">
                          <span class="agent-name">{{ agent.name }}</span>
                          <span class="agent-address">{{ agent.ip }}:{{ agent.port }}</span>
                        </div>
                      </div>

                      <!-- [需求1] ROS 状态胶囊 + 删除按钮 -->
                      <div class="header-right-group">
                        <!-- 仅 Ready 状态显示 -->
                        <div
                          v-if="agent.status === 'ready'"
                          class="ros-control-group"
                          :class="{
                            'is-fully-active':
                              agent.serviceStatus?.roscore === 'active' &&
                              agent.serviceStatus?.bridge === 'active',
                            'is-loading': agent.isStackLoading
                          }"
                          @click.stop="toggleRosStack(agent)"
                        >
                          <!-- Loading 遮罩 (居中显示) -->
                          <div v-if="agent.isStackLoading" class="loading-overlay">
                            <el-icon class="is-loading"><Loading /></el-icon>
                          </div>

                          <!-- 胶囊 1: CORE -->
                          <div
                            class="ros-pill"
                            :class="{ active: agent.serviceStatus?.roscore === 'active' }"
                          >
                            <div class="pill-dot"></div>
                            <span>Core</span>
                          </div>

                          <!-- 胶囊 2: BRIDGE -->
                          <div
                            class="ros-pill"
                            :class="{ active: agent.serviceStatus?.bridge === 'active' }"
                          >
                            <div class="pill-dot"></div>
                            <span>Bridge</span>
                          </div>
                        </div>

                        <!-- 删除按钮 -->
                        <el-button
                          v-if="agent.status !== 'ready'"
                          link
                          type="danger"
                          size="small"
                          class="del-btn"
                          @click.stop="handleDelete(agent.originalIndex)"
                        >
                          <el-icon><Delete /></el-icon>
                        </el-button>
                      </div>
                    </div>

                    <div class="card-body">
                      <template v-if="agent.status === 'ready'">
                        <!-- 1. Info Section (Refactored) -->
                        <div class="info-section">
                          <!-- Top Line: Model + Arch -->
                          <div class="info-primary-row">
                            <span class="cpu-chip" :title="agent.sysInfo?.cpu_model">
                              {{ agent.sysInfo?.cpu_model || 'Unknown CPU' }}
                            </span>
                            <span class="arch-badge">{{ agent.sysInfo?.arch || 'UNK' }}</span>
                          </div>

                          <!-- Bottom Line: Cores | Temp -->
                          <div class="info-secondary-row">
                            <span v-if="agent.sysInfo?.num_cpu" class="meta-item"
                              >{{ agent.sysInfo.num_cpu }} Cores</span
                            >
                            <span v-if="agent.sysInfo?.num_cpu" class="meta-divider">|</span>
                            <span
                              class="meta-item temp"
                              :class="{ hot: (agent.stats?.temperature || 0) > 70 }"
                            >
                              {{ Math.round(agent.stats?.temperature || 0) }}°C
                            </span>
                          </div>
                        </div>

                        <!-- 2. Split Layout (Rings | Stats) - 保持不变 -->
                        <div class="monitor-split">
                          <!-- Left: 3 Rings -->
                          <div class="rings-col">
                            <div class="ring-wrapper">
                              <el-progress
                                type="dashboard"
                                :percentage="Math.round(agent.stats?.cpu_usage || 0)"
                                :width="40"
                                :stroke-width="3"
                                :color="colors.cpu"
                              >
                                <template #default="{ percentage }"
                                  ><span class="ring-val">{{ percentage }}%</span></template
                                >
                              </el-progress>
                              <span class="ring-label">CPU</span>
                            </div>
                            <div class="ring-wrapper">
                              <el-progress
                                type="dashboard"
                                :percentage="Math.round(agent.stats?.gpu_usage || 0)"
                                :width="40"
                                :stroke-width="3"
                                :color="colors.gpu"
                              >
                                <template #default="{ percentage }"
                                  ><span class="ring-val">{{ percentage }}%</span></template
                                >
                              </el-progress>
                              <span class="ring-label">GPU</span>
                            </div>
                            <div class="ring-wrapper">
                              <el-progress
                                type="dashboard"
                                :percentage="Math.round(agent.stats?.mem_usage || 0)"
                                :width="40"
                                :stroke-width="3"
                                :color="colors.mem"
                              >
                                <template #default="{ percentage }"
                                  ><span class="ring-val">{{ percentage }}%</span></template
                                >
                              </el-progress>
                              <span class="ring-label">RAM</span>
                            </div>
                          </div>

                          <!-- Right: Stats List -->
                          <div class="stats-col">
                            <div class="stat-row">
                              <el-icon class="icon-down"><Download /></el-icon>
                              <span class="stat-text">{{
                                formatSpeed(agent.stats?.net_rx_rate)
                              }}</span>
                            </div>
                            <div class="stat-row">
                              <el-icon class="icon-up"><Upload /></el-icon>
                              <span class="stat-text">{{
                                formatSpeed(agent.stats?.net_tx_rate)
                              }}</span>
                            </div>
                            <div
                              class="stat-row"
                              :class="{ 'disk-full': (agent.stats?.disk_usage || 0) > 90 }"
                            >
                              <el-icon class="icon-disk"><Files /></el-icon>
                              <span class="stat-text"
                                >{{ Math.round(agent.stats?.disk_usage || 0) }}% Disk</span
                              >
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
                            <component
                              :is="agent.status === 'setting_up' ? Loading : SwitchButton"
                            />
                          </el-icon>
                          <span class="state-text">{{ getStatusText(agent.status) }}</span>
                          <div class="agent-ip">{{ agent.ip }}</div>
                        </div>
                      </template>
                    </div>

                    <div class="card-footer" @click.stop>
                      <template v-if="agent.status === 'ready'">
                        <el-button
                          class="footer-btn primary"
                          text
                          bg
                          @click="handleCardClick(agent)"
                        >
                          <el-icon><Monitor /></el-icon> 查看详情
                        </el-button>
                        <div class="footer-divider"></div>
                        <el-button
                          class="footer-btn danger"
                          text
                          bg
                          @click="handleDisconnect(agent)"
                        >
                          <el-icon><SwitchButton /></el-icon>
                        </el-button>
                      </template>
                      <template v-else>
                        <el-button class="footer-btn" text bg disabled>
                          {{ agent.status === 'setting_up' ? '连接中...' : '点击卡片连接' }}
                        </el-button>
                      </template>
                    </div>
                  </div>

                  <!-- Back Side (Split Columns) -->
                  <!-- [需求2] 背面分为两列：节点和序列 -->
                  <div class="card-face back" @click="flipCard(agent.uniqueKey)">
                    <div class="card-header back-header">
                      <span>快速启动</span>
                      <el-icon><Back /></el-icon>
                    </div>

                    <div class="split-body">
                      <!-- Left: Nodes -->
                      <div class="split-col border-right">
                        <div class="col-title">Nodes</div>
                        <el-scrollbar>
                          <div class="mini-list">
                            <template v-if="agent.nodes && agent.nodes.length > 0">
                              <div
                                v-for="node in agent.nodes"
                                :key="node.id"
                                class="mini-item"
                                :class="{
                                  'process-start':
                                    getPendingState(agent.uniqueKey, node.id) === 'start',
                                  'process-stop':
                                    getPendingState(agent.uniqueKey, node.id) === 'stop',
                                  'is-active': node.status === 'running' /* [新增] 运行状态类名 */
                                }"
                                @click.stop="toggleNode(agent, node)"
                              >
                                <div class="mini-status" :class="node.status"></div>
                                <span class="mini-name" :title="node.name">{{ node.name }}</span>
                                <!-- 如果正在处理，显示 Loading 图标，否则显示播放/停止 -->
                                <el-icon
                                  class="mini-action"
                                  :class="{
                                    'is-loading': getPendingState(agent.uniqueKey, node.id),
                                    stop: node.status === 'running',
                                    play: node.status !== 'running'
                                  }"
                                >
                                  <component
                                    :is="
                                      getPendingState(agent.uniqueKey, node.id)
                                        ? Loading
                                        : node.status === 'running'
                                          ? SwitchButton
                                          : VideoPlay
                                    "
                                  />
                                </el-icon>
                              </div>
                            </template>
                            <div v-else class="mini-empty">No Nodes</div>
                          </div>
                        </el-scrollbar>
                      </div>

                      <!-- Right: Sequences -->
                      <div class="split-col">
                        <div class="col-title">Sequences</div>
                        <el-scrollbar>
                          <div class="mini-list">
                            <template v-if="agent.sequences && agent.sequences.length > 0">
                              <div
                                v-for="seq in agent.sequences"
                                :key="seq.id"
                                class="mini-item"
                                :class="{
                                  'process-start':
                                    getPendingState(agent.uniqueKey, seq.id) === 'start',
                                  'process-stop':
                                    getPendingState(agent.uniqueKey, seq.id) === 'stop'
                                }"
                                @click.stop="toggleSequence(agent, seq)"
                              >
                                <div
                                  class="mini-status"
                                  :class="seq._status === 'running' ? 'running' : 'stopped'"
                                ></div>
                                <span class="mini-name" :title="seq.name">{{ seq.name }}</span>
                                <el-icon
                                  class="mini-action"
                                  :class="{
                                    'is-loading': getPendingState(agent.uniqueKey, seq.id),
                                    stop: seq._status === 'running',
                                    play: seq._status !== 'running'
                                  }"
                                >
                                  <component
                                    :is="
                                      getPendingState(agent.uniqueKey, seq.id)
                                        ? Loading
                                        : seq._status === 'running'
                                          ? SwitchButton
                                          : VideoPlay
                                    "
                                  />
                                </el-icon>
                              </div>
                            </template>
                            <div v-else class="mini-empty">No Sequences</div>
                          </div>
                        </el-scrollbar>
                      </div>
                    </div>

                    <div class="card-footer back-footer" @click.stop>
                      <el-button class="footer-btn primary" text bg @click="handleCardClick(agent)">
                        <el-icon><Monitor /></el-icon> 查看详情
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
        </div>

        <div class="panel-watermark">
          <el-icon><Connection /></el-icon> Agent
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { onMounted, onUnmounted, computed, reactive } from 'vue'
import {
  Monitor,
  Connection,
  VideoPlay,
  SwitchButton,
  Loading,
  Download,
  Upload,
  Files,
  Back,
  Timer,
  Iphone,
  Plus,
  MagicStick,
  Delete
} from '@element-plus/icons-vue'
import { useRobotStore } from '../store/robot'
import { ElMessage, ElMessageBox } from 'element-plus'

const emit = defineEmits(['switch-view', 'add', 'first-setup', 'delete'])
const robotStore = useRobotStore()
const props = defineProps({
  savedBackends: { type: Array, default: () => [] }
})

// 颜色配置
const colors = {
  cpu: [
    { color: '#67c23a', percentage: 40 },
    { color: '#e6a23c', percentage: 70 },
    { color: '#f56c6c', percentage: 100 }
  ],
  mem: '#409eff',
  gpu: '#8e44ad' // [新增] 紫色代表 GPU
}

// 主机信息
const hostInfo = computed(() => robotStore.hostSystemState.info)
const hostCpuUsage = computed(() => robotStore.hostSystemState.cpuUsage)
let refreshTimer = null

const refreshHostStats = () => {
  robotStore.updateHostSystemState()
}

// ------------------- 核心逻辑 -------------------

const allAgents = computed(() => {
  return props.savedBackends.map((item, index) => {
    const activeClient = Object.values(robotStore.clients).find(
      (c) => c.ip === item.settings.ip || (item.settings.id && c.id === item.settings.id)
    )
    if (activeClient) {
      return {
        ...activeClient,
        name: item.name,
        ip: item.settings.ip,
        port: item.settings.port || 8080,
        uniqueKey: activeClient.id,
        status: activeClient.status,
        settings: item.settings,
        stats: activeClient.stats || {},
        originalIndex: index,
        nodes: activeClient.nodes || [],
        sequences: activeClient.sequences || [],
        sysInfo: activeClient.sysInfo || {},
        serviceStatus: activeClient.serviceStatus || { roscore: 'unknown', bridge: 'unknown' },
        isStackLoading: activeClient.isStackLoading || false
      }
    } else {
      return {
        id: item.settings.id,
        ip: item.settings.ip,
        port: item.settings.port || 8080,
        name: item.name,
        status: 'disconnected',
        uniqueKey: item.settings.ip,
        settings: item.settings,
        stats: {},
        originalIndex: index,
        nodes: [],
        sequences: [],
        sysInfo: {},
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' }
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
  const total = hostInfo.value.totalmem
  const free = hostInfo.value.freemem
  if (!total) return 0
  return Math.round(((total - free) / total) * 100)
})

const getStatusText = (status) => {
  const map = {
    ready: '在线',
    setting_up: '连接中...',
    disconnected: '点击以连接到',
    failed: '连接失败'
  }
  return map[status] || status
}
const isWifi = (name) => {
  if (!name) return false
  const lower = name.toLowerCase()
  return lower.includes('wi') || lower.includes('wl')
}

// ------------------- 交互 -------------------
const flippedCards = reactive({})
const flipCard = (key) => {
  flippedCards[key] = !flippedCards[key]
}
// 临时状态记录 (Key: agentId_itemId, Value: 'start' | 'stop')
const pendingStates = reactive({})
const getPendingState = (agentId, itemId) => {
  return pendingStates[`${agentId}_${itemId}`]
}

// [新增] ROS 栈控制函数
const toggleRosStack = async (agent) => {
  if (agent.isStackLoading) return // 防止重复点击

  // 状态判断：只要有一个没启动，目标就是“启动”；只有全部启动了，目标才是“停止”
  const roscore = agent.serviceStatus?.roscore === 'active'
  const bridge = agent.serviceStatus?.bridge === 'active'
  const isFullyActive = roscore && bridge
  const action = isFullyActive ? 'stop' : 'start'

  try {
    // 如果是停止操作，需要二次确认
    if (action === 'stop') {
      await ElMessageBox.confirm('确定要停止 ROS 服务吗？这将停止所有 ROS 进程。', '停止确认', {
        confirmButtonText: '停止服务',
        cancelButtonText: '取消',
        type: 'warning',
        confirmButtonClass: 'el-button--danger'
      })
    }
    // 执行操作
    await robotStore.controlRosStack(agent.uniqueKey, action)
    ElMessage.success(action === 'start' ? '正在启动 ROS 服务...' : '正在停止 ROS 服务...')
  } catch (e) {
    if (e !== 'cancel') {
      ElMessage.error(e.message || '操作失败')
    }
  }
}

// [需求2] 节点控制
const toggleNode = async (agent, node) => {
  const key = `${agent.uniqueKey}_${node.id}`
  const isRunning = node.status === 'running'
  // 设置临时状态
  pendingStates[key] = isRunning ? 'stop' : 'start'
  // 预期目标状态
  const targetStatus = isRunning ? 'stopped' : 'running'

  try {
    if (node.status === 'running') {
      await robotStore.stopNodeProcess(agent.uniqueKey, node.name)
    } else {
      await robotStore.startNodeProcess(agent.uniqueKey, node)
    }
    // 3. [核心修复] 等待状态真正同步
    // 设置一个 5秒 的超时保护，避免网络卡死时动画一直转
    const timeout = 5000
    const startWait = Date.now()

    while (Date.now() - startWait < timeout) {
      // 如果 Store 中的状态已经变成了我们预期的状态 (由后台心跳更新)，则退出循环，结束动画
      if (node.status === targetStatus) break
      // 如果出现错误，也退出
      if (node.status === 'error' || node.status === 'crashed') break
      // 每 200ms 检查一次
      await new Promise((r) => setTimeout(r, 200))
    }
  } catch (e) {
    ElMessage.error(e.message)
  } finally {
    // 4. 关闭动画
    delete pendingStates[key]
  }
}

// [需求2] 序列控制
const toggleSequence = async (agent, seq) => {
  const key = `${agent.uniqueKey}_${seq.id}`
  const isRunning = seq._status === 'running'

  pendingStates[key] = isRunning ? 'stop' : 'start'

  try {
    if (seq._status === 'running') {
      await robotStore.stopSequenceNodes(agent.uniqueKey, seq)
    } else {
      ElMessage.info(`Executing: ${seq.name}`)
      await robotStore.runSequence(agent.uniqueKey, seq)
    }
  } catch (e) {
    ElMessage.error(e.message)
  } finally {
    delete pendingStates[key]
  }
}

const handleDisconnect = (agent) =>
  emit('switch-view', { index: agent.originalIndex, action: 'disconnect' })
const handleCardClick = (agent) => {
  if (agent.status === 'ready') emit('switch-view', { index: agent.originalIndex, action: 'jump' })
  else emit('switch-view', { index: agent.originalIndex, action: 'connect' })
}
const handleDelete = (index) => emit('delete', index)

const refreshAllAgents = async () => {
  for (const backend of props.savedBackends) {
    const id = backend.settings.id
    const client = robotStore.clients[id]
    if (client && client.status === 'ready') {
      robotStore.refreshStatus(id).catch((e) => console.warn('Overview refresh failed:', e))
    }
  }
}

onMounted(() => {
  refreshHostStats()
  refreshTimer = setInterval(refreshHostStats, 3000)
  refreshAllAgents()
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
  height: 20px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid var(--panel-border-color);
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
  /* [修改] 隐藏原生滚动条，交给 el-scrollbar */
  overflow: hidden;
  padding: 0;
  display: flex;
  flex-direction: column;
  z-index: 1;
  min-height: 0; /* 允许 Flex 收缩 */
}

/* [新增] 强制 Scrollbar View 占满高度并使用 Flex 布局 */
/* 这样可以保证 .net-section 能够通过 flex: 1 自动填满底部剩余空间 */
.host-content :deep(.el-scrollbar__view) {
  min-height: 100%;
  display: flex;
  flex-direction: column;
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
  gap: 5px; /* 增加一点间距 */
}
.cm-val {
  font-size: 16px;
  font-weight: 700;
  color: var(--text-primary);
}
.cm-label {
  font-size: 12px;
  color: var(--text-secondary);
  font-weight: 600;
  margin-top: 0; /* 之前是 -5px，现在自然排列 */
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
  /* [修改] 保持 flex: 1，使其在内容较少时自动拉伸填满底部 */
  flex: 1;
  display: flex;
  flex-direction: column;
  background: rgba(128, 128, 128, 0.03);
  padding: 15px 20px;
  border-top: 1px solid var(--divider-color);
  /* 移除之前的 min-height: 0，因为现在是在 scrollbar 内部自然生长 */
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
.enhanced-progress :deep(path:first-child) {
  stroke: rgba(0, 0, 0, 0.1) !important; /* 浅色模式下的深轨 */
}
:global(html.dark) .enhanced-progress :deep(path:first-child) {
  stroke: rgba(255, 255, 255, 0.15) !important; /* 深色模式下的亮轨 */
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
.el-scrollbar {
  overflow: auto;
}
.fleet-list-wrapper {
  flex: 1; /* 自动占据剩余高度 */
  min-height: 0; /* [关键] 允许 Flex 子项收缩，防止被内容撑爆 */
  width: 100%;
  position: relative;
  overflow: hidden; /* 确保 scrollbar 不会溢出 */
}
.fleet-list-wrapper :deep(.el-scrollbar__view) {
  display: block !important;
  width: 100%;
  /* 确保 view 能够被内容撑开，不强制 100% 导致截断 */
  min-height: 100%;
}
.agent-grid {
  display: grid;
  /* 自动填充列，最小宽度 260px */
  grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
  gap: 15px;
  /* 将 padding 移到这里，替代 view-style */
  padding: 10px;
  /* 确保 Grid 本身高度是 auto，能被子元素撑开 */
  height: auto;
  align-content: start; /* 内容不足时靠上对齐，防止拉伸 */
}

.flip-container {
  perspective: 1000px;
  height: 220px;
  width: 100%;
  z-index: 1;
  position: relative;
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
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  backface-visibility: hidden; /* 关键：隐藏背面 */
  background: var(--card-inner-bg);
  border: 1px solid var(--panel-border-color);
  border-radius: 16px;
  box-shadow: var(--card-shadow);
  display: flex;
  flex-direction: column;
  overflow: hidden;
  transition: all 0.3s; /* 悬浮动效过渡 */
  cursor: pointer;
  /* 硬件加速防止闪烁 */
  transform: translate3d(0, 0, 0);
  -webkit-transform: translate3d(0, 0, 0);
  transform-style: flat; /* 确保子元素在平面内 */
}
.card-face.front {
  /* 默认 0度 */
  transform: rotateY(0deg);
  z-index: 2;
}
.card-face.front:hover {
  transform: rotateY(0deg) translateY(-4px); /* 保持角度，增加位移 */
  box-shadow: 0 12px 32px rgba(0, 0, 0, 0.15);
  border-color: rgba(64, 158, 255, 0.4);
}

/* --- 背面 (Back) --- */
.card-face.back {
  transform: rotateY(180deg);
  background: var(--bg-color); /* 背面背景色稍作区分 */
}

/* 背面悬浮：上浮 */
.card-face.back:hover {
  transform: rotateY(180deg) translateY(-4px);
  box-shadow: 0 12px 32px rgba(0, 0, 0, 0.15);
  border-color: rgba(64, 158, 255, 0.4);
}

.status-online {
  border-left: 3px solid #67c23a;
}
.status-offline {
  border-left: 3px solid var(--divider-color);
  opacity: 0.9;
}

/* Front Header */
.card-header {
  padding: 10px 8px 8px 15px;
  height: 32px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid var(--divider-color);
  background: rgba(128, 128, 128, 0.03);
}
.header-left {
  display: flex;
  align-items: center;
  gap: 8px;
  overflow: hidden;
  max-width: 80%;
}
.id-text-group {
  display: flex;
  flex-direction: column;
  justify-content: center;
  overflow: hidden;
  line-height: 1.2;
  gap: 2px;
}
.agent-address {
  font-size: 10px;
  color: var(--text-secondary);
  font-family: 'Consolas', monospace;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  opacity: 0.8;
}
.agent-name {
  font-weight: 700;
  font-size: 14px;
  color: var(--text-primary);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.status-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #909399;
  flex-shrink: 0;
}
.status-indicator.ready {
  background: #67c23a;
  box-shadow: 0 0 6px #67c23a;
}
.status-indicator.setting_up {
  background: #e6a23c;
  animation: blink 1s infinite;
}

/* [需求1] ROS Capsule */
/* ============================================
   ROS Control Group (Split Visuals, Unified Interaction)
   ============================================ */
.header-right-group {
  display: flex;
  align-items: center;
  gap: 10px;
}

/* 1. 父容器：负责交互区域和整体布局 */
.ros-control-group {
  position: relative; /* 为 loading 遮罩定位 */
  display: flex;
  align-items: center;
  gap: 4px; /* 两个胶囊之间的间距 */
  padding: 4px; /* 增加一点热区 */
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  background: transparent; /* 默认透明 */
  user-select: none;
}

/* 2. 子胶囊：负责各自的状态显示 */
.ros-pill {
  display: flex;
  align-items: center;
  gap: 4px;
  height: 20px;
  padding: 0 8px;
  border-radius: 10px; /* 椭圆胶囊 */
  font-size: 10px;
  font-weight: 700;
  font-family: 'Consolas', monospace;

  /* 默认状态 (Inactive): 灰色 */
  background-color: rgba(128, 128, 128, 0.1);
  color: var(--text-secondary);
  border: 1px solid transparent;
  transition: all 0.2s;
}

/* 子胶囊内的小圆点 */
.pill-dot {
  width: 5px;
  height: 5px;
  border-radius: 50%;
  background-color: currentColor; /* 跟随文字颜色 */
  opacity: 0.5;
  transition: all 0.3s;
}

/* 3. 子胶囊激活态 (Active): 蓝色 */
.ros-pill.active {
  background-color: rgba(103, 194, 58, 0.1);
  color: #67c23a;
  border-color: rgba(103, 194, 58, 0.2);
}
.ros-pill.active .pill-dot {
  opacity: 1;
  box-shadow: 0 0 4px currentColor;
}

/* ============================================
   交互逻辑 (Hover Group -> Affect Pills)
   ============================================ */

/* 场景 A: 未全开 (Hover -> Green/Start) */
.ros-control-group:not(.is-fully-active):not(.is-loading):hover {
  background-color: rgba(103, 194, 58, 0.05); /* 父容器微绿底 */
}
/* 强制内部所有胶囊变绿 */
.ros-control-group:not(.is-fully-active):not(.is-loading):hover .ros-pill {
  background-color: #67c23a; /* 实心绿背景 */
  color: white;
  border-color: transparent;
  transform: translateY(-1px);
}
.ros-control-group:not(.is-fully-active):not(.is-loading):hover .pill-dot {
  background-color: white;
  opacity: 1;
  box-shadow: none;
}

/* 场景 B: 全开 (Hover -> Red/Stop) */
.ros-control-group.is-fully-active:not(.is-loading):hover {
  background-color: rgba(245, 108, 108, 0.05); /* 父容器微红底 */
}
/* 强制内部所有胶囊变红 */
.ros-control-group.is-fully-active:not(.is-loading):hover .ros-pill {
  background-color: #f56c6c; /* 实心红背景 */
  color: white;
  border-color: transparent;
  transform: translateY(-1px);
}
.ros-control-group.is-fully-active:not(.is-loading):hover .pill-dot {
  background-color: white;
  opacity: 1;
  box-shadow: none;
}

/* ============================================
   Loading 状态
   ============================================ */
.ros-control-group.is-loading {
  cursor: not-allowed;
  opacity: 0.7;
}
.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(255, 255, 255, 0.6); /* 遮罩 */
  z-index: 10;
  border-radius: 8px;
}
:global(html.dark) .loading-overlay {
  background: rgba(0, 0, 0, 0.6);
}

/* ============================================
   Card Body Layout (Compacted)
   ============================================ */
.card-body {
  flex: 1;
  padding: 12px 15px;
  margin-top: 2px;
  display: flex;
  flex-direction: column;
  gap: 2px; /* 增加各区块间距 */
  overflow: hidden;
  justify-content: center;
}

/* 1. Info Section */
.info-section {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

/* 第一行：型号 + 架构 */
.info-primary-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}

.cpu-chip {
  font-size: 11px;
  font-weight: 700;
  color: var(--text-primary);
  flex: 1; /* 占据剩余空间 */
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  margin-right: 8px; /* 与右侧架构标签保持距离 */
}

.arch-badge {
  font-size: 9px;
  background: rgba(128, 128, 128, 0.1);
  padding: 1px 5px;
  border-radius: 4px;
  color: var(--text-secondary);
  font-weight: 700;
  text-transform: uppercase;
  flex-shrink: 0; /* 不许压缩 */
}

/* 第二行：核心 + 温度 */
.info-secondary-row {
  display: flex;
  align-items: center;
  gap: 6px;
}

.meta-item {
  font-size: 10px;
  color: var(--text-secondary);
  font-family: 'Consolas', monospace;
  font-weight: 600;
}
.meta-item.temp.hot {
  color: #f56c6c;
}
.meta-divider {
  font-size: 9px;
  color: var(--divider-color);
  opacity: 0.6;
}

/* 2. Main Monitor Split (Left Rings | Right Stats) */
.monitor-split {
  display: flex;
  align-items: center;
  justify-content: space-between;
  flex: 1; /* 占满剩余高度 */
  gap: 10px;
}

/* --- Left: Rings --- */
.rings-col {
  flex: 3; /* 占比 60% */
  display: flex;
  justify-content: space-between; /* 均匀分布 */
  align-items: flex-end; /* 底部对齐 */
}

.ring-wrapper {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0;
  margin-top: -2px; /* 修正 el-progress 底部留白 */
}

.ring-val {
  font-size: 9px;
  font-weight: 700;
  color: var(--text-primary);
  font-family: 'Consolas', monospace;
  line-height: 1;
}
/* 调整环形进度条内部文字位置 */
.ring-wrapper :deep(.el-progress__text) {
  top: 50%;
  transform: translate(-10%, -50%);
}
/* 标签 */
.ring-label {
  font-size: 9px;
  color: var(--text-secondary);
  font-weight: 600;
  margin-top: -2px; /* 紧贴圆环 */
}

/* --- Right: Stats List --- */
.stats-col {
  flex: 2; /* 占比 40% */
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 6px;
  background: rgba(128, 128, 128, 0.05);
  padding: 6px 8px;
  border-radius: 6px;
  align-items: flex-end;
}

.stat-row {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 10px;
  color: var(--text-secondary);
  font-family: 'Consolas', monospace;
  white-space: nowrap;
}

/* Icons */
.stat-row .el-icon {
  font-size: 12px;
}
.icon-down {
  color: #409eff;
}
.icon-up {
  color: #67c23a;
}
.icon-disk {
  color: #909399;
}

/* Warning State */
.stat-row.disk-full {
  color: #f56c6c;
  font-weight: 700;
}
.stat-row.disk-full .icon-disk {
  color: #f56c6c;
}

/* Offline State (不变) */
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

/* Front Footer */
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
:deep(.footer-btn.el-button > span) {
  gap: 8px;
  font-size: 13px;
}
.footer-btn.primary {
  color: #409eff;
}
.footer-btn.primary:hover {
  background: rgba(64, 158, 255, 0.1);
}
.footer-btn.danger {
  color: #f56c6c !important;
  background: rgba(245, 108, 108, 0.1) !important;
  max-width: 45px;
  border-left: 1px solid var(--divider-color);
  transition: all 0.3s ease;
}
:deep(.footer-btn.el-button.danger > span) {
  font-size: 15px;
}

.footer-btn.danger:hover {
  color: white !important;
  background: #f56c6c !important;
  transition: all 0.3s ease;
}
.footer-divider {
  width: 1px;
  height: 20px;
  background: var(--divider-color);
  align-self: center;
}

/* Back Side (Split) */
/* [需求2] */
.back-header {
  background: transparent;
  color: var(--text-primary);
  font-weight: 700;
  font-size: 13px;
}
.split-body {
  flex: 1;
  display: flex;
  overflow: hidden;
}
.split-col {
  flex: 1;
  display: flex;
  flex-direction: column;
  min-width: 0;
}
.split-col.border-right {
  border-right: 1px solid var(--divider-color);
}
.col-title {
  font-size: 10px;
  font-weight: 700;
  color: var(--text-secondary);
  text-transform: uppercase;
  padding: 8px 10px 4px;
  background: rgba(128, 128, 128, 0.02);
}

.mini-list {
  padding: 5px;
  display: flex;
  flex-direction: column;
  gap: 4px;
}
.mini-item {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 6px 8px;
  border-radius: 6px;
  background: rgba(128, 128, 128, 0.05);
  border: 1px solid transparent; /* 预留边框 */
  cursor: pointer;
  transition: all 0.2s;
  position: relative;
  overflow: hidden;
}
.mini-item:hover {
  background: rgba(64, 158, 255, 0.1);
  color: #409eff;
}
/* [新增] 运行中状态 (蓝色高亮) */
.mini-item.is-active {
  background-color: rgba(64, 158, 255, 0.1); /* 浅蓝底 */
  border-color: rgba(64, 158, 255, 0.2); /* 浅蓝框 */
}
.mini-item.is-active .mini-name {
  color: #409eff; /* 蓝字 */
  font-weight: 700;
}
.mini-item.is-active .mini-action {
  color: #f56c6c; /* 停止按钮显红，形成对比 */
}
:global(html.dark) .mini-item.is-active {
  background-color: rgba(64, 158, 255, 0.15);
  border-color: rgba(64, 158, 255, 0.3);
}

/* ============================================
   能量流动画 (Energy Flow)
   ============================================ */

/* 1. 启动动画 (淡绿色光波) */
.mini-item.process-start::after {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  bottom: 0;
  width: 200%; /* 宽度两倍，方便移动 */
  background: linear-gradient(
    90deg,
    transparent 0%,
    rgba(103, 194, 58, 0.2) 25%,
    rgba(103, 194, 58, 0.6) 50%,
    rgba(103, 194, 58, 0.2) 75%,
    transparent 100%
  );
  transform: translateX(-100%);
  animation: energy-run 1.5s infinite linear;
  z-index: 0; /* 在背景之上，文字之下 */
  pointer-events: none;
}

/* 2. 停止动画 (淡红色光波) */
.mini-item.process-stop::after {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  bottom: 0;
  width: 200%;
  background: linear-gradient(
    90deg,
    transparent 0%,
    rgba(245, 108, 108, 0.2) 25%,
    rgba(245, 108, 108, 0.6) 50%,
    rgba(245, 108, 108, 0.2) 75%,
    transparent 100%
  );
  transform: translateX(-100%);
  animation: energy-run 1.5s infinite linear;
  z-index: 0;
  pointer-events: none;
}

/* 确保内容在光波之上 */
.mini-status,
.mini-name,
.mini-action {
  position: relative;
  z-index: 1;
}

/* Loading 图标旋转 */
.mini-action.is-loading {
  animation: spin 1s infinite linear;
  color: var(--text-primary);
}

@keyframes energy-run {
  0% {
    transform: translateX(-100%);
  }
  100% {
    transform: translateX(100%);
  }
}
.mini-status {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #909399;
}
.mini-status.running {
  background: #67c23a;
  box-shadow: 0 0 4px #67c23a;
}
.mini-name {
  flex: 1;
  font-size: 11px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  font-weight: 600;
}
.mini-action {
  font-size: 14px;
  color: var(--text-secondary);
}
.mini-action.stop {
  color: #f56c6c;
}
.mini-action.play {
  color: #67c23a;
}
.mini-empty {
  font-size: 10px;
  color: var(--text-secondary);
  text-align: center;
  margin-top: 20px;
  font-style: italic;
}

.back-footer {
  height: 36px;
}

/* Add New Card */
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
