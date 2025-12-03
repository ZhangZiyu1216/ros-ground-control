<template>
  <div class="app-container">
    <!-- 1. Header: 顶部工具栏 -->
    <header class="app-glass-header drag-region">
      <div class="header-left">
        <div class="app-branding">
          <span class="app-name">ROS Ground Control</span>
        </div>
      </div>

      <div class="header-right no-drag">
        <!-- 功能按钮组 -->
        <div class="function-buttons">
          <el-button text round class="win-func-btn" @click="openEditorWindow">
            <el-icon>
              <EditPen />
            </el-icon>
            <span>编辑</span>
          </el-button>
          <el-button text round class="win-func-btn" @click="showSettings = true">
            <el-icon>
              <Tools />
            </el-icon>
            <span>设置</span>
          </el-button>
          <el-button text round class="win-func-btn" @click="showInfoDialog = true">
            <el-icon>
              <InfoFilled />
            </el-icon>
            <span>关于</span>
          </el-button>
        </div>

        <!-- Windows 风格窗口控制 -->
        <div class="window-controls windows-style">
          <div class="win-ctrl-btn min" @click="minimizeWindow">
            <el-icon>
              <Minus />
            </el-icon>
          </div>
          <div class="win-ctrl-btn max" @click="maximizeWindow">
            <el-icon>
              <FullScreen />
            </el-icon>
          </div>
          <div class="win-ctrl-btn close" @click="closeWindow">
            <el-icon>
              <Close />
            </el-icon>
          </div>
        </div>
      </div>
    </header>

    <!-- 2. Main: 内容区域 -->
    <main class="app-main-content">
      <div v-show="activeTab === 'dashboard'" class="view-wrapper">
        <!-- 模式 A: 总览模式 -->
        <OverviewDashboard
          v-if="isOverviewMode"
          :saved-backends="savedBackends"
          @switch-view="handleOverviewSwitch"
          @add="openAddDialog"
          @first-setup="openFirstTimeSetupDialog"
          @delete="deleteBackend"
        />
        <!-- 模式 B: 详情模式 -->
        <Dashboard
          v-else
          :current-backend-id="currentBackendId"
          :is-current-backend-ready="isCurrentBackendReady"
        />
      </div>
      <div v-show="activeTab === 'monitor'" class="view-wrapper">
        <TopicMonitor
          :current-backend-id="currentBackendId"
          :is-current-backend-ready="isCurrentBackendReady"
        />
      </div>
      <div v-show="activeTab === 'logs'" class="view-wrapper">
        <LogManager
          :current-backend-id="currentBackendId"
          :is-current-backend-ready="isCurrentBackendReady"
        />
      </div>
    </main>

    <!-- 3. Footer: 底部栏 -->
    <footer class="app-glass-footer">
      <!-- Left: 连接配置 -->
      <div class="footer-section left">
        <!-- 1. 连接列表按钮 -->
        <el-popover
          v-model:visible="isPopoverVisible"
          placement="top-start"
          :width="320"
          trigger="hover"
          :show-after="100"
          :hide-after="200"
          popper-class="backend-list-popover"
          :show-arrow="false"
          :offset="15"
        >
          <template #reference>
            <el-button
              class="current-conn-btn"
              :type="backendStatusProps.type"
              text
              :style="{
                '--el-button-text-color': backendStatusProps.color,
                '--el-button-hover-text-color': backendStatusProps.color,
                '--el-button-active-text-color': backendStatusProps.color
              }"
              @click="handleConfigButtonClick"
            >
              <el-icon :class="{ 'is-loading': currentBackendStatus === 'setting_up' }">
                <component :is="backendStatusProps.icon" />
              </el-icon>
              <span class="conn-text">{{ backendStatusProps.text }}</span>
            </el-button>
          </template>

          <!-- Popover 内容 (保持不变) -->
          <div class="backend-list-container">
            <el-scrollbar max-height="280px">
              <!-- 固定总览项 -->
              <el-divider class="backend-list-header" content-position="left">可用连接</el-divider>
              <div
                class="backend-item overview-item"
                :class="{ 'is-active': isOverviewMode }"
                @click="switchToOverview"
              >
                <div class="menu-dot-container">
                  <el-icon><Menu /></el-icon>
                </div>
                <div class="backend-info">
                  <div class="backend-name">连接总览</div>
                  <div class="backend-detail">系统控制中心</div>
                </div>
                <div v-if="isOverviewMode" class="active-check">
                  <el-icon><Check /></el-icon>
                </div>
              </div>
              <div
                v-for="(item, index) in savedBackends"
                :key="index"
                class="backend-item"
                :class="{ 'is-active': isActiveBackend(item), 'is-disabled': isAnyLoading }"
                @click="!isAnyLoading && switchToBackend(item)"
              >
                <div class="status-dot-container">
                  <div class="list-status-dot" :class="getItemStatus(item)"></div>
                </div>
                <div class="backend-info">
                  <div class="backend-name">{{ getBackendDisplayName(item) }}</div>
                  <div class="backend-detail">
                    {{ item.settings.mode === 'local' ? '127.0.0.1' : item.settings.ip }}
                  </div>
                </div>
                <div class="backend-actions" @click.stop>
                  <el-button
                    v-if="!isBackendRunning(item)"
                    link
                    class="action-icon del"
                    @click="deleteBackend(index)"
                    ><el-icon> <Delete /> </el-icon
                  ></el-button>
                  <el-button
                    v-if="getItemStatus(item) === 'ready'"
                    link
                    class="action-icon stop"
                    @click="handleManualDisconnect(item)"
                    ><el-icon> <SwitchButton /> </el-icon
                  ></el-button>
                  <el-button
                    v-if="!isBackendRunning(item)"
                    link
                    class="action-icon edit"
                    @click="openEditDialog(index, item)"
                    ><el-icon> <Edit /> </el-icon
                  ></el-button>
                </div>
                <div v-if="isActiveBackend(item)" class="active-check">
                  <el-icon>
                    <Check />
                  </el-icon>
                </div>
              </div>
            </el-scrollbar>
            <div
              class="add-buttons-row"
              @mouseenter="handleAddHover(true)"
              @mouseleave="handleAddHover(false)"
            >
              <div class="add-btn-base" @click.stop="openAddDialog">
                <el-icon> <Plus /> </el-icon><span>新建连接</span>
              </div>
              <div
                class="slide-wrapper"
                :class="{ 'is-open': showFirstTimeSetupButton && isHoveringAdd }"
              >
                <div class="add-btn-slide orange-style" @click.stop="openFirstTimeSetupDialog">
                  <el-icon> <MagicStick /> </el-icon><span>首次连接</span>
                </div>
              </div>
            </div>
          </div>
        </el-popover>

        <!-- 分割线 -->
        <template v-if="!isOverviewMode">
          <div class="footer-divider"></div>

          <!-- 连接切换按钮 -->
          <el-button
            class="connect-switch-btn"
            :type="isConnected ? 'danger' : 'success'"
            :loading="isActionInProgress"
            :disabled="!isCurrentBackendReady"
            plain
            @click="toggleServiceConnection"
          >
            {{ connectionButtonText }}
          </el-button>
        </template>
      </div>

      <!-- Center: 核心导航 -->
      <div class="footer-section center">
        <div class="nav-capsule">
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'dashboard' }"
            @click="activeTab = 'dashboard'"
          >
            <el-icon> <Grid /> </el-icon><span>节点</span>
          </div>
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'monitor' }"
            @click="activeTab = 'monitor'"
          >
            <el-icon> <Monitor /> </el-icon><span>监控</span>
          </div>
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'logs' }"
            @click="activeTab = 'logs'"
          >
            <el-icon> <DataBoard /> </el-icon><span>日志</span>
          </div>
        </div>
      </div>

      <!-- Right: 状态指示器 -->
      <div class="footer-section right">
        <template v-if="!isOverviewMode">
          <!-- Roscore Status -->
          <div class="status-block" :class="roscoreProps.type">
            <span class="block-label">Core</span>
            <span class="block-value">{{ roscoreProps.text }}</span>
            <div class="block-dot" :class="roscoreStatus"></div>
          </div>

          <!-- Rosbridge Status -->
          <div class="status-block" :class="rosbridgeProps.type">
            <span class="block-label">Bridge</span>
            <span class="block-value">{{ rosbridgeProps.text }}</span>
            <div class="block-dot" :class="rosbridgeStatus"></div>
          </div>
        </template>
      </div>
    </footer>
  </div>

  <!-- Dialogs (不变) -->
  <ConnectionDialog
    v-model:visible="showConnection"
    :initial-data="dialogInitialData"
    @apply="handleConnectionApply"
  />
  <SettingsDialog
    v-model="showSettings"
    v-model:settings="appSettings"
    :is-connected="isConnected"
    @reset-config="handleAppReset"
  />
  <FirstTimeSetupDialog
    v-model="showFirstTimeSetupDialog"
    :is-deploying="isDeploying"
    @deploy="handleDeploy"
  />
  <InfoDialog v-model="showInfoDialog" />
</template>

<script setup>
// #region 1. Imports
import { ref, onMounted, computed, watch, toRaw } from 'vue'
import Dashboard from './Dashboard.vue'
import LogManager from './LogManager.vue'
import TopicMonitor from './TopicMonitor.vue'
import OverviewDashboard from '../components/OverviewDashboard.vue'
import ConnectionDialog from '../components/ConnectionDialog.vue'
import SettingsDialog from '../components/SettingsDialog.vue'
import FirstTimeSetupDialog from '../components/FirstTimeSetupDialog.vue'
import InfoDialog from '../components/InfoDialog.vue'
import { useRobotStore } from '../store/robot'
import { useConnection } from '../composables/useConnection.js'
import { ElMessage, ElMessageBox, ElNotification } from 'element-plus'
import {
  Grid,
  Monitor,
  DataBoard,
  Tools,
  InfoFilled,
  Link,
  Loading,
  Close,
  CloseBold,
  Edit,
  Delete,
  Check,
  Plus,
  SwitchButton,
  EditPen,
  MagicStick,
  Minus,
  FullScreen,
  Menu
} from '@element-plus/icons-vue'

// 窗口控制
const minimizeWindow = () => window.electronWindow?.minimize()
const maximizeWindow = () => window.electronWindow?.toggleMaximize()
const closeWindow = () => window.electronWindow?.close()
const isWindows = ref(false)
// #endregion

// #region 2. Global State & Configs
const robotStore = useRobotStore()
const {
  currentBackendId,
  currentBackendStatus,
  serviceStatus,
  roscoreStatus,
  rosbridgeStatus,
  pendingAction,
  isActionInProgress,
  allSessionStatuses,
  connect,
  switchView,
  resetView,
  disconnectBackend,
  connectServices,
  stopServices
} = useConnection()

// App Settings
const showSettings = ref(false)
const defaultAppSettings = {
  autoConnect: true,
  autoStartRos: false,
  autoStopBridge: false,
  autoStopRoscore: false
}
const appSettings = ref({ ...defaultAppSettings })

watch(
  appSettings,
  (newVal) => {
    window.api.setConfig('app_settings', toRaw(newVal))
  },
  { deep: true }
)

// Connection Data
const savedBackends = ref([])
const DEFAULT_SETTINGS = {
  mode: 'local',
  ip: '127.0.0.1',
  name: 'Localhost',
  id: 'local'
}
const connectionSettings = ref({ ...DEFAULT_SETTINGS })

// UI State
const activeTab = ref('dashboard')
const isPopoverVisible = ref(false)
const showConnection = ref(false)
const isWindowFocused = ref(true)

// Dialog State
const dialogMode = ref('add') // 'add' | 'edit'
const editingBackendIndex = ref(-1)
const dialogInitialData = ref({ mode: 'remote', ip: '', name: '' })

// First Time Setup
const showFirstTimeSetupButton = ref(false)
const showFirstTimeSetupDialog = ref(false)
const showInfoDialog = ref(false)
const isDeploying = ref(false)
const isHoveringAdd = ref(false)

// isOverviewMode: 总览模式
const isOverviewMode = ref(true)

// #endregion

// #region 3. Computed Props

const isCurrentBackendReady = computed(() => currentBackendStatus.value === 'ready')
const isConnected = computed(
  () => isCurrentBackendReady.value && serviceStatus.value === 'connected'
)
const isAnyLoading = computed(() =>
  Object.values(allSessionStatuses.value || {}).some((s) => s === 'setting_up')
)

const connectionModeText = computed(() => {
  if (isOverviewMode.value) return '总览模式'

  const s = connectionSettings.value
  if (!s) return '未配置'
  if (s.mode === 'local') return 'Localhost'
  return s.name ? `${s.name} (${s.ip})` : s.ip || 'Unknown'
})

const connectionButtonText = computed(() => {
  if (isActionInProgress.value) {
    if (pendingAction.value === 'connect') return '启动中...'
    if (pendingAction.value === 'disconnect') return '关闭中...'
    return serviceStatus.value === 'connected' ? '关闭中...' : '启动中...'
  }
  return isConnected.value ? '关闭ROS服务' : '启动ROS服务'
})

const backendStatusProps = computed(() => {
  if (isOverviewMode.value) {
    return { text: '连接总览', type: 'info', icon: Menu, color: '#909399' }
  }

  const common = { text: connectionModeText.value }
  switch (currentBackendStatus.value) {
    case 'setting_up':
      return { ...common, type: 'warning', icon: Loading, color: '#E6A23C' }
    case 'ready':
      return {
        ...common,
        type: isConnected.value ? 'disable' : 'success',
        icon: Link,
        color: '#409EFF'
      }
    case 'failed':
      return { ...common, type: 'danger', icon: CloseBold, color: '' }
    default:
      return { ...common, type: 'info', icon: Close, color: '' }
  }
})

// Helpers
function getBackendDisplayName(item) {
  if (item.name) return item.name
  return item.settings.mode === 'local' ? 'Localhost' : item.settings.ip || 'Unknown'
}

function findClientBySettings(settings) {
  return Object.values(robotStore.clients).find(
    (c) => c.ip === settings.ip || (settings.hostname && c.hostname === settings.hostname)
  )
}

function getItemStatus(item) {
  const client = findClientBySettings(item.settings)
  return client ? client.status : 'disconnected'
}

function isActiveBackend(item) {
  if (isOverviewMode.value) return false

  if (!currentBackendId.value) return false
  const client = findClientBySettings(item.settings)
  return client && client === robotStore.activeClient
}

function isBackendRunning(item) {
  const status = getItemStatus(item)
  return status === 'ready' || status === 'setting_up'
}
// #endregion

// #region 4. Logic & Actions

// 切换到总览
function switchToOverview() {
  resetView() // useConnection 中已有的方法，将 activeID 设为 null
  isOverviewMode.value = true
  isPopoverVisible.value = false
  activeTab.value = 'dashboard'
}

// 供 OverviewDashboard 组件调用的回调，用于点击卡片进入详情
const handleOverviewSwitch = async (payload) => {
  const { index, action } = payload
  // 1. 通过索引直接获取配置对象，解决 ID 不一致导致的查找失败问题
  const targetItem = savedBackends.value[index]

  if (!targetItem) {
    ElMessage.warning('无法找到对应的连接配置')
    return
  }

  switch (action) {
    case 'jump':
      // 跳转进详情页
      await switchToBackend(targetItem)
      activeTab.value = 'dashboard' // 确保 Tab 切回控制台
      break

    case 'connect':
      // 发起连接但停留在当前页
      await switchToBackend(targetItem, true)
      break

    case 'disconnect':
      // 断开连接
      if (targetItem) {
        // 复用现有的断开逻辑 (含确认弹窗)
        await handleManualDisconnect(targetItem)
      }
      break

    default:
      console.warn('Unknown action:', action)
  }
}

// 打开编辑器窗口 (调用 Store 方法以广播 Token)
function openEditorWindow() {
  robotStore.openEditor()
}

// 保存 IP 和 ID 变更
async function checkAndSaveIpChange() {
  const currentActiveID = robotStore.activeID
  const client = robotStore.clients[currentActiveID]

  if (client) {
    const savedItem = savedBackends.value.find(
      (b) =>
        (client.hostname && b.settings.hostname === client.hostname) || b.settings.ip === client.ip
    )
    if (savedItem) {
      let hasChange = false
      if (client.ipChanged || savedItem.settings.ip !== client.ip) {
        savedItem.settings.ip = client.ip
        hasChange = true
      }
      if (savedItem.settings.id !== currentActiveID) {
        savedItem.settings.id = currentActiveID
        hasChange = true
      }
      // Update local connectionSettings ref
      if (
        connectionSettings.value.hostname === client.hostname ||
        connectionSettings.value.ip === client.ip
      ) {
        connectionSettings.value.ip = client.ip
        connectionSettings.value.id = currentActiveID
      }

      if (hasChange) {
        await saveBackendsToStore()
        if (client.ipChanged) {
          ElNotification({
            title: '配置已更新',
            message: `IP 已更新为 ${client.ip}`,
            type: 'success'
          })
          client.ipChanged = false
        }
      }
    }
  }
}

// Dialogs
function handleAddHover(val) {
  isHoveringAdd.value = val
  showFirstTimeSetupButton.value = val
}
function openFirstTimeSetupDialog() {
  isPopoverVisible.value = false
  showFirstTimeSetupDialog.value = true
}
function openAddDialog() {
  dialogMode.value = 'add'
  dialogInitialData.value = { mode: 'remote', ip: '', name: '' }
  showConnection.value = true
}
function openEditDialog(index, item) {
  if (isBackendRunning(savedBackends.value[index])) return ElMessage.warning('请先断开连接')
  dialogMode.value = 'edit'
  editingBackendIndex.value = index
  dialogInitialData.value = JSON.parse(JSON.stringify(item.settings))
  showConnection.value = true
}

// Deployment
async function handleDeploy(sshSettings) {
  isDeploying.value = true
  try {
    const result = await window.api.deployAgent(sshSettings)
    if (result.success) {
      ElNotification({ title: '部署成功', message: 'Agent 已启动', type: 'success' })
      showFirstTimeSetupDialog.value = false
      const finalName = result.hostname || sshSettings.host
      dialogMode.value = 'add'
      dialogInitialData.value = {
        mode: 'remote',
        ip: sshSettings.host,
        name: finalName,
        hostname: finalName
      }
      setTimeout(() => {
        showConnection.value = true
      }, 1000)
    } else {
      throw new Error(result.message)
    }
  } catch (e) {
    ElMessage.error(`部署失败: ${e.message}`)
  } finally {
    isDeploying.value = false
  }
}

// Connection Management
async function handleConnectionApply(newSettings) {
  if (dialogMode.value === 'add') {
    const exists = savedBackends.value.some((b) => b.settings.ip === newSettings.ip)
    if (exists) return ElMessage.warning('配置已存在')
    savedBackends.value.push({ name: newSettings.name || newSettings.ip, settings: newSettings })
    await saveBackendsToStore()
  } else if (dialogMode.value === 'edit') {
    const target = savedBackends.value[editingBackendIndex.value]
    target.settings = newSettings
    target.name = newSettings.name || newSettings.ip
    await saveBackendsToStore()
    showConnection.value = false
    ElNotification({ title: '配置已更新', type: 'success' })
    return
  }

  showConnection.value = false
  const previousSettings = JSON.parse(JSON.stringify(connectionSettings.value))
  const previousId = currentBackendId.value
  connectionSettings.value = newSettings

  try {
    await connect(newSettings)
    ElNotification({ title: '连接成功', type: 'success' })
  } catch (error) {
    ElNotification({ title: '连接失败', message: error.message, type: 'error' })
    setTimeout(() => {
      connectionSettings.value = previousSettings
      if (previousId) switchView(previousId)
      else resetView()
      // 清理无效的 Add
      if (dialogMode.value === 'add') {
        const idx = savedBackends.value.findIndex((b) => b.settings.ip === newSettings.ip)
        if (idx !== -1) {
          savedBackends.value.splice(idx, 1)
          saveBackendsToStore()
        }
      }
      robotStore.setClientStatus(newSettings.ip, 'disconnected')
    }, 2000)
  }
}

async function switchToBackend(item, stopJump = false) {
  isPopoverVisible.value = false
  const client = findClientBySettings(item.settings)
  const clientKey = client
    ? client.id || Object.keys(robotStore.clients).find((k) => robotStore.clients[k] === client)
    : null

  if (!isOverviewMode.value && currentBackendId.value && clientKey === currentBackendId.value)
    return

  // 已经连接且状态为 Ready，总是直接跳转
  if (client && client.status === 'ready') {
    switchView(clientKey)
    connectionSettings.value = item.settings
    isOverviewMode.value = false
    return
  }

  // 否则尝试连接
  const previousId = currentBackendId.value
  const previousSettings = JSON.parse(JSON.stringify(connectionSettings.value))
  connectionSettings.value = item.settings
  try {
    // 明确指示是否需要跳转
    if (stopJump) isOverviewMode.value = true
    else isOverviewMode.value = false

    await connect(item.settings)
    await checkAndSaveIpChange()
    await handleAutoStartService()
    // eslint-disable-next-line no-unused-vars
  } catch (e) {
    setTimeout(() => {
      if (previousSettings) connectionSettings.value = previousSettings
      if (previousId && allSessionStatuses.value[previousId] === 'ready') switchView(previousId)
      else resetView()

      const failedClient = findClientBySettings(item.settings)
      if (failedClient) {
        const k = Object.keys(robotStore.clients).find(
          (key) => robotStore.clients[key] === failedClient
        )
        if (k) robotStore.setClientStatus(k, 'disconnected')
      }
    }, 2000)
  }
}

function deleteBackend(index) {
  const item = savedBackends.value[index]
  if (isBackendRunning(item)) return
  ElMessageBox.confirm('确定删除此配置?', '警告', { type: 'warning' }).then(async () => {
    const isCurrent = JSON.stringify(item.settings) === JSON.stringify(connectionSettings.value)
    savedBackends.value.splice(index, 1)
    await saveBackendsToStore()
    if (isCurrent) {
      connectionSettings.value =
        savedBackends.value[0]?.settings ||
        (isWindows.value ? { mode: 'remote', ip: '', name: '' } : { ...DEFAULT_SETTINGS })

      // 如果删除了当前项，切回总览
      resetView()
      isOverviewMode.value = true
    }
    ElMessage.success('已删除')
  })
}

async function handleManualDisconnect(item) {
  try {
    await ElMessageBox.confirm(`确定断开 ${getBackendDisplayName(item)}?`, '确认', {
      type: 'warning'
    })
    const client = findClientBySettings(item.settings)
    if (client) {
      if (appSettings.value.autoStopRoscore)
        await client.api.post('/ros/action', { service: 'stack', action: 'stop' }).catch(() => {})
      else if (appSettings.value.autoStopBridge) {
        await client.api
          .post('/ros/action', { service: 'foxglove', action: 'stop' })
          .catch(() => {})
        await client.api
          .post('/ros/action', { service: 'compressor', action: 'stop' })
          .catch(() => {})
      }

      const k = Object.keys(robotStore.clients).find((key) => robotStore.clients[key] === client)
      if (k) {
        await disconnectBackend(k)
        ElMessage.info('已断开')
      }
    }
  } catch {
    /* cancel */
  }
}

async function handleConfigButtonClick() {
  if (isOverviewMode.value) return
  // 严格防抖检查
  if (isActionInProgress.value) return

  if (currentBackendStatus.value === 'ready') {
    handleManualDisconnect({
      settings: connectionSettings.value,
      name: connectionSettings.value.name
    })
  } else {
    try {
      await connect(connectionSettings.value)
      await checkAndSaveIpChange()
      ElNotification({ title: '连接成功', type: 'success' })
      await handleAutoStartService()
    } catch (e) {
      ElNotification({ title: '连接失败', message: e.message, type: 'error' })
      // 仅当当前 Store 状态确实是 failed/setting_up 时才重置
      // 防止“成功后的一瞬间被之前的失败回调覆盖”
      setTimeout(() => {
        const client = findClientBySettings(connectionSettings.value)
        // 只有当状态不是 ready 时才强制设为 disconnected
        // 这样如果其实已经并发连接成功了，就不会把绿灯变成灭灯
        if (client && client.status !== 'ready') {
          const clientKey = Object.keys(robotStore.clients).find(
            (k) => robotStore.clients[k] === client
          )
          if (clientKey) {
            robotStore.setClientStatus(clientKey, 'disconnected')
          }
        }
      }, 3000)
    }
  }
}

async function toggleServiceConnection() {
  if (isConnected.value) {
    try {
      const client = robotStore.clients[currentBackendId.value]
      const runningCount =
        client?.nodes?.filter((n) => n.status === 'running' || n.status === 'starting').length || 0
      if (runningCount > 0) {
        await ElMessageBox.confirm(`检测到 ${runningCount} 个活跃节点，确定强制停止?`, '警告', {
          type: 'error'
        })
      }
      await stopServices()
    } catch {
      /* cancel */
    }
  } else if (isCurrentBackendReady.value) {
    await connectServices()
  }
}

async function handleAutoStartService() {
  if (appSettings.value.autoStartRos) await connectServices()
}

const handleAppReset = async () => {
  await window.api.setConfig('saved_backends', [])
  await window.api.setConfig('last_connection_settings', null)
  await window.api.setConfig('app_settings', null)
  location.reload()
}

const saveBackendsToStore = async () =>
  window.api.setConfig('saved_backends', toRaw(savedBackends.value))
// #endregion

// #region 5. Lifecycle
onMounted(async () => {
  // [核心] 启动 Store 的 IPC 监听，确保编辑器可以请求 Token
  robotStore.setupSyncListeners()
  window.api.onWindowFocusChanged((v) => (isWindowFocused.value = v))

  const hostInfo = await window.api.getHostInfo()
  // process.platform 返回 'win32' 代表 Windows
  isWindows.value = hostInfo.platform === 'win32'

  // Load Settings
  const sApp = await window.api.getConfig('app_settings')
  if (sApp) appSettings.value = { ...defaultAppSettings, ...sApp }

  // Load Backends
  const sBackends = await window.api.getConfig('saved_backends')
  let valid = Array.isArray(sBackends)
    ? sBackends.filter((i) => i.settings?.mode === 'local' || i.settings?.ip)
    : []

  if (valid.length > 0) savedBackends.value = valid
  else savedBackends.value = []
  saveBackendsToStore()

  // Restore last connection settings
  const sLastConn = await window.api.getConfig('last_connection_settings')
  if (sLastConn) {
    // 仅仅是把 IP/Port 填回到 footer 的输入框里，方便用户直接点连接
    connectionSettings.value = sLastConn
  }
  // 6. [核心修改] 强制进入总览模式
  isOverviewMode.value = true
  resetView()
})

// Watchers
watch(connectionSettings, (val) => window.api.setConfig('last_connection_settings', toRaw(val)), {
  deep: true
})
watch(
  () => robotStore.activeClient?.ipChanged,
  async (val) => {
    if (val) await checkAndSaveIpChange()
  }
)

// Status Props Helper
const getStatusProps = (status) => {
  const map = {
    connected: { type: 'success', text: '已连接' },
    disconnected: { type: 'info', text: '未连接' },
    error: { type: 'danger', text: '错误' }
  }
  return map[status] || { type: 'info', text: '未知' }
}
const roscoreProps = computed(() => getStatusProps(roscoreStatus.value))
const rosbridgeProps = computed(() => getStatusProps(rosbridgeStatus.value))

watch([roscoreStatus, rosbridgeStatus], ([nR, nB], [oR, oB]) => {
  if ((oR === 'connected' || oB === 'connected') && nR !== 'connected' && nB !== 'connected') {
    ElNotification({
      title: 'System Stopped',
      message: 'ROS 服务已停止',
      type: 'info',
      duration: 2000
    })
  }
})
// #endregion
</script>

<style scoped>
/* ============================================
   1. 布局变量
   ============================================ */
.app-container {
  --header-height: 48px;
  --footer-height: 58px;
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  background-color: var(--bg-color);
  overflow: hidden;
  font-family: 'Segoe UI', sans-serif;
}

/* 拖拽区域 */
.drag-region {
  -webkit-app-region: drag;
}

.no-drag {
  -webkit-app-region: no-drag;
}

/* ============================================
   2. Header (Glass)
   ============================================ */
.app-glass-header {
  height: var(--header-height);
  background: var(--glass-bg);
  backdrop-filter: blur(12px);
  border-bottom: 1px solid var(--glass-border);
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0 0 0 20px;
  z-index: 100;
}

.app-branding {
  display: flex;
  align-items: center;
  gap: 10px;
}

.app-name {
  font-weight: 800;
  font-size: 24px;
  /* 增大字号 */
  letter-spacing: 1px;
  background: linear-gradient(90deg, #409eff, #337ecc);
  background-clip: text;
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  padding-left: 6px;
}

.header-right {
  display: flex;
  align-items: center;
  height: 100%;
}

/* 功能按钮组 */
.function-buttons {
  display: flex;
  gap: 8px;
  padding-right: 10px;
}

.function-buttons .el-button {
  border: none;
  background: transparent;
  color: var(--text-secondary);
  padding: 8px 12px;
}

.function-buttons .el-button:hover {
  background: rgba(0, 0, 0, 0.05);
  color: var(--text-primary);
}

:global(html.dark) .function-buttons .el-button:hover {
  background: rgba(255, 255, 255, 0.1);
}

/* 自定义窗口控制按钮 */
/* [需求3] Windows 风格窗口控制 */
.window-controls.windows-style {
  display: flex;
  height: 100%;
  align-items: flex-start;
  -webkit-app-region: no-drag;
}

.win-ctrl-btn {
  width: 48px;
  /* Windows 标准宽度 */
  height: 100%;
  /* 填满 Header */
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-primary);
  transition: all 0.2s;
  cursor: pointer;
  font-size: 14px;
  border-radius: 0 !important;
  /* [重点] 强制直角 */
}

.win-ctrl-btn:hover {
  background-color: rgba(0, 0, 0, 0.05);
}

.win-ctrl-btn.close:hover {
  background-color: #e81123;
  color: white;
}

:global(html.dark) .win-ctrl-btn:hover {
  background-color: rgba(255, 255, 255, 0.1);
}

:global(html.dark) .win-ctrl-btn.close:hover {
  background-color: #e81123;
}

/* ============================================
   3. Main Content
   ============================================ */
.app-main-content {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.view-wrapper {
  height: 100%;
  width: 100%;
  /* 确保 Dashboard 内部也能撑满 */
  display: flex;
  flex-direction: column;
}

/* ============================================
   4. Footer (Glass & Navigation)
   ============================================ */
.app-glass-footer {
  height: var(--footer-height);
  display: flex;
  align-items: center;
  gap: 20px;
  /* 区块间距 */
}

/* 左侧区域 */
.footer-section.left {
  display: flex;
  margin-left: 6px;
  align-items: center;
  gap: 4px;
  width: 100%;
  /* 填满 grid cell */
}

.current-conn-btn {
  width: auto;
  /* 防止溢出 */
  height: 32px;
  font-size: 18px;
  justify-content: flex-start;
  border: 1px solid transparent;
  transition: all 0.3s;
}

.current-conn-btn:hover {
  transform: scale(1.02);
  background: transparent;
}

.connect-switch-btn {
  width: 118px;
  height: 32px;
  font-size: 14px;
  font-weight: 600;
  border-radius: 16px;
  padding: 0;
}

/* 连接按钮内的文字截断 */
.conn-text {
  flex: 1;
  text-align: left;
  overflow: hidden;
  text-overflow: ellipsis;
  margin: 0 0px;
  font-size: 18px;
  font-weight: 600;
}

/* 中间：胶囊导航 (Nav Capsule) */
.nav-capsule {
  width: auto;
  display: flex;
  background: rgba(128, 128, 128, 0.08);
  padding: 4px;
  border-radius: 24px;
  border: 1px solid var(--glass-border);
  gap: 2px;
}

.nav-item {
  display: flex;
  width: 56px;
  align-items: center;
  gap: 6px;
  padding: 4px 12px;
  border-radius: 20px;
  cursor: pointer;
  color: var(--text-secondary);
  font-size: 16px;
  font-weight: 600;
  transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
}

.nav-item:hover {
  background: rgba(255, 255, 255, 0.5);
  color: var(--text-primary);
}

.nav-item.is-active {
  background: white;
  color: #409eff;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
}

:global(html.dark) .nav-item.is-active {
  background: #333;
  color: #409eff;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.4);
}

/* 右侧区域 */
.footer-section.right {
  display: flex;
  margin-right: 20px;
  justify-content: flex-end;
  /* 靠右对齐 */
  gap: 10px;
  width: 100%;
}

.status-block {
  width: 108px;
  height: 32px;
  /* [需求3] 高度减小，与按钮一致 */
  padding: 0 10px;
  border-radius: 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: rgba(128, 128, 128, 0.08);
}

.block-label {
  font-size: 14px;
  font-weight: 700;
  color: var(--text-secondary);
  width: 60px;
}

.block-value {
  font-family: 'consola', monospace;
  /* 你的等宽字体 */
  font-size: 13px;
  font-weight: 600;
  color: var(--text-primary);
  margin-right: auto;
  /* 靠左推 */
  white-space: nowrap;
}

.block-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #dcdfe6;
  flex-shrink: 0;
  margin-left: 8px;
}

/* --- 状态颜色映射 (对应 props.type) --- */
/* Success (Green) */
.status-block.success {
  background: rgba(103, 194, 58, 0.1);
  border-color: rgba(103, 194, 58, 0.2);
  color: #67c23a;
}

.status-block.success .block-value {
  color: #67c23a;
}

.status-block.success .block-dot {
  background: #67c23a;
  box-shadow: 0 0 6px rgba(103, 194, 58, 0.4);
}

/* Danger / Error (Red) */
.status-block.danger,
.status-block.error {
  background: rgba(245, 108, 108, 0.1);
  border-color: rgba(245, 108, 108, 0.2);
}

.status-block.danger .block-value {
  color: #f56c6c;
}

.status-block.danger .block-dot {
  background: #f56c6c;
  box-shadow: 0 0 6px rgba(245, 108, 108, 0.4);
}

/* Warning (Orange) */
.status-block.warning {
  background: rgba(230, 162, 60, 0.1);
  border-color: rgba(230, 162, 60, 0.2);
}

.status-block.warning .block-value {
  color: #e6a23c;
}

.status-block.warning .block-dot {
  background: #e6a23c;
  box-shadow: 0 0 6px rgba(230, 162, 60, 0.4);
}

/* Info / Disconnected (Gray) */
.status-block.info {
  background: rgba(144, 147, 153, 0.1);
  border-color: rgba(144, 147, 153, 0.2);
}

.status-block.info .block-dot {
  background: #909399;
}

/* ============================================
   5. Popover 内部样式 (Fixed Layout & Animation)
   ============================================ */
/* 列表项文字截断修复 */
.backend-info {
  flex: 1;
  min-width: 0;
  /* 关键：允许 flex 子项收缩 */
  margin: 0 8px;
}

.backend-name {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  font-weight: 600;
  font-size: 14px;
}

/* 新建连接 - 横向滑动动画修复 */
.add-buttons-row {
  display: flex;
  align-items: center;
  overflow: hidden;
  margin-top: 8px;
  /* 隐藏滑出的部分 */
}

.add-btn-base {
  flex: 1;
  /* 默认占满 */
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 10px;
  border-radius: 8px;
  border: 1px dashed #dcdfe6;
  color: var(--text-secondary);
  cursor: pointer;
  transition: all 0.3s;
  white-space: nowrap;
}

.add-btn-base:hover {
  border-color: #409eff;
  color: #409eff;
  background: rgba(64, 158, 255, 0.05);
}

/* 滑动容器 */
.slide-wrapper {
  width: 0;
  /* 初始宽度 0 */
  opacity: 0;
  overflow: hidden;
  transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
}

.slide-wrapper.is-open {
  width: 140px;
  /* 展开后的宽度 */
  opacity: 1;
  margin-left: 8px;
}

.add-btn-slide {
  flex: 1;
  display: flex;
  height: 100%;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 10px;
  border-radius: 8px;
  border: 1px dashed #67c23a;
  color: #67c23a;
  background: rgba(103, 194, 58, 0.05);
  cursor: pointer;
  white-space: nowrap;
}

.add-btn-slide:hover {
  background: rgba(103, 194, 58, 0.1);
  transform: scale(1.02);
}
</style>

<style>
/* ============================================
   5. 全局组件样式 (Popovers & Dialogs)
   ============================================ */

/* 覆盖 Popover 默认样式，使其变成玻璃卡片风格 */
.el-popover.backend-list-popover {
  /* 适配深色模式的 CSS 变量需确保在此作用域生效，或者使用硬编码 fallback */
  --popover-bg: rgba(255, 255, 255, 0.9);
  --popover-border: #e4e7ed;
  --item-hover: #f5f7fa;
  --text-main: #303133;
  --text-sub: #909399;
}

html.dark .el-popover.backend-list-popover {
  --popover-bg: rgba(30, 30, 30, 0.95);
  --popover-border: #4c4d4f;
  --item-hover: rgba(255, 255, 255, 0.05);
  --text-main: #e5eaf3;
  --text-sub: #a3a6ad;
}

.el-popover.backend-list-popover {
  padding: 12px !important;
  border-radius: 12px !important;
  border: 1px solid var(--popover-border) !important;
  background: var(--popover-bg) !important;
  backdrop-filter: blur(12px);
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.15) !important;
}

/* 列表容器 */
.backend-list-container {
  display: flex;
  flex-direction: column;
}

.el-divider.backend-list-header {
  font-size: 8px;
  font-weight: 400;
  color: var(--text-sub);
  margin-top: 12px;
  margin-bottom: 18px;
  margin-left: 3px;
  width: 98.5%;
}

.el-divider.backend-list-header .el-divider__text {
  /* 1. 修改字体大小和颜色 */
  font-size: 12px;
  font-weight: 400;
  color: #909399;
  /* var(--text-secondary) */
  letter-spacing: 0.5px;
  background-color: var(--popover-bg);
}

/* 深色模式适配 */
html.dark .el-divider.backend-list-header {
  border-color: rgba(255, 255, 255, 0.1);
}

html.dark .el-divider.backend-list-header .el-divider__text {
  color: #606266;
  background-color: var(--popover-bg);
  /* 深色背景 */
}

/* 列表项 (仿 Dashboard 风格) */
.backend-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 10px 12px;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  margin-bottom: 4px;
  border: 1px solid transparent;
}

.backend-item:hover {
  background-color: var(--item-hover);
}

.backend-item.is-active {
  background-color: rgba(64, 158, 255, 0.1);
  /* 浅蓝底 */
  border-color: rgba(64, 158, 255, 0.2);
}

/* 文本信息 */
.backend-name {
  font-size: 14px;
  font-weight: 500;
  color: var(--text-main);
}

.backend-detail {
  font-size: 12px;
  color: var(--text-sub);
}

/* 状态点 */
.status-dot-container {
  margin-left: 3px;
  margin-right: 9px;
}
.menu-dot-container {
  margin-right: 6px;
}

.list-status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: var(--text-sub);
  transition: all 0.3s;
}

.list-status-dot.ready {
  background-color: #67c23a;
  box-shadow: 0 0 6px rgba(103, 194, 58, 0.5);
}

.list-status-dot.setting_up {
  background-color: #e6a23c;
  animation: blink 1s infinite;
}

.list-status-dot.failed {
  background-color: #f56c6c;
}

/* 悬浮操作按钮 */
.backend-actions .el-button.action-icon {
  font-size: 14px;
  padding: 6px;
  margin: 0 2px;
  border-radius: 4px;
}

/* 删除按钮 - 红 */
.backend-actions .el-button.action-icon.del {
  color: #f56c6c !important;
}

.backend-actions .el-button.action-icon.del:hover {
  background: rgba(245, 108, 108, 0.1);
}

/* 停止按钮 - 橙/红 */
.backend-actions .el-button.action-icon.stop {
  color: #e6a23c !important;
}

.backend-actions .el-button.action-icon.stop:hover {
  background: rgba(230, 162, 60, 0.1);
}

/* 编辑按钮 - 蓝 */
.backend-actions .el-button.action-icon.edit {
  color: #409eff !important;
}

.backend-actions .el-button.action-icon.edit:hover {
  background: rgba(64, 158, 255, 0.1);
}

/* 首次连接按钮 - 橙色高亮风格 */
.add-btn-slide.orange-style {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 10px;
  border-radius: 8px;

  /* 默认状态：浅橙色线框 */
  border: 1px dashed #e6a23c;
  color: #e6a23c;
  background: rgba(230, 162, 60, 0.08);

  cursor: pointer;
  white-space: nowrap;
  transition: all 0.2s;
}

/* 悬浮状态：实心亮橙色，白字，无边框，无放大 */
.add-btn-slide.orange-style:hover {
  border-color: transparent;
  background: #ff9900;
  /* 明亮橙色 */
  color: white;
  transform: none;
  box-shadow: 0 4px 12px rgba(255, 153, 0, 0.3);
}
</style>
