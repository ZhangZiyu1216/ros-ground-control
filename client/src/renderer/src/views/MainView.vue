<template>
  <el-container class="app-container">
    <el-header class="app-title-header">
      <div class="info-left">
        <img :src="logoURL" alt="App Logo" class="header-logo" />
        <h1>ROS Ground Control</h1>
      </div>
      <div class="button-right">
        <el-divider direction="vertical" :class="{ 'is-inactive': !isWindowFocused }" />
        <el-button
          type="primary"
          circle
          :class="{ 'is-inactive': !isWindowFocused }"
          @click="openEditorWindow"
        >
          <el-icon :size="16"> <EditPen /> </el-icon>
        </el-button>
        <el-divider direction="vertical" :class="{ 'is-inactive': !isWindowFocused }" />
        <el-button
          type="primary"
          circle
          :class="{ 'is-inactive': !isWindowFocused }"
          :disabled="isConnected"
          @click="showSettings = true"
        >
          <el-icon :size="16"> <Tools /> </el-icon>
        </el-button>
        <el-divider direction="vertical" :class="{ 'is-inactive': !isWindowFocused }" />
        <el-button
          type="primary"
          circle
          :class="{ 'is-inactive': !isWindowFocused }"
          :disabled="isConnected"
          @click="dialogVisible = true"
        >
          <el-icon :size="16"> <InfoFilled /> </el-icon>
        </el-button>
        <el-divider direction="vertical" :class="{ 'is-inactive': !isWindowFocused }" />
      </div>
    </el-header>
    <el-main>
      <el-tabs v-model="activeTab" type="border-card" tab-position="left" class="main-tabs">
        <el-tab-pane name="dashboard">
          <template #label>
            <div class="tabs-title">
              <el-icon :size="24"> <Grid /> </el-icon>
              <span class="vertical-tab-label">节点控制</span>
            </div>
          </template>
          <Dashboard
            :current-backend-id="currentBackendId"
            :is-current-backend-ready="isCurrentBackendReady"
          />
        </el-tab-pane>
        <el-tab-pane name="monitor">
          <template #label>
            <div class="tabs-title">
              <el-icon :size="24"> <Monitor /> </el-icon>
              <span class="vertical-tab-label">话题监控</span>
            </div>
          </template>
          <TopicMonitor
            :current-backend-id="currentBackendId"
            :is-current-backend-ready="isCurrentBackendReady"
          />
        </el-tab-pane>
        <el-tab-pane name="logs">
          <template #label>
            <div class="tabs-title">
              <el-icon :size="24"> <DataBoard /> </el-icon>
              <span class="vertical-tab-label">终端日志</span>
            </div>
          </template>
          <LogManager :current-backend-id="currentBackendId" />
        </el-tab-pane>
      </el-tabs>
    </el-main>

    <el-footer class="app-footer">
      <div class="footer-left-controls">
        <div class="connection-config">
          <!-- 使用 Popover 实现向上展开的列表 -->
          <el-popover
            v-model:visible="isPopoverVisible"
            placement="top-start"
            :width="280"
            trigger="hover"
            :show-after="50"
            :hide-after="50"
            popper-class="backend-list-popover"
          >
            <template #reference>
              <div class="popover-trigger-wrapper">
                <el-button
                  class="connection-status-button"
                  :type="backendStatusProps.type"
                  :icon="backendStatusProps.icon"
                  :style="{
                    '--el-button-text-color': backendStatusProps.color,
                    '--el-button-disabled-text-color': backendStatusProps.color
                  }"
                  :loading="
                    currentBackendStatus === 'setting_up' || currentBackendStatus === 'tearing_down'
                  "
                  round
                  text
                  @click="handleConfigButtonClick"
                >
                  <!-- 显示文本逻辑：如果是 Local 且没名字，显示 Local；否则显示名字或 User@Host -->
                  <span class="connection-btn-contain">{{ backendStatusProps.text }}</span>
                </el-button>
              </div>
            </template>

            <!-- 弹出层内容：连接列表 -->
            <div class="backend-list-container">
              <div class="backend-list-header">可用连接</div>
              <el-scrollbar max-height="300px">
                <div
                  v-for="(item, index) in savedBackends"
                  :key="index"
                  class="backend-item"
                  :class="{ 'is-active': isActiveBackend(item), 'is-disabled': isAnyLoading }"
                  @click="switchToBackend(item)"
                >
                  <div class="status-dot-container">
                    <!-- 使用动态 class 绑定颜色 -->
                    <div class="list-status-dot" :class="getItemStatus(item)"></div>
                  </div>
                  <div class="backend-info">
                    <div class="backend-name">{{ getBackendDisplayName(item) }}</div>
                    <div class="backend-detail">
                      {{ item.settings.mode === 'local' ? 'Localhost' : item.settings.ip }}
                    </div>
                  </div>
                  <!-- 悬浮操作按钮 -->
                  <div class="backend-actions" @click.stop>
                    <!-- 如果未运行，显示“删除”按钮 -->
                    <el-button
                      v-if="!isBackendRunning(item) && !isDefaultLocalBackend(item)"
                      circle
                      type="danger"
                      link
                      title="删除连接"
                      @click="deleteBackend(index)"
                    >
                      <el-icon><Delete /></el-icon>
                    </el-button>
                    <!-- 如果已连接，显示“断开”按钮 -->
                    <el-button
                      v-if="getItemStatus(item) === 'ready'"
                      circle
                      type="danger"
                      link
                      title="断开连接"
                      @click="handleManualDisconnect(item)"
                    >
                      <el-icon><SwitchButton /></el-icon>
                    </el-button>
                    <!-- 如果未运行，显示“编辑”按钮 -->
                    <el-button
                      v-if="!isBackendRunning(item) && !isDefaultLocalBackend(item)"
                      circle
                      type="primary"
                      link
                      title="编辑连接"
                      @click="openEditDialog(index, item)"
                    >
                      <el-icon><Edit /></el-icon>
                    </el-button>
                  </div>
                  <!-- 选中标记 -->
                  <div v-if="isActiveBackend(item)" class="active-check">
                    <el-icon><Check /></el-icon>
                  </div>
                </div>
              </el-scrollbar>

              <el-divider style="margin: 8px 0" />

              <!-- 添加新连接按钮 -->
              <div
                class="add-buttons-wrapper"
                @mouseenter="handleAddHover(true)"
                @mouseleave="handleAddHover(false)"
              >
                <transition-group
                  name="btn-slide"
                  tag="div"
                  class="add-buttons-container"
                  :class="{ 'is-hovered': isHoveringAdd }"
                >
                  <!-- 1. 原有的“添加新连接”按钮 -->
                  <!-- key 是 transition-group 必需的 -->
                  <div key="add-new" class="add-backend-btn" @click.stop="openAddDialog">
                    <el-icon><Plus /></el-icon>
                    <span>添加新连接</span>
                  </div>
                  <!-- 2. 新增的“首次部署”按钮 -->
                  <div
                    v-if="showFirstTimeSetupButton"
                    key="first-setup"
                    class="add-backend-btn primary-style"
                    @click.stop="openFirstTimeSetupDialog"
                  >
                    <el-icon><MagicStick /></el-icon>
                    <span>首次连接</span>
                  </div>
                </transition-group>
              </div>
            </div>
          </el-popover>
        </div>

        <div class="connect-toggle-button">
          <el-button
            :type="isConnected ? 'danger' : 'success'"
            :loading="isActionInProgress"
            :disabled="!isCurrentBackendReady"
            round
            plain
            @click="toggleServiceConnection"
          >
            {{ connectionButtonText }}
          </el-button>
        </div>
      </div>

      <div class="status-indicators">
        <div class="status-item">
          <span>Roscore:</span>
          <el-tag :type="roscoreProps.type" effect="dark" size="small" round>
            <span class="dot" :class="roscoreStatus"></span>
            {{ roscoreProps.text }}
          </el-tag>
        </div>
        <div class="status-item">
          <span>Rosbridge:</span>
          <el-tag :type="rosbridgeProps.type" effect="dark" size="small" round>
            <span class="dot" :class="rosbridgeStatus"></span>
            {{ rosbridgeProps.text }}
          </el-tag>
        </div>
      </div>
    </el-footer>
  </el-container>
  <ConnectionDialog
    v-model:visible="dialogVisible"
    :initial-data="dialogInitialData"
    @apply="handleConnectionApply"
  />
  <SettingsDialog
    v-model="showSettings"
    v-model:settings="appSettings"
    @reset-config="handleAppReset"
  />
  <FirstTimeSetupDialog
    v-model="isFirstTimeSetupDialogVisible"
    :is-deploying="isDeploying"
    @deploy="handleDeploy"
  />
</template>

<script setup>
// #region 1. Imports
import { ref, onMounted, computed, watch, toRaw } from 'vue' // 新增 nextTick
import Dashboard from './Dashboard.vue'
import LogManager from './LogManager.vue'
import TopicMonitor from './TopicMonitor.vue'
import ConnectionDialog from '../components/ConnectionDialog.vue'
import SettingsDialog from '../components/SettingsDialog.vue'
import FirstTimeSetupDialog from '../components/FirstTimeSetupDialog.vue'
import { useRobotStore } from '../store/robot.js'
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
  MagicStick
} from '@element-plus/icons-vue'
import logoURL from '../../../../resources/icon.png'

// [新增] 全局应用设置
const showSettings = ref(false)
const defaultAppSettings = {
  autoConnect: true, // 默认自动连接
  autoStartRos: false, // 默认不自动启动服务
  autoStopBridge: false, // 默认不自动停
  autoStopRoscore: false
}
const appSettings = ref({ ...defaultAppSettings })

// [新增] 监听设置变化并持久化
watch(
  appSettings,
  (newVal) => {
    window.api.setConfig('app_settings', toRaw(newVal))
  },
  { deep: true }
)

// [新增] 首次部署相关状态
const showFirstTimeSetupButton = ref(false)
const isFirstTimeSetupDialogVisible = ref(false)
const isDeploying = ref(false) // 用于控制弹窗 loading
const isHoveringAdd = ref(false)
function handleAddHover(val) {
  isHoveringAdd.value = val
  showFirstTimeSetupButton.value = val
}
// [新增] 打开部署弹窗
function openFirstTimeSetupDialog() {
  isPopoverVisible.value = false // 关闭列表弹窗
  isFirstTimeSetupDialogVisible.value = true
}

// 处理部署请求
async function handleDeploy(sshSettings) {
  isDeploying.value = true
  try {
    // 1. 调用主进程进行部署
    const result = await window.api.deployAgent(sshSettings)

    if (result.success) {
      ElNotification({
        title: '部署成功',
        message: 'Agent 已安装并启动，正在尝试连接...',
        type: 'success',
        duration: 3000
      })

      isFirstTimeSetupDialogVisible.value = false

      // 2. 自动打开“添加连接”对话框，并填入信息
      const finalName = result.hostname || sshSettings.host

      dialogMode.value = 'add'
      dialogInitialData.value = {
        mode: 'remote',
        ip: sshSettings.host,
        name: finalName, // UI 显示名称
        hostname: finalName // Store 逻辑需要的 hostname
      }

      setTimeout(() => {
        dialogVisible.value = true
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

// #region 2. State & Storage
const activeTab = ref('dashboard')
const isPopoverVisible = ref(false)
const dialogVisible = ref(false)
const isWindowFocused = ref(true)

// Dialog 状态
const dialogMode = ref('add') // 'add' | 'edit'
const editingBackendIndex = ref(-1)
const dialogInitialData = ref({ mode: 'remote', ip: '', name: '' })

// 核心连接数据
const savedBackends = ref([])
const DEFAULT_SETTINGS = {
  mode: 'local',
  ip: '127.0.0.1',
  name: 'Localhost',
  id: 'local'
}
const connectionSettings = ref({ ...DEFAULT_SETTINGS })

// 引入 Composable
const {
  currentBackendId,
  currentBackendStatus,
  serviceStatus,
  roscoreStatus,
  rosbridgeStatus,
  pendingAction,
  isActionInProgress,
  allSessionStatuses, // 注意：这是一个 ComputedRef
  connect,
  switchView,
  resetView,
  disconnectBackend,
  connectServices,
  stopServices
} = useConnection()
const robotStore = useRobotStore()
// #endregion

// #region 3. Computed Helpers (UI 显示逻辑)

const isCurrentBackendReady = computed(() => currentBackendStatus.value === 'ready')
const isConnected = computed(
  () => isCurrentBackendReady.value && serviceStatus.value === 'connected'
)
const isAnyLoading = computed(() =>
  Object.values(allSessionStatuses.value || {}).some((s) => s === 'setting_up')
)

// 左下角当前连接信息的文本
const connectionModeText = computed(() => {
  const s = connectionSettings.value
  if (!s) return '未配置'
  if (s.mode === 'local') return 'Localhost'
  // 优先显示名称，没有则显示 IP
  return s.name ? `${s.name} (${s.ip})` : s.ip || 'Unknown'
})

// 连接按钮文本
const connectionButtonText = computed(() => {
  if (isActionInProgress.value) {
    // 1. 显式操作 (用户点击了该按钮)
    if (pendingAction.value === 'connect') return '启动中...'
    if (pendingAction.value === 'disconnect') return '关闭中...'
    // 2. 隐式操作 (由节点启动触发，或者 Store 后台 Loading)
    // 根据当前状态反推意图：
    // 如果现在没连接，那就是在启动；如果现在连着，那就是在停止
    return serviceStatus.value === 'connected' ? '关闭中...' : '启动中...'
  }
  // 空闲状态
  return isConnected.value ? '关闭ROS服务' : '启动ROS服务'
})

// 连接按钮样式与图标
const backendStatusProps = computed(() => {
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
    case 'tearing_down':
      return { ...common, type: 'warning', icon: Loading, color: '#E6A23C' }
    default:
      return { ...common, type: 'info', icon: Close, color: '' }
  }
})

// 列表项辅助函数
function getBackendDisplayName(item) {
  if (item.name) return item.name
  if (item.settings.mode === 'local') return 'Localhost'
  return item.settings.ip || 'Unknown'
}

function findClientBySettings(settings) {
  const storeClients = robotStore.clients
  // 遍历所有 client 值
  return Object.values(storeClients).find(
    (c) => c.ip === settings.ip || (settings.hostname && c.hostname === settings.hostname)
  )
}

function getItemStatus(item) {
  // 尝试在 Store 中找到对应的 Client
  const client = findClientBySettings(item.settings)
  return client ? client.status : 'disconnected'
}

function isActiveBackend(item) {
  if (!currentBackendId.value) return false
  // 1. 找到该配置项对应的 Client 对象
  const client = findClientBySettings(item.settings)
  // 2. 如果找到了 Client
  if (client) {
    return client === robotStore.activeClient
  }
  return false
}

function isBackendRunning(item) {
  const status = getItemStatus(item)
  return status === 'ready' || status === 'setting_up'
}

// 判断是否为默认 Localhost 配置（禁止编辑/删除）
function isDefaultLocalBackend(item) {
  if (!item || !item.settings) return false
  const s = item.settings
  return (
    s.mode === 'local' && (s.id === 'local' || s.ip === '127.0.0.1') // 基于 id 或 IP 判断
  )
}
// #endregion

// #region 4. Interaction Handlers (点击事件)

// 辅助：连接成功后，保存 IP 和 ID 到配置文件
async function checkAndSaveIpChange() {
  const currentActiveID = robotStore.activeID
  const client = robotStore.clients[currentActiveID]

  // 只要连接成功 (client存在)，我们就应该尝试保存 UUID，不仅仅是 IP 变了才存
  if (client) {
    // 1. 在保存的列表中找到对应的项
    // 优先用 hostname 匹配，其次用 IP 匹配
    const savedItem = savedBackends.value.find(
      (b) =>
        (client.hostname && b.settings.hostname === client.hostname) || b.settings.ip === client.ip
    )
    if (savedItem) {
      let hasChange = false
      // 更新 IP
      if (client.ipChanged || savedItem.settings.ip !== client.ip) {
        savedItem.settings.ip = client.ip
        hasChange = true
      }
      // [关键修复] 保存 UUID (id)
      // 这样下次启动时，即使不连网也能找到缓存 Key
      if (savedItem.settings.id !== currentActiveID) {
        savedItem.settings.id = currentActiveID
        hasChange = true
      }
      // 同步更新左下角的 connectionSettings
      if (
        connectionSettings.value.hostname === client.hostname ||
        connectionSettings.value.ip === client.ip
      ) {
        connectionSettings.value.ip = client.ip
        connectionSettings.value.id = currentActiveID // [新增]
      }

      if (hasChange) {
        await saveBackendsToStore()
        console.log('Config saved with updated IP/ID')
        // 如果是因为 IP 变了才进来的，提示一下；否则静默保存 ID
        if (client.ipChanged) {
          ElNotification({
            title: '配置已更新',
            message: `已更新机器人 ${client.hostname || client.ip} 的连接信息。`,
            type: 'success'
          })
          client.ipChanged = false
        }
      }
    }
  }
}

function openAddDialog() {
  dialogMode.value = 'add'
  dialogInitialData.value = { mode: 'remote', ip: '', name: '' }
  dialogVisible.value = true
}

function openEditDialog(index, item) {
  if (isDefaultLocalBackend(item)) {
    ElMessage.warning('默认 Localhost 连接不能编辑。')
    return
  }
  if (isBackendRunning(savedBackends.value[index])) {
    ElMessage.warning('无法编辑正在运行的连接，请先断开。')
    return
  }
  dialogMode.value = 'edit'
  editingBackendIndex.value = index
  dialogInitialData.value = JSON.parse(JSON.stringify(item.settings))
  dialogVisible.value = true
}

async function handleConnectionApply(newSettings) {
  // 1. 保存/查重逻辑 (Add/Edit) 保持不变 ...
  if (dialogMode.value === 'add') {
    const exists = savedBackends.value.some(
      (b) => b.settings.ip === newSettings.ip && b.settings.mode === newSettings.mode
    )
    if (exists) {
      ElMessage.warning('列表已存在该 IP 的连接配置。')
      return
    }
    const newItem = {
      name: newSettings.name || newSettings.ip,
      settings: newSettings
    }
    savedBackends.value.push(newItem)
    await saveBackendsToStore()
  } else if (dialogMode.value === 'edit') {
    // 编辑逻辑保持不变...
    const target = savedBackends.value[editingBackendIndex.value]
    target.settings = newSettings
    target.name = newSettings.name || newSettings.ip
    await saveBackendsToStore()
    dialogVisible.value = false
    ElNotification({ title: '配置已更新', type: 'success' })
    return
  }
  // 2. 尝试连接 (仅 Add 模式)
  dialogVisible.value = false
  // 记录回退点
  const previousId = currentBackendId.value
  const previousSettings = JSON.parse(JSON.stringify(connectionSettings.value))
  connectionSettings.value = newSettings
  try {
    await connect(newSettings)
    ElNotification({ title: '连接成功', type: 'success' })
  } catch (error) {
    ElNotification({ title: '连接失败', message: error.message, type: 'error', duration: 3000 })
    // --- 失败延迟回退逻辑 ---
    setTimeout(() => {
      // A. 回退 UI 设置
      connectionSettings.value = previousSettings
      // B. 回退视图
      if (previousId) {
        switchView(previousId)
      } else {
        resetView()
      }
      // C. 移除刚才添加的那个无效配置
      const idx = savedBackends.value.findIndex(
        (b) => b.settings.ip === newSettings.ip && b.settings.mode === newSettings.mode
      )
      if (idx !== -1) {
        savedBackends.value.splice(idx, 1)
        saveBackendsToStore()
      }
      // D. 清理 Store 中的状态 (防止残留)
      robotStore.setClientStatus(newSettings.ip, 'disconnected')
    }, 3000)
  }
}

// 切换连接
async function switchToBackend(item) {
  isPopoverVisible.value = false
  // 1. 查找 Store 中对应的 Client 对象
  const client = findClientBySettings(item.settings)
  const clientKey = client
    ? client.id || Object.keys(robotStore.clients).find((k) => robotStore.clients[k] === client)
    : null

  // 1. 如果已经是当前视角，不操作
  if (currentBackendId.value && clientKey === currentBackendId.value) return

  // 2. 如果目标已经是 Ready 状态，直接切换视角（无网络请求）
  if (client && client.status === 'ready') {
    // 这里的 clientKey 肯定是 UUID (因为 status 是 ready)
    switchView(clientKey)
    connectionSettings.value = item.settings
    return
  }

  // 3. 目标未连接，尝试连接
  // 记录“上一个”可用的连接 ID 和设置，用于回退
  const previousId = currentBackendId.value
  const previousSettings = JSON.parse(JSON.stringify(connectionSettings.value))

  // 更新 UI 到目标设置 (让用户看到正在连接目标)
  connectionSettings.value = item.settings

  try {
    await connect(item.settings)
    await checkAndSaveIpChange()
    await handleAutoStartService()
    // 连接成功：connect 内部会自动 switchView，这里无需操作
    // eslint-disable-next-line no-unused-vars
  } catch (e) {
    // --- 失败处理逻辑 ---
    // 错误通知已在 connect 内部或 store 抛出，这里主要处理 UI 回退
    console.warn('Switch failed, scheduling revert...')

    // 延迟 3 秒回退
    setTimeout(() => {
      // A. 回退 UI 显示的配置文字
      if (previousSettings && previousId) {
        connectionSettings.value = previousSettings
      }

      // B. 回退视图 (如果上一个视图存在且有效)
      if (previousId && allSessionStatuses.value[previousId] === 'ready') {
        switchView(previousId)
        ElMessage.info('已回退到上一个活跃连接')
      } else {
        resetView()
      }
      // C. 将刚才失败的那个连接状态置为“灰色” (Disconnected) 而不是“红色” (Failed)
      // 这样用户体验更好，不会一直看到错误的红灯
      const failedClient = findClientBySettings(item.settings)
      if (failedClient) {
        // 找到它在 Store 里的 Key (可能是临时 IP)
        const failedKey = Object.keys(robotStore.clients).find(
          (k) => robotStore.clients[k] === failedClient
        )
        if (failedKey) robotStore.setClientStatus(failedKey, 'disconnected')
      }
    }, 3000)
  }
}

// 删除连接
function deleteBackend(index) {
  const item = savedBackends.value[index]
  if (isDefaultLocalBackend(item)) {
    ElMessage.warning('默认 Localhost 连接不能删除。')
    return
  }

  if (isBackendRunning(item)) return
  ElMessageBox.confirm('确定删除此配置?', '警告', { type: 'warning' }).then(async () => {
    // 检查是否删除了当前显示的配置 (即使未连接)
    const isCurrent = JSON.stringify(item.settings) === JSON.stringify(connectionSettings.value)

    savedBackends.value.splice(index, 1)
    await saveBackendsToStore()

    if (isCurrent) {
      // 重置为 Local 或列表第一个
      const fallback = savedBackends.value[0] || {
        name: 'Localhost',
        settings: { mode: 'local', ip: '127.0.0.1', name: 'Localhost' }
      }
      connectionSettings.value = fallback.settings
      resetView()
    }
    ElMessage.success('已删除')
  })
}

// 手动断开
async function handleManualDisconnect(item) {
  try {
    await ElMessageBox.confirm(`确定断开 ${getBackendDisplayName(item)}?`, '确认', {
      type: 'warning'
    })

    // 【修复】先找到 Client 对象，再获取其 Store Key
    const client = findClientBySettings(item.settings)
    if (client) {
      const { autoStopBridge, autoStopRoscore } = appSettings.value
      if (autoStopBridge || autoStopRoscore) {
        try {
          // 逻辑简化：
          // 1. 如果配置了停止 Roscore，实际上就是停止全栈 (Stack)
          if (autoStopRoscore) {
            await client.api.post('/ros/action', { service: 'stack', action: 'stop' })
            ElMessage.info('已自动停止 ROS 服务栈 (Roscore + Bridge)')
          }
          // 2. 否则，如果只配置了停止 Bridge
          else if (autoStopBridge) {
            // 停止 Foxglove 和 压缩服务
            await client.api.post('/ros/action', { service: 'foxglove', action: 'stop' })
            await client.api.post('/ros/action', { service: 'compressor', action: 'stop' })
            ElMessage.info('已自动停止 Bridge')
          }
        } catch (err) {
          console.warn('Auto-stop services failed:', err)
          // 不阻断断开流程
        }
      }

      // 获取 Key (client.id 可能为空如果还在连接中，所以通过 activeClient 逻辑找 Key)
      // 在 Store 中，client 对象所在的 Key 就是我们需要传给 removeConnection 的参数
      const clientKey = Object.keys(robotStore.clients).find(
        (k) => robotStore.clients[k] === client
      )

      if (clientKey) {
        await disconnectBackend(clientKey)
        ElMessage.info('已断开')
      }
    }
    // eslint-disable-next-line no-unused-vars
  } catch (e) {
    /* cancel */
  }
}

// 左下角主按钮点击逻辑
async function handleConfigButtonClick() {
  const currentSettingsIp = connectionSettings.value.ip

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
      ElNotification({ title: '连接失败', message: e.message, type: 'error', duration: 3000 })

      setTimeout(() => {
        // 【修复】找到对应的 Key 才能设置状态
        const client = findClientBySettings(connectionSettings.value)
        if (client && connectionSettings.value.ip === currentSettingsIp) {
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

// ROS 服务启停逻辑
async function toggleServiceConnection() {
  if (isConnected.value) {
    try {
      // [新增] 检查是否有正在运行的节点
      const client = robotStore.clients[currentBackendId.value]
      const runningCount =
        client?.nodes?.filter((n) => n.status === 'running' || n.status === 'starting').length || 0

      let confirmMsg = '确定停止所有 ROS 服务?'
      let confirmType = 'warning'

      if (runningCount > 0) {
        confirmMsg = `检测到 ${runningCount} 个正在运行的节点。停止 ROS 服务将强制终止这些节点。确定要继续吗？`
        confirmType = 'error' // 使用红色图标示警
      }

      await ElMessageBox.confirm(confirmMsg, '警告', {
        confirmButtonText: '强制停止',
        cancelButtonText: '取消',
        type: confirmType
      })

      await stopServices()
    } catch {
      /* cancel */
    }
  } else if (isCurrentBackendReady.value) {
    await connectServices()
  }
}

// [新增] 自动启动服务处理
async function handleAutoStartService() {
  if (appSettings.value.autoStartRos) {
    console.log('[AutoStart] Triggering ROS stack start...')
    // 复用 useConnection 里的 connectServices
    await connectServices()
  }
}

const handleAppReset = async () => {
  // 1. 清除配置
  await window.api.setConfig('saved_backends', [])
  await window.api.setConfig('last_connection_settings', null)
  await window.api.setConfig('app_settings', null)
  // 还需要清除所有节点的缓存
  // 简单做法：重新加载窗口
  location.reload()
}

// #endregion

// #region 5. Lifecycle & Utils

// 保存配置到 electron-store
const saveBackendsToStore = async () =>
  window.api.setConfig('saved_backends', toRaw(savedBackends.value))

onMounted(async () => {
  window.api.onWindowFocusChanged((v) => (isWindowFocused.value = v))

  // 加载应用设置
  const savedAppSettings = await window.api.getConfig('app_settings')
  if (savedAppSettings) {
    appSettings.value = { ...defaultAppSettings, ...savedAppSettings }
  }

  // 1. 读取配置
  const saved = await window.api.getConfig('saved_backends')
  let validConfigs = []

  if (Array.isArray(saved)) {
    validConfigs = saved.filter(
      (item) => item.settings && (item.settings.mode === 'local' || item.settings.ip)
    )
  }

  if (validConfigs.length > 0) {
    savedBackends.value = validConfigs
  } else {
    // 默认初始化
    const defaultLocal = {
      name: 'Localhost',
      settings: { mode: 'local', ip: '127.0.0.1', name: 'Localhost', id: 'local' } // 默认给个 id
    }
    savedBackends.value = [defaultLocal]
    await saveBackendsToStore()
  }

  if (window.api && window.api.broadcastBackendStatus) {
    savedBackends.value.forEach((item) => {
      // 确保有 ID 才能广播
      const id = item.settings.id
      if (id) {
        window.api.broadcastBackendStatus({ id: id, status: 'disconnected' })
      }
    })
  }

  // 2. 恢复 ConnectionSettings
  if (connectionSettings.value.ssh) {
    connectionSettings.value = savedBackends.value[0].settings
  }

  // 3. 离线状态“注水” (Hydrate Store)
  // 如果当前选中的配置里有 UUID (id)，我们直接告诉 Store：“虽然没连网，但现在选中的是这个 ID”
  // 这样 Dashboard 就能拿到 ID 去加载缓存了
  const currentSettings = connectionSettings.value

  // 尝试从 savedBackends 找最全的信息 (因为 useStorage 可能存的是旧的)
  const matchedSaved = savedBackends.value.find(
    (b) =>
      (currentSettings.hostname && b.settings.hostname === currentSettings.hostname) ||
      b.settings.ip === currentSettings.ip
  )

  if (matchedSaved && matchedSaved.settings.id) {
    const uuid = matchedSaved.settings.id

    // 在 Store 中初始化一个“离线”Client
    if (!robotStore.clients[uuid]) {
      robotStore.clients[uuid] = {
        id: uuid,
        ip: matchedSaved.settings.ip,
        hostname: matchedSaved.settings.hostname,
        name: matchedSaved.name,
        status: 'disconnected', // 标记为离线
        nodes: [], // 稍后 Dashboard 会填充它
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' }
      }
    }
    // 设置活跃 ID -> 触发 Dashboard 的 watch -> 加载缓存
    robotStore.activeID = uuid
  }

  // 4. 自动连接
  const shouldAutoConnect = appSettings.value.autoConnect && connectionSettings.value.mode // 只要有有效配置就尝试

  if (shouldAutoConnect) {
    connect(connectionSettings.value)
      .then(() => {
        handleAutoStartService()
      })
      .catch((e) => console.log('Auto-connect skipped/failed:', e))
  }
})

// 监听 connectionSettings 变化并持久化到 electron-store
watch(
  connectionSettings,
  (newVal) => {
    // 使用 'last_connection_settings' 作为 key
    window.api.setConfig('last_connection_settings', toRaw(newVal))
  },
  { deep: true }
)
// 监听 Store 中当前活跃 Client 的 ipChanged 标记
watch(
  () => {
    if (!robotStore.activeClient) return false
    return robotStore.activeClient.ipChanged
  },
  async (isChanged) => {
    if (isChanged) {
      await checkAndSaveIpChange()
    }
  }
)

// 状态指示器辅助
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

// [新增] 打开编辑器函数
function openEditorWindow() {
  // 传空对象，表示只打开窗口不指定文件
  window.api.openFileEditor({})
}

// 监听服务状态变更通知
watch([roscoreStatus, rosbridgeStatus], ([newRscore, newRbridge], [oldRscore, oldRbridge]) => {
  const wasDisconnected = oldRscore !== 'connected' && oldRbridge !== 'connected'
  const isDisconnected = newRscore !== 'connected' && newRbridge !== 'connected'

  if (!wasDisconnected && isDisconnected) {
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

<style>
@font-face {
  font-family: 'consola'; /* <-- 给你的字体起一个在CSS中使用的名字 */
  src: url('/fonts/consola.ttf') format('ttf'); /* <-- 引用 public 目录下的字体文件 */
  font-weight: normal; /* 定义这个字体文件对应的字重 */
  font-style: normal; /* 定义这个字体文件对应的样式 */
}

/* 2. 应用根容器：使用 Flexbox 垂直布局 */
.app-container {
  height: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}
.app-container .el-main {
  height: auto;
  width: auto;
  padding: 0;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}
.el-header.app-title-header {
  height: 50px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  background-color: #2c3e50; /* 一个深色的头部背景 */
  color: #ffffff; /* 白色文字 */
  -webkit-app-region: drag;
  padding-left: 20px;
  padding-right: 110px;
}
.app-title-header .info-left {
  display: flex;
  align-items: center; /* 垂直居中 Logo 和标题 */
  gap: 20px; /* 在 Logo 和标题之间创建一些间距 */
  flex-direction: row;
  justify-content: center;
}
.app-title-header .header-logo {
  height: 1.2em; /* 控制 Logo 的高度，宽度会自动缩放 */
  width: auto;
  object-fit: contain; /* 确保图片在保持比例的同时完整显示 */
}
.app-title-header .button-right {
  display: flex;
  flex-direction: row;
  align-items: center;
  gap: 0px;
}
.app-title-header .button-right .el-divider.is-inactive {
  color: #ffffff;
}
.app-title-header .button-right .el-divider.is-inactive {
  color: #999999;
}
.app-title-header .button-right .el-button {
  background-color: #2c3e50;
  border: 0px solid #2c3e50;
  margin-left: -5px;
  margin-right: -5px;
  padding: 0px;
  transition: all 0.3s ease-out;
  -webkit-app-region: no-drag;
}
.app-title-header .button-right .el-button.is-inactive {
  color: #999999;
}
.app-title-header .button-right .el-button:hover {
  transform: scale(1.1);
}
.app-title-header .button-right .el-button:hover,
.app-title-header .button-right .el-button:focus {
  background-color: rgba(255, 255, 255, 0.1);
  border: 0px solid #2c3e50;
}
.app-title-header h1 {
  margin: 0; /* 去掉 h1 标签的默认边距 */
  font-size: 20px;
}

/* 3. Tabs 组件：让它弹性增长以填满剩余空间 */
.main-tabs {
  flex-grow: 1;
  display: flex;
  flex-direction: row;
}
.main-tabs > .el-tabs__content {
  flex-grow: 1;
  padding: 5px;
}
.el-tabs--left.el-tabs--border-card .el-tabs__header.is-left {
  height: 100%;
  display: flex;
  margin: 0;
}
.el-tabs__nav.is-left {
  height: 100%;
  display: flex; /* <-- 关键：将其变为 Flex 容器 */
  flex-direction: column; /* 让 tab 垂直排列 */
}
.el-tabs__item.is-left {
  flex-grow: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  padding: 0 20px;
  height: auto;
}
.tabs-title {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 0.8em;
}
.vertical-tab-label {
  font-size: 20px;
  writing-mode: vertical-lr; /* <-- 关键：开启垂直书写模式 */
  letter-spacing: 0.1em; /* (可选) 增加字间距，让垂直文字更好看 */
  line-height: 1; /* (可选) 调整行高，让文字更紧凑 */
  padding-right: 0.2em;
}

/* 5. 确保每个 Tab Pane 都是块级布局并占满高度 */
.el-tabs--border-card {
  background: transparent;
  border: none;
}

.el-tab-pane {
  height: 100%;
  width: 100%;
}
.el-tabs__content {
  background-color: transparent;
  padding: 20px; /* 增加一些内边距，让卡片不贴边 */
}

/* 6. Footer 样式 */
.el-footer.app-footer {
  height: 50px;
  border-top: 1px solid #e4e7ed;
  background-color: #f5f7fa;
  display: flex;
  align-items: center;
  justify-content: space-between;
}

/* 左侧和中间按钮的容器 */
.footer-left-controls {
  display: flex;
  align-items: center;
  gap: 12px;
}

/* 1. 配置按钮的样式 */
.connection-config .connection-status-button {
  height: 30px;
  margin-left: -8px;
}
.connection-config .connection-status-button .connection-btn-contain {
  font-size: 16px;
}
/* 调整原按钮，支持图标右对齐 */
.connection-status-button {
  padding-right: 8px !important; /* 给箭头留点空间 */
}
/* Popover 内部容器 */
.connection-config .popover-trigger-wrapper {
  display: inline-block; /* 确保 wrapper 包裹住按钮且不独占一行 */
  display: flex;
  transition: transform 0.2s ease-in-out;
}
.connection-config .popover-trigger-wrapper:hover {
  transform: scale(1.02);
}
.connection-config .popover-trigger-wrapper:active {
  transform: scale(0.98);
}
.backend-list-container {
  display: flex;
  flex-direction: column;
}
.backend-list-header {
  font-size: 12px;
  color: #909399;
  margin-bottom: 8px;
  padding-left: 4px;
}
/* 容器 Wrapper */
/* 增加这个 wrapper 是为了精确控制 hover 区域 */
.add-buttons-wrapper {
  padding: 5px 0; /* 给一点垂直内边距 */
  min-height: 40px;
}
/* 按钮容器，现在是 transition-group 的子元素 */
.add-buttons-container {
  display: flex;
  align-items: center;
  gap: 10px;
}
/* 原始“添加”按钮样式 (保持不变) */
.add-backend-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 5px;
  padding: 8px;
  cursor: pointer;
  color: #606266;
  font-size: 13px;
  border-radius: 4px;
  border: 1px dashed #dcdfe6;
  white-space: nowrap;
  box-sizing: border-box; /* 确保 padding 不会影响宽度计算 */
  white-space: nowrap;
  width: 100%;
  transition: all 0.3s ease;
}
.add-backend-btn:hover,
.add-backend-btn.primary-style:hover {
  border-color: #409eff;
  color: #409eff;
  background-color: #ecf5ff;
}
.add-buttons-container.is-hovered .add-backend-btn {
  width: calc(50% - 5px);
}

/* “首次部署”按钮的特殊样式 */
.add-backend-btn.primary-style {
  color: #606266;
  font-size: 13px;
  border-radius: 4px;
  border: 1px dashed #dcdfe6;
  white-space: nowrap;
  box-sizing: border-box; /* 确保 padding 不会影响宽度计算 */
  white-space: nowrap;
}
/* 定义进入和离开时的动画 */
.btn-slide-enter-active {
  transition: all 0.3s ease;
  overflow: hidden;
}
.btn-slide-leave-active {
  right: 0;
  transition: all 0.3s ease;
  overflow: hidden;
}
/* 进入前 和 离开后 的状态 */
.btn-slide-enter-from {
  opacity: 0;
  transform: scale(0.8);
  /* 初始宽度为0，让它平滑地“长”出来 */
  width: 0;
  padding: 0;
  margin-left: 0 !important;
}
.btn-slide-leave-to {
  opacity: 0;
  transform: scale(0.8);
  /* 目标宽度为0，让它平滑地“缩”回去 */
  width: 0;
  padding: 0;
  margin-left: 0 !important;
}
.btn-slide-move {
  transition: transform 0.3s ease;
}

/* 列表项样式 */
.backend-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 8px 12px;
  border-radius: 6px;
  cursor: pointer;
  transition: background-color 0.2s;
  position: relative;
  margin-bottom: 4px;
}
.backend-item:hover {
  background-color: #f5f7fa;
}
.backend-item.is-active {
  background-color: #ecf5ff;
}
.backend-info {
  flex-grow: 1;
  overflow: hidden;
}
.backend-name {
  font-size: 14px;
  font-weight: 500;
  color: #303133;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.backend-detail {
  font-size: 12px;
  color: #909399;
}
.backend-item.is-disabled {
  opacity: 0.6;
  cursor: not-allowed;
}
/* 悬浮按钮区域：默认隐藏，Hover时显示 */
.backend-actions {
  display: none;
  align-items: center;
  gap: 4px;
}
.backend-item:hover .backend-actions {
  display: flex;
}
/* 当显示操作按钮时，隐藏选中标记 */
.backend-item:hover .active-check {
  display: none;
}
.active-check {
  color: #409eff;
  font-weight: bold;
}

.status-dot-container {
  margin-right: 10px;
  display: flex;
  align-items: center;
}

.list-status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: #dcdfe6; /* 默认灰色 (disconnected) */
  transition: background-color 0.3s;
}

.list-status-dot.ready {
  background-color: #67c23a; /* 绿色 */
  box-shadow: 0 0 4px #67c23a;
}

.list-status-dot.setting_up {
  background-color: #e6a23c; /* 黄色 */
  animation: blink 1s infinite;
}

.list-status-dot.failed {
  background-color: #f56c6c; /* 红色 */
}

.connected-tag {
  font-size: 10px;
  color: #67c23a;
  margin-left: 4px;
}

@keyframes blink {
  0% {
    opacity: 1;
  }
  50% {
    opacity: 0.4;
  }
  100% {
    opacity: 1;
  }
}

/* 2. 连接/断开按钮 (Connect/Disconnect) 的样式 */
.connect-toggle-button .el-button {
  transition: all 0.5s ease; /* 添加过渡效果 */
  height: 28px;
  font-size: 14px;
}
/* 默认状态 (Success - Green) */
.connect-toggle-button .el-button--success.is-plain,
.connect-toggle-button .el-button--success.is-plain:focus {
  border-color: #67c23a;
  background-color: #ffffff;
  color: #67c23a;
}
/* 悬停/聚焦状态 (Success - Green): 颜色翻转 */
.connect-toggle-button .el-button--success.is-plain:hover {
  background-color: #67c23a;
  color: #ffffff;
}
/* 默认状态 (Danger - Red) */
.connect-toggle-button .el-button--danger.is-plain,
.connect-toggle-button .el-button--danger.is-plain:focus {
  border-color: #f56c6c;
  background-color: #ffffff;
  color: #f56c6c;
}
/* 悬停/聚焦状态 (Danger - Red): 颜色翻转 */
.connect-toggle-button .el-button--danger.is-plain:hover {
  background-color: #f56c6c;
  color: #ffffff;
}
.status-indicators {
  display: flex;
  align-items: center;
  gap: 20px;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
}

.dot {
  display: inline-block;
  width: 8px;
  height: 8px;
  border-radius: 50%;
  margin-right: 5px;
}

.dot.connected {
  background-color: #ffffff;
}
.dot.disconnected {
  background-color: #ffffff;
}
.dot.error {
  background-color: #ffffff;
}
</style>
