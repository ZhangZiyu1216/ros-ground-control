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
            <el-icon><EditPen /></el-icon>
            <span>编辑</span>
          </el-button>
          <el-button
            text
            round
            class="win-func-btn"
            :disabled="isConnected"
            @click="showSettings = true"
          >
            <el-icon><Tools /></el-icon>
            <span>设置</span>
          </el-button>
          <el-button
            text
            round
            class="win-func-btn"
            :disabled="isConnected"
            @click="dialogVisible = true"
          >
            <el-icon><InfoFilled /></el-icon>
            <span>关于</span>
          </el-button>
        </div>

        <!-- Windows 风格窗口控制 -->
        <div class="window-controls windows-style">
          <div class="win-ctrl-btn min" @click="minimizeWindow">
            <el-icon><Minus /></el-icon>
          </div>
          <div class="win-ctrl-btn max" @click="maximizeWindow">
            <el-icon><FullScreen /></el-icon>
          </div>
          <div class="win-ctrl-btn close" @click="closeWindow">
            <el-icon><Close /></el-icon>
          </div>
        </div>
      </div>
    </header>

    <!-- 2. Main: 内容区域 -->
    <main class="app-main-content">
      <div v-show="activeTab === 'dashboard'" class="view-wrapper">
        <Dashboard
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
        <LogManager :current-backend-id="currentBackendId" />
      </div>
    </main>

    <!-- 3. Footer: 底部栏 -->
    <footer class="app-glass-footer">
      <!-- Left: 连接配置 -->
      <div class="footer-section left">
        <!-- 1. 连接列表按钮 (最宽, flex: 2) -->
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
            <el-divider class="backend-list-header" content-position="left">可用连接</el-divider>
            <el-scrollbar max-height="280px">
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
                    {{ item.settings.mode === 'local' ? 'Localhost' : item.settings.ip }}
                  </div>
                </div>
                <div class="backend-actions" @click.stop>
                  <el-button
                    v-if="!isBackendRunning(item) && !isDefaultLocalBackend(item)"
                    link
                    class="action-icon del"
                    @click="deleteBackend(index)"
                    ><el-icon><Delete /></el-icon
                  ></el-button>
                  <el-button
                    v-if="getItemStatus(item) === 'ready'"
                    link
                    class="action-icon stop"
                    @click="handleManualDisconnect(item)"
                    ><el-icon><SwitchButton /></el-icon
                  ></el-button>
                  <el-button
                    v-if="!isBackendRunning(item) && !isDefaultLocalBackend(item)"
                    link
                    class="action-icon edit"
                    @click="openEditDialog(index, item)"
                    ><el-icon><Edit /></el-icon
                  ></el-button>
                </div>
                <div v-if="isActiveBackend(item)" class="active-check">
                  <el-icon><Check /></el-icon>
                </div>
              </div>
            </el-scrollbar>
            <div
              class="add-buttons-row"
              @mouseenter="handleAddHover(true)"
              @mouseleave="handleAddHover(false)"
            >
              <div class="add-btn-base" @click.stop="openAddDialog">
                <el-icon><Plus /></el-icon><span>新建连接</span>
              </div>
              <div
                class="slide-wrapper"
                :class="{ 'is-open': showFirstTimeSetupButton && isHoveringAdd }"
              >
                <div class="add-btn-slide orange-style" @click.stop="openFirstTimeSetupDialog">
                  <el-icon><MagicStick /></el-icon><span>首次连接</span>
                </div>
              </div>
            </div>
          </div>
        </el-popover>

        <!-- 分割线 -->
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
      </div>

      <!-- Center: 核心导航 -->
      <div class="footer-section center">
        <div class="nav-capsule">
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'dashboard' }"
            @click="activeTab = 'dashboard'"
          >
            <el-icon><Grid /></el-icon><span>节点</span>
          </div>
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'monitor' }"
            @click="activeTab = 'monitor'"
          >
            <el-icon><Monitor /></el-icon><span>监控</span>
          </div>
          <div
            class="nav-item"
            :class="{ 'is-active': activeTab === 'logs' }"
            @click="activeTab = 'logs'"
          >
            <el-icon><DataBoard /></el-icon><span>日志</span>
          </div>
        </div>
      </div>

      <!-- Right: 状态指示器 -->
      <div class="footer-section right">
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
      </div>
    </footer>
  </div>

  <!-- Dialogs (不变) -->
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
  MagicStick,
  Minus,
  FullScreen
} from '@element-plus/icons-vue'

const minimizeWindow = () => window.electronWindow?.minimize()
const maximizeWindow = () => window.electronWindow?.toggleMaximize()
const closeWindow = () => window.electronWindow?.close()

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

<style scoped>
/* ============================================
   1. 布局变量
   ============================================ */
.app-container {
  --bg-color: #f6f8fa;
  --glass-bg: rgba(255, 255, 255, 0.85);
  --glass-border: rgba(255, 255, 255, 0.6);
  --text-primary: #303133;
  --text-secondary: #606266;
  --header-height: 48px;
  --footer-height: 58px; /* 底部稍微高一点放导航 */

  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  background-color: var(--bg-color);
  overflow: hidden;
  font-family: 'Segoe UI', sans-serif;
}

:global(html.dark) .app-container {
  --bg-color: #141414;
  --glass-bg: rgba(30, 30, 30, 0.85);
  --glass-border: rgba(255, 255, 255, 0.1);
  --text-primary: #e5eaf3;
  --text-secondary: #a3a6ad;
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
  font-size: 24px; /* 增大字号 */
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
  width: 48px; /* Windows 标准宽度 */
  height: 100%; /* 填满 Header */
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-primary);
  transition: all 0.2s;
  cursor: pointer;
  font-size: 14px;
  border-radius: 0 !important; /* [重点] 强制直角 */
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
  gap: 20px; /* 区块间距 */
}

/* 左侧区域 */
.footer-section.left {
  display: flex;
  margin-left: 6px;
  align-items: center;
  gap: 4px;
  width: 100%; /* 填满 grid cell */
}
/* [需求3] 左侧百分比宽度 */
.current-conn-btn {
  width: auto; /* 防止溢出 */
  height: 32px;
  justify-content: flex-start;
  border: 1px solid transparent;
  transition: all 0.3s;
}
.current-conn-btn:hover {
  transform: scale(1.02);
  background: transparent;
}
.connect-switch-btn {
  width: 120px;
  height: 32px;
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
  font-weight: 500;
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
  justify-content: flex-end; /* 靠右对齐 */
  gap: 10px;
  width: 100%;
}
.status-block {
  width: 108px;
  height: 32px; /* [需求3] 高度减小，与按钮一致 */
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
  font-family: 'consola', monospace; /* 你的等宽字体 */
  font-size: 13px;
  font-weight: 600;
  color: var(--text-primary);
  margin-right: auto; /* 靠左推 */
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
  min-width: 0; /* 关键：允许 flex 子项收缩 */
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
  overflow: hidden; /* 隐藏滑出的部分 */
}

.add-btn-base {
  flex: 1; /* 默认占满 */
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 10px;
  margin-top: 4px;
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
  width: 0; /* 初始宽度 0 */
  opacity: 0;
  overflow: hidden;
  transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
}
.slide-wrapper.is-open {
  width: 140px; /* 展开后的宽度 */
  opacity: 1;
  margin-left: 8px;
}

.add-btn-slide {
  display: flex;
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
  color: #909399; /* var(--text-secondary) */
  letter-spacing: 0.5px;
  background-color: var(--popover-bg);
}

/* 深色模式适配 */
html.dark .el-divider.backend-list-header {
  border-color: rgba(255, 255, 255, 0.1);
}
html.dark .el-divider.backend-list-header .el-divider__text {
  color: #606266;
  background-color: var(--popover-bg); /* 深色背景 */
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
  transform: translateX(2px);
}
.backend-item.is-active {
  background-color: rgba(64, 158, 255, 0.1); /* 浅蓝底 */
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
  margin-right: 12px;
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
  background: #ff9900; /* 明亮橙色 */
  color: white;
  transform: none;
  box-shadow: 0 4px 12px rgba(255, 153, 0, 0.3);
}
</style>
