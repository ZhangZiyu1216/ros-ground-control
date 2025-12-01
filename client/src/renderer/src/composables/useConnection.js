/* eslint-disable no-unused-vars */
import { computed, ref } from 'vue'
import { useRobotStore } from '../store/robot'
import { ElMessage, ElNotification } from 'element-plus'

export function useConnection() {
  const store = useRobotStore()

  // --- UI 状态 ---
  const localConnectionLoading = ref(false)
  const pendingAction = ref(null) // 'connect' | 'disconnect' | null

  // --- 1. 状态映射 (Computed) ---

  // 获取所有会话状态 (供左下角列表显示颜色)
  // 返回格式: { 'uuid-1': 'ready', '192.168.1.10': 'setting_up' }
  const allSessionStatuses = computed(() => {
    const map = {}
    if (!store.clients) return map
    for (const [key, client] of Object.entries(store.clients)) {
      map[key] = client.status
    }
    return map
  })

  // 当前激活的 Backend ID
  const currentBackendId = computed(() => store.activeID)

  // 当前 Backend 的大状态 (用于 UI 顶栏颜色/文字)
  const currentBackendStatus = computed(() => {
    const client = store.activeClient
    if (!client) return 'disconnected'
    return client.status // 'setting_up' | 'ready' | 'failed' | 'disconnected'
  })

  // 核心服务状态 (ROS Core)
  const roscoreStatus = computed(() => {
    const client = store.activeClient
    if (!client || !client.serviceStatus) return 'disconnected'
    return client.serviceStatus.roscore === 'active' ? 'connected' : 'disconnected'
  })

  // 桥接服务状态 (Foxglove/Rosbridge)
  const rosbridgeStatus = computed(() => {
    const client = store.activeClient
    if (!client || !client.serviceStatus) return 'disconnected'
    return client.serviceStatus.bridge === 'active' ? 'connected' : 'disconnected'
  })

  // 综合服务状态 (两者都 Connected 才算 Connected)
  const serviceStatus = computed(() => {
    if (roscoreStatus.value === 'connected' && rosbridgeStatus.value === 'connected') {
      return 'connected'
    }
    return 'disconnected'
  })

  // 是否有正在进行的操作 (显示 loading 转圈)
  const isActionInProgress = computed(() => {
    // 1. 本地连接正在握手
    if (localConnectionLoading.value) return true

    // 2. 远程 Stack 正在启停
    const client = store.activeClient
    if (client && client.isStackLoading) return true

    // 3. Client 处于初始化握手阶段
    if (client && client.status === 'setting_up') return true

    return false
  })

  // --- 2. 核心交互方法 ---

  // 辅助：生成初始 IP 显示文本
  const generateIpFromSettings = (settings) => {
    return settings.ip || 'localhost'
  }

  /**
   * 建立连接
   * @param {Object} settings - { ip, port, name, hostname, ... }
   */
  const connect = async (settings) => {
    // 防止重复点击
    if (localConnectionLoading.value || isActionInProgress.value) return

    localConnectionLoading.value = true
    pendingAction.value = 'connect'

    try {
      // 1. 初始化占位符 (让 UI 立即响应，显示“连接中”)
      store.initClientPlaceholder(settings)

      // 2. 执行核心连接逻辑 (含 WebSocket 握手、RSA 准备、Key 迁移)
      // 注意：store.activeID 可能会在这个过程中从 IP 变为 UUID
      await store.addConnection(settings)

      // 3. 广播最终状态给主进程 (用于多窗口同步)
      if (window.api && window.api.broadcastBackendStatus) {
        window.api.broadcastBackendStatus({
          id: store.activeID, // 此时必须取 store 中的最新 ID
          status: 'connected'
        })
      }

      // 成功提示交由 Store 内部的 Notification 处理，或者在这里补一个
      // ElNotification({ title: '连接成功', type: 'success' })
    } catch (e) {
      // 错误处理：如果是 409 或 握手失败，Store 可能已经抛出了 Error
      console.error('Connection logic failed:', e)
      // 这里不需要再弹窗，request.js 和 robot.js 内部通常已经弹了
      // 如果需要兜底：
      // ElMessage.error(e.message)
    } finally {
      localConnectionLoading.value = false
      pendingAction.value = null
    }
  }

  /**
   * 断开连接
   * @param {string} key - Client ID or IP
   */
  const disconnectBackend = async (key) => {
    store.removeConnection(key)

    // 通知主进程
    if (window.api && window.api.broadcastBackendStatus) {
      window.api.broadcastBackendStatus({ id: key, status: 'disconnected' })
    }
  }

  /**
   * 切换当前视图 (不涉及网络连接，仅切换 UI 显示)
   */
  const switchView = (key) => {
    if (store.clients[key]) {
      store.activeID = key
    }
  }

  const resetView = () => {
    store.activeID = null
  }

  // --- 3. 远程 ROS 服务控制 ---

  const connectServices = async () => {
    if (!store.activeID) return
    // loading 状态由 isActionInProgress 自动接管 (依赖 store.isStackLoading)

    try {
      await store.controlRosStack(store.activeID, 'start')
      ElNotification({
        title: '服务已启动',
        message: 'ROS 服务运行中',
        type: 'success'
      })
    } catch (e) {
      // 错误已在 Store 处理，此处可忽略或做额外 UI 处理
    }
  }

  const stopServices = async () => {
    if (!store.activeID) return

    try {
      await store.controlRosStack(store.activeID, 'stop')
      ElNotification({
        title: '服务已停止',
        message: 'ROS 服务已关闭',
        type: 'info'
      })
    } catch (e) {
      console.error(e)
    }
  }

  return {
    // States
    allSessionStatuses,
    currentBackendId,
    currentBackendStatus,
    serviceStatus,
    roscoreStatus,
    rosbridgeStatus,
    pendingAction,
    isActionInProgress,

    // Actions
    connect,
    disconnectBackend,
    switchView,
    resetView,
    generateIpFromSettings,
    connectServices,
    stopServices
  }
}
