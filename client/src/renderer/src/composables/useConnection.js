import { computed, ref } from 'vue'
import { useRobotStore } from '../store/robot'
import { ElMessage } from 'element-plus'

export function useConnection() {
  const store = useRobotStore()

  // --- 状态映射 ---

  // 1. 获取所有会话状态 (供左下角列表显示颜色)
  // 格式: { '192.168.1.10': 'ready', '192.168.1.12': 'failed' }
  const allSessionStatuses = computed(() => {
    const map = {}
    for (const [key, client] of Object.entries(store.clients)) {
      map[key] = client.status
    }
    return map
  })

  // 2. 当前视图 ID (即 IP)
  const currentBackendId = computed(() => store.activeID)

  // 3. 当前视图的大状态
  const currentBackendStatus = computed(() => {
    if (!store.activeClient) return 'disconnected'
    return store.activeClient.status
  })

  // 4. 服务指示灯状态
  const roscoreStatus = computed(() => {
    if (!store.activeClient) return 'disconnected'
    return store.activeClient.serviceStatus.roscore === 'active' ? 'connected' : 'disconnected'
  })

  const rosbridgeStatus = computed(() => {
    if (!store.activeClient) return 'disconnected'
    return store.activeClient.serviceStatus.bridge === 'active' ? 'connected' : 'disconnected'
  })

  const serviceStatus = computed(() => {
    if (roscoreStatus.value === 'connected' && rosbridgeStatus.value === 'connected')
      return 'connected'
    return 'disconnected'
  })

  // --- UI 交互状态 ---
  const pendingAction = ref(null)
  const isActionInProgress = ref(false)

  // --- 核心方法 ---

  // 根据 Settings 生成 ID (新架构下 ID 就是 IP)
  const generateIpFromSettings = (settings) => {
    return settings.ip || 'localhost'
  }

  // 建立连接
  const connect = async (settings) => {
    isActionInProgress.value = true
    pendingAction.value = 'connect'
    try {
      await store.addConnection(settings)
    } catch (e) {
      ElMessage.error(`连接失败: ${e.message}`)
      throw e
    } finally {
      isActionInProgress.value = false
      pendingAction.value = null
    }
  }

  // 断开连接 (从列表中移除)
  const disconnectBackend = async (key) => {
    store.removeConnection(key)
  }

  // 切换视图
  const switchView = (key) => {
    store.activeID = key
  }

  const resetView = () => {
    store.activeID = null
  }

  // 从缓存恢复视图 (用于页面刷新后，暂简单实现)
  const restoreViewFromSettings = (settings) => {
    if (settings && settings.ip) {
      // 尝试在 clients 中找到匹配 IP 或 Hostname 的对象
      const client = Object.values(store.clients).find(
        (c) => c.ip === settings.ip || (settings.hostname && c.hostname === settings.hostname)
      )
      if (client) {
        store.activeID =
          client.id || Object.keys(store.clients).find((key) => store.clients[key] === client)
      }
    }
  }

  // --- 服务控制 (REST API) ---

  // 启动服务 (调用 bridge start)
  const connectServices = async () => {
    if (!store.activeClient || !store.activeClient.api) return

    isActionInProgress.value = true
    pendingAction.value = 'connect'
    try {
      // 调用 Agent 启动所有服务
      await store.activeClient.api.post('/bridge/action', { service: 'all', action: 'start' })
      ElMessage.success('已发送启动指令')

      // 立即刷新一次状态，并在 1秒、2秒后各刷新一次 (因为启动需要时间)
      store.refreshStatus(store.activeIp)
      setTimeout(() => store.refreshStatus(store.activeIp), 1000)
      setTimeout(() => store.refreshStatus(store.activeIp), 2000)
    } catch (e) {
      ElMessage.error('启动服务失败: ' + e.message)
    } finally {
      isActionInProgress.value = false
      pendingAction.value = null
    }
  }

  // 停止服务 (调用 bridge start)
  const stopServices = async () => {
    if (!store.activeClient || !store.activeClient.api) return

    isActionInProgress.value = true
    pendingAction.value = 'disconnect'
    try {
      await store.activeClient.api.post('/bridge/action', { service: 'all', action: 'stop' })
      ElMessage.success('已发送停止指令')

      store.refreshStatus(store.activeIp)
      setTimeout(() => store.refreshStatus(store.activeIp), 1000)
    } catch (e) {
      ElMessage.error('停止服务失败: ' + e.message)
    } finally {
      isActionInProgress.value = false
      pendingAction.value = null
    }
  }

  return {
    allSessionStatuses,
    currentBackendId,
    currentBackendStatus,
    serviceStatus,
    roscoreStatus,
    rosbridgeStatus,
    pendingAction,
    isActionInProgress,
    connect,
    disconnectBackend,
    switchView,
    resetView,
    generateIpFromSettings,
    restoreViewFromSettings,
    connectServices,
    stopServices
  }
}
