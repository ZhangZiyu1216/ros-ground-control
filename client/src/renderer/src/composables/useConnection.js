import { computed, ref } from 'vue'
import { useRobotStore } from '../store/robot'
import { ElMessage, ElNotification } from 'element-plus'

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

  // 2. 当前视图 ID
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
  const localConnectionLoading = ref(false)
  const isActionInProgress = computed(() => {
    const stackLoading = store.activeClient ? store.activeClient.isStackLoading : false
    return localConnectionLoading.value || stackLoading
  })

  // --- 核心方法 ---

  // 根据 Settings 生成 Ip
  const generateIpFromSettings = (settings) => {
    return settings.ip || 'localhost'
  }

  // 建立连接
  const connect = async (settings) => {
    localConnectionLoading.value = true
    pendingAction.value = 'connect'
    try {
      await store.addConnection(settings)
      if (window.api && window.api.broadcastBackendStatus) {
        // 注意：这里使用 store.activeID，它是连接成功后的 UUID
        window.api.broadcastBackendStatus({
          id: store.activeID,
          status: 'connected'
        })
      }
    } catch (e) {
      ElMessage.error(`连接失败: ${e.message}`)
      throw e
    } finally {
      localConnectionLoading.value = false
      pendingAction.value = null
    }
  }

  // 断开连接 (从列表中移除)
  const disconnectBackend = async (key) => {
    console.log(`[Connection] Disconnecting ${key}...`)
    store.removeConnection(key)

    if (window.api && window.api.broadcastBackendStatus) {
      console.log(`[Connection] Broadcasting disconnect signal for ${key}`)
      window.api.broadcastBackendStatus({ id: key, status: 'disconnected' })
    } else {
      console.error('[Connection] API not found, cannot broadcast')
    }
  }

  // 切换视图
  const switchView = (key) => {
    store.activeID = key
  }

  const resetView = () => {
    store.activeID = null
  }

  // --- 服务控制 (REST API) ---

  // 启动服务 (调用 bridge start)
  const connectServices = async () => {
    if (!store.activeID) return
    // 这里不需要设置 localConnectionLoading，因为 store.isStackLoading 会自动触发 computed 更新
    pendingAction.value = 'connect'
    try {
      await store.controlRosStack(store.activeID, 'start')
      ElNotification({ title: '启动成功', message: 'ROS 服务栈已完全启动', type: 'success' })
    } catch (e) {
      ElMessage.error('启动过程异常: ' + e.message)
    } finally {
      pendingAction.value = null
    }
  }

  const stopServices = async () => {
    if (!store.activeID) return

    pendingAction.value = 'disconnect'
    try {
      await store.controlRosStack(store.activeID, 'stop')
      ElNotification({ title: '停止成功', message: 'ROS 服务已停止', type: 'info' })
    } catch (e) {
      ElMessage.error('停止过程异常: ' + e.message)
    } finally {
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
    connectServices,
    stopServices
  }
}
