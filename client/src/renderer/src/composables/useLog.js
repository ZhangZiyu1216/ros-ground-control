import { computed, toValue } from 'vue'
import { useRobotStore } from '../store/robot'

// 参数改为 accept getter/ref
export function useLog(backendIdSource) {
  const store = useRobotStore()

  // 1. 动态获取当前的 ID (处理 Ref 或 Getter)
  const backendId = computed(() => toValue(backendIdSource))

  // 2. 获取当前 Client (增加判空保护)
  const client = computed(() => {
    const id = backendId.value
    if (!id) return null
    return store.clients[id]
  })

  // 3. 安全获取 Keys
  const nodeLogKeys = computed(() => {
    if (!client.value || !client.value.nodeLogs) return []
    return Object.keys(client.value.nodeLogs)
  })

  // 4. 安全获取 Terminals
  const terminals = computed(() => {
    if (!client.value || !client.value.terminals) return []
    return client.value.terminals
  })

  // 5. 获取特定日志 (增加保护)
  const getNodeLog = (nodeId) => {
    if (!client.value || !client.value.nodeLogs || !client.value.nodeLogs[nodeId]) {
      return { status: 'stopped', lines: [] }
    }
    return client.value.nodeLogs[nodeId]
  }

  // 动作函数需要获取当前的 ID 值
  const createTerminal = () => {
    const id = backendId.value
    if (id) {
      // console.log('[useLog] Creating terminal for backend:', id)
      return store.addTerminal(id)
    }
    console.warn('[useLog] No backend ID available for createTerminal')
  }

  const closeTerminal = (terminalId) => {
    const id = backendId.value
    if (id) store.removeTerminal(id, terminalId)
  }

  const closeNodeLog = (nodeId) => {
    const id = backendId.value
    if (id) store.removeNodeLog(id, nodeId)
  }

  const isNodeRunning = (nodeId) => {
    const log = getNodeLog(nodeId)
    return log.status === 'running'
  }

  return {
    nodeLogKeys,
    terminals,
    getNodeLog,
    createTerminal,
    closeTerminal,
    closeNodeLog,
    isNodeRunning
  }
}
