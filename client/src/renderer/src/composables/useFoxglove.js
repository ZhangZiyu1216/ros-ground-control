import { reactive } from 'vue'
import { FoxgloveClient } from '@foxglove/ws-protocol'
import { useRobotStore } from '../store/robot'
import { parse as parseRosMsg } from '@foxglove/rosmsg'
import { MessageReader } from '@foxglove/rosmsg-serialization'

// --- 全局状态池 ---
const connections = new Map() // backendId -> ConnectionState
const reconnectTimers = new Map() // backendId -> TimerID

class ConnectionState {
  constructor() {
    this.client = null
    this.channels = reactive(new Map())
    this.messages = reactive(new Map())
    this.readers = new Map()
    this.subscriptions = new Map()
    this.subIdToChannelId = new Map()
  }
}

export function useFoxglove() {
  const robotStore = useRobotStore()

  const createMessageReader = (channel) => {
    try {
      if (channel.encoding === 'json') {
        return (data) => {
          const text = new TextDecoder().decode(data)
          return JSON.parse(text)
        }
      }

      if (channel.encoding === 'ros1' || channel.schemaName === 'ros1') {
        const definitions = parseRosMsg(channel.schema)
        const reader = new MessageReader(definitions)
        return (data) => reader.readMessage(data)
      }

      return (data) => {
        if (channel.schemaName.includes('Image')) {
          const binary = new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
          let binaryStr = ''
          // 性能优化提示：对于高频大图，建议寻找更高效的转换方式
          for (let i = 0; i < binary.length; i++) binaryStr += String.fromCharCode(binary[i])
          return { data: window.btoa(binaryStr), format: 'raw', _isBinary: true }
        }
        return { error: `Unsupported encoding: ${channel.encoding}` }
      }
    } catch (e) {
      console.warn(`Failed to create reader for ${channel.topic}:`, e)
      return () => ({ error: 'Parse Error' })
    }
  }

  // --- 连接核心 ---
  const ensureConnection = (backendId) => {
    // 1. 基础检查
    const robot = robotStore.clients[backendId]
    if (!robot || robot.status !== 'ready') return

    // 2. 如果已存在连接，跳过
    if (connections.has(backendId)) return

    // 3. 如果正在等待重连，也跳过（防止频繁创建）
    if (reconnectTimers.has(backendId)) return

    const state = new ConnectionState()
    connections.set(backendId, state)

    const ip = robot.ip
    const url = `ws://${ip}:8765`
    console.log(`[Foxglove] Connecting to ${backendId} (${url})...`)

    // 4. 创建客户端
    const client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL])
    })
    state.client = client

    // --- 事件处理 ---

    client.on('open', () => {
      console.log(`[Foxglove] Connected to ${backendId}`)
      // 连接成功，清除重连定时器标记（如果有）
      if (reconnectTimers.has(backendId)) {
        clearTimeout(reconnectTimers.get(backendId))
        reconnectTimers.delete(backendId)
      }
    })

    // 统一的清理/重连逻辑
    const handleDisconnect = () => {
      // 避免重复处理
      if (!connections.has(backendId)) return

      console.log(`[Foxglove] Disconnected/Error on ${backendId}. Cleaning up...`)

      // 1. 清理当前状态
      connections.delete(backendId)

      // 2. 触发重连机制
      // 检查 Store 中该机器人是否还处于“连接”状态。如果是，说明是意外断开，需要重连。
      const currentRobot = robotStore.clients[backendId]
      if (currentRobot && currentRobot.status === 'ready') {
        console.log(`[Foxglove] Robot is still ready. Scheduling reconnect in 3s...`)

        if (reconnectTimers.has(backendId)) clearTimeout(reconnectTimers.get(backendId))

        const timer = setTimeout(() => {
          reconnectTimers.delete(backendId)
          ensureConnection(backendId)
        }, 3000)

        reconnectTimers.set(backendId, timer)
      }
    }

    client.on('error', (e) => {
      console.warn(`[Foxglove] Error on ${backendId}:`, e)
      handleDisconnect()
    })

    client.on('close', handleDisconnect)

    client.on('advertise', (newChannels) => {
      newChannels.forEach((ch) => {
        const reader = createMessageReader(ch)
        state.readers.set(ch.id, reader)
        state.channels.set(ch.id, ch)
      })
    })

    client.on('unadvertise', (channelIds) => {
      channelIds.forEach((id) => {
        state.channels.delete(id)
        state.readers.delete(id)
        state.messages.delete(id)
      })
    })

    client.on('message', ({ subscriptionId, data }) => {
      const channelId = state.subIdToChannelId.get(subscriptionId)
      if (channelId === undefined) return

      const reader = state.readers.get(channelId)
      if (reader) {
        try {
          const decoded = reader(data)
          state.messages.set(channelId, decoded)
          // eslint-disable-next-line no-unused-vars
        } catch (e) {
          // ignore parsing errors
        }
      }
    })
  }

  // --- API ---

  const getTopics = (backendId) => {
    ensureConnection(backendId)
    const state = connections.get(backendId)
    return state ? Array.from(state.channels.values()) : []
  }

  const getMessage = (backendId, channelId) => {
    const state = connections.get(backendId)
    return state ? state.messages.get(channelId) : null
  }

  const subscribe = (backendId, channelId) => {
    ensureConnection(backendId)
    const state = connections.get(backendId)

    // 如果连接正在重连中 (state 为空)，返回空函数
    // 等连接成功后，MonitorCard 重新渲染会再次调用 subscribe
    if (!state || !state.client) return () => {}

    let subInfo = state.subscriptions.get(channelId)

    if (!subInfo) {
      try {
        const subId = state.client.subscribe(channelId)
        subInfo = { subId, count: 0 }
        state.subscriptions.set(channelId, subInfo)
        state.subIdToChannelId.set(subId, channelId)
        console.log(`[Foxglove] Subscribed ${channelId}`)
      } catch (e) {
        console.error('Subscribe failed:', e)
        return () => {}
      }
    }

    subInfo.count++

    return () => {
      // 检查 state 是否还存在 (可能因为断线重连被新建了)
      const currentState = connections.get(backendId)
      if (!currentState || !currentState.client) return

      // 检查 subInfo 是否还通过引用存在于 map 中
      // (断线重连会清空 subscriptions Map，所以旧的 subInfo 会失效)
      if (!currentState.subscriptions.has(channelId)) return

      subInfo.count--
      if (subInfo.count <= 0) {
        console.log(`[Foxglove] Unsubscribing ${channelId}`)
        try {
          currentState.client.unsubscribe(subInfo.subId)
          // eslint-disable-next-line no-unused-vars
        } catch (e) {
          /* ignore */
        }

        currentState.subscriptions.delete(channelId)
        currentState.subIdToChannelId.delete(subInfo.subId)
      }
    }
  }

  return { getTopics, getMessage, subscribe }
}
