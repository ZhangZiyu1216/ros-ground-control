import { reactive, ref } from 'vue'
import { FoxgloveClient } from '@foxglove/ws-protocol'
import { useRobotStore } from '../store/robot'
import { parse as parseRosMsg } from '@foxglove/rosmsg'
import { MessageReader } from '@foxglove/rosmsg-serialization'

// 数据仓库：按后端 ID 隔离
const dataStores = new Map()

const getDataStore = (backendId) => {
  if (!dataStores.has(backendId)) {
    dataStores.set(backendId, {
      topics: ref([]), // UI 列表
      // 使用 shallowRef 存储最新消息，避免深度监听带来的性能开销
      messages: reactive(new Map()),

      nameToChannel: new Map(), // topic -> channel info
      readers: new Map(), // topic -> decoding function

      // Graph 状态 (用于判断话题是否活跃)
      topicGraphState: new Map(), // topic -> { pubCount, subCount }
      activeTopicNames: new Set(),
      hasReceivedGraph: false,

      // 订阅管理
      subscriptions: new Map(), // topic -> { count, subId }
      subIdToTopicName: new Map() // subId -> topic
    })
  }
  return dataStores.get(backendId)
}

const activeConnections = new Map()

export function useFoxglove() {
  const robotStore = useRobotStore()

  // --- 1. 创建消息解码器 ---
  const createMessageReader = (channel) => {
    try {
      // ROS 1 / CDR
      if (
        channel.encoding === 'ros1' ||
        channel.schemaName === 'ros1' ||
        channel.encoding === 'cdr'
      ) {
        // parseRosMsg 能自动处理包含依赖的 schema 字符串
        const definitions = parseRosMsg(channel.schema, { ros2: false })
        const reader = new MessageReader(definitions)

        return (dataView) => {
          // 必须转换为 Uint8Array
          const uint8 = new Uint8Array(dataView.buffer, dataView.byteOffset, dataView.byteLength)
          return reader.readMessage(uint8)
        }
      }

      // JSON
      if (channel.encoding === 'json') {
        const decoder = new TextDecoder()
        return (dataView) => JSON.parse(decoder.decode(dataView))
      }

      // Raw Image Fallback (如果无法解析 schema 但知道是图片)
      if (channel.schemaName.includes('Image')) {
        return (dataView) => {
          const uint8 = new Uint8Array(dataView.buffer, dataView.byteOffset, dataView.byteLength)
          return { data: uint8, format: 'raw', _isBinary: true }
        }
      }

      return () => ({ error: `Unsupported encoding: ${channel.encoding}` })
    } catch (e) {
      console.warn(`[Foxglove] Reader init failed for ${channel.topic}:`, e)
      return () => ({ error: 'Parse Init Failed' })
    }
  }

  // --- 2. 状态计算 ---
  // 结合 Advertise (存在性) 和 Graph (活跃性) 更新列表
  const updateTopicList = (store) => {
    const list = Array.from(store.nameToChannel.values()).map((ch) => {
      const name = ch.topic

      // 判断是否活跃 (Is Alive?)
      // 如果还没收到过 Graph 数据，默认都认为是活跃的
      // 如果收到了，则检查 Graph 中是否有 Publisher
      let isAlive = true
      if (store.hasReceivedGraph) {
        const graphStats = store.topicGraphState.get(name)
        // 只有当有发布者时，或者我们自己订阅了且有其他订阅者时(略复杂，简单点：看发布者)
        isAlive = graphStats && graphStats.pubCount > 0
      }

      return {
        id: ch.id,
        topic: ch.topic,
        schemaName: ch.schemaName,
        schema: ch.schema,
        isAlive: !!isAlive
      }
    })

    store.topics.value = list
  }

  // --- 3. 重置 ---
  const resetConnectionState = (backendId) => {
    const store = getDataStore(backendId)
    store.topics.value = []
    store.messages.clear()
    store.nameToChannel.clear()
    store.readers.clear()
    store.subIdToTopicName.clear()
    store.topicGraphState.clear()
    store.hasReceivedGraph = false

    // 标记订阅失效，等待重连后恢复
    for (const sub of store.subscriptions.values()) {
      sub.subId = null
    }
  }

  // --- 4. 连接核心 ---
  const ensureConnection = (backendId) => {
    const robot = robotStore.clients[backendId]
    if (!robot || !robot.ip) return
    if (activeConnections.has(backendId)) return

    const store = getDataStore(backendId)
    const url = `ws://${robot.ip}:8765`
    console.log(`[Foxglove] Init connection: ${url}`)

    const client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL])
    })
    activeConnections.set(backendId, client)

    client.on('open', () => {
      console.log(`[Foxglove] Open: ${backendId}`)
      resetConnectionState(backendId)
    })

    // 关键：检查 Capability 并订阅 Graph
    client.on('serverInfo', (info) => {
      console.log(`[Foxglove] ServerInfo:`, info)
      if (info.capabilities && info.capabilities.includes('connectionGraph')) {
        client.subscribeConnectionGraph()
      }
    })

    // Advertise: 话题上线 (定义)
    client.on('advertise', (channels) => {
      channels.forEach((ch) => {
        store.nameToChannel.set(ch.topic, ch)
        // 懒加载 Reader
        if (!store.readers.has(ch.topic)) {
          store.readers.set(ch.topic, createMessageReader(ch))
        }

        // 自动恢复之前的订阅
        const existingSub = store.subscriptions.get(ch.topic)
        if (existingSub && existingSub.count > 0 && existingSub.subId === null) {
          try {
            const subId = client.subscribe(ch.id)
            existingSub.subId = subId
            store.subIdToTopicName.set(subId, ch.topic)
            console.log(`[Foxglove] Auto-resubscribed ${ch.topic}`)
          } catch (e) {
            console.warn(e)
          }
        }
      })
      updateTopicList(store)
    })

    // Unadvertise: 话题下线 (定义移除)
    client.on('unadvertise', (channelIds) => {
      const idsToRemove = new Set(channelIds)
      for (const [name, ch] of store.nameToChannel.entries()) {
        if (idsToRemove.has(ch.id)) {
          store.nameToChannel.delete(name)
          store.readers.delete(name)
          // 清理订阅映射
          const sub = store.subscriptions.get(name)
          if (sub) {
            store.subIdToTopicName.delete(sub.subId)
            sub.subId = null
          }
        }
      }
      updateTopicList(store)
    })

    // Connection Graph: 话题活跃状态 (Liveness)
    client.on('connectionGraphUpdate', (graph) => {
      store.hasReceivedGraph = true
      const graphState = store.topicGraphState

      // 1. Removed Topics: 话题彻底消失（无发布也无订阅）
      if (graph.removedTopics) {
        graph.removedTopics.forEach((name) => {
          // 直接删除统计信息，这将导致 updateTopicList 中获取不到 stats，isAlive 变为 false
          graphState.delete(name)
        })
      }

      // 2. Published Topics: 更新发布者数量
      if (graph.publishedTopics) {
        graph.publishedTopics.forEach((t) => {
          if (!graphState.has(t.name)) graphState.set(t.name, { pubCount: 0 })
          graphState.get(t.name).pubCount = t.publisherIds.length
        })
      }

      // [关键] 立即触发列表更新，重新计算 isAlive
      updateTopicList(store)
    })

    // Message Data
    // eslint-disable-next-line no-unused-vars
    client.on('message', ({ subscriptionId, timestamp, data }) => {
      const topicName = store.subIdToTopicName.get(subscriptionId)
      if (!topicName) return

      const reader = store.readers.get(topicName)
      if (reader) {
        // data 是 DataView，reader 负责转换
        const decoded = reader(data)
        // 注入时间戳 (BigInt -> Number for simplicity UI)
        // 注意：Foxglove 协议的 timestamp 是纳秒 (uint64)
        if (typeof decoded === 'object' && decoded !== null) {
          // 稍微注入一点元数据方便 debug
          // decoded._receiveTime = Number(timestamp) / 1e9
        }
        store.messages.set(topicName, decoded)
      }
    })

    client.on('close', () => {
      console.log(`[Foxglove] Close: ${backendId}`)
      activeConnections.delete(backendId)
      resetConnectionState(backendId)
    })
  }

  // --- API ---
  const getTopicsRef = (backendId) => {
    ensureConnection(backendId)
    return getDataStore(backendId).topics
  }

  const getMessage = (backendId, topicName) => {
    if (!topicName) return null
    return getDataStore(backendId).messages.get(topicName)
  }

  const subscribe = (backendId, topicName) => {
    ensureConnection(backendId)
    const store = getDataStore(backendId)
    const client = activeConnections.get(backendId)

    let subInfo = store.subscriptions.get(topicName)
    if (!subInfo) {
      subInfo = { count: 0, subId: null }
      store.subscriptions.set(topicName, subInfo)
    }
    subInfo.count++

    if (client && subInfo.subId === null) {
      const channel = store.nameToChannel.get(topicName)
      if (channel) {
        try {
          const subId = client.subscribe(channel.id)
          subInfo.subId = subId
          store.subIdToTopicName.set(subId, topicName)
          console.log(`[Foxglove] Subscribed ${topicName} (ch:${channel.id}, sub:${subId})`)
        } catch (e) {
          console.error(`[Foxglove] Sub Error ${topicName}:`, e)
        }
      } else {
        console.warn(`[Foxglove] Cannot subscribe ${topicName}: Not advertised yet`)
      }
    }

    return () => {
      subInfo.count--
      if (subInfo.count <= 0) {
        if (client && subInfo.subId !== null) {
          client.unsubscribe(subInfo.subId)
          store.subIdToTopicName.delete(subInfo.subId)
          console.log(`[Foxglove] Unsubscribed ${topicName}`)
        }
        store.subscriptions.delete(topicName)
      }
    }
  }

  return { getTopicsRef, getMessage, subscribe }
}
