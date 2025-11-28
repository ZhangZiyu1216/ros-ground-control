import { reactive, ref } from 'vue'
import { FoxgloveClient } from '@foxglove/ws-protocol'
import { useRobotStore } from '../store/robot'
import { parse as parseRosMsg } from '@foxglove/rosmsg'
import { MessageReader } from '@foxglove/rosmsg-serialization'

// --- 1. 数据仓库 (Data Stores) ---
// Key: backendId (UUID)
const dataStores = new Map()

const getDataStore = (backendId) => {
  if (!dataStores.has(backendId)) {
    dataStores.set(backendId, {
      // --- 外部响应式数据 (UI 绑定) ---
      topics: ref([]), // 供下拉列表使用
      messages: reactive(new Map()), // TopicName -> Decoded Message

      // --- 内部状态 ---
      nameToChannel: new Map(), // TopicName -> ChannelInfo (ID, Schema)
      readers: new Map(), // TopicName -> MessageReader Function

      // 活跃状态管理
      // TopicName -> { pubCount: Number, subCount: Number }
      topicGraphState: new Map(),
      activeTopicNames: new Set(), // 缓存计算结果
      hasReceivedGraph: false, // 是否收到过图更新

      // 订阅意图管理
      // TopicName -> { count: Number, subId: Number|null, expectedType: String }
      subscriptions: new Map(),

      // 反查映射 (subId -> TopicName)
      subIdToTopicName: new Map()
    })
  }
  return dataStores.get(backendId)
}

// --- 2. 连接池 ---
const activeConnections = new Map()

export function useFoxglove() {
  const robotStore = useRobotStore()

  // --- 内部：创建消息读取器 ---
  const createMessageReader = (channel) => {
    try {
      if (channel.encoding === 'json') {
        return (data) => JSON.parse(new TextDecoder().decode(data))
      }
      if (channel.encoding === 'ros1' || channel.schemaName === 'ros1') {
        const definitions = parseRosMsg(channel.schema)
        const reader = new MessageReader(definitions)
        return (data) => reader.readMessage(data)
      }
      // 二进制回退 (如 CompressedImage)
      return (data) => {
        if (channel.schemaName.includes('Image')) {
          const binary = new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
          return { data: binary, format: 'raw', _isBinary: true }
        }
        return { error: `Unsupported encoding: ${channel.encoding}` }
      }
    } catch (e) {
      console.warn(`Failed to create reader for ${channel.topic}:`, e)
      return () => ({ error: 'Parse Error' })
    }
  }

  // --- 内部：计算并更新活跃话题列表 ---
  const recalculateActiveTopics = (store) => {
    const newActiveSet = new Set()

    // 遍历图状态
    for (const [name, stats] of store.topicGraphState.entries()) {
      // 1. 发布者存在 -> 活跃
      if (stats.pubCount > 0) {
        newActiveSet.add(name)
        continue
      }

      // 2. 只有订阅者的情况：排除 Foxglove 自身的订阅
      // 检查我们自己是否订阅了这个话题
      const mySubInfo = store.subscriptions.get(name)
      const amISubscribing = mySubInfo && mySubInfo.count > 0

      // 估算“其他订阅者”数量
      // 注意：这里假设 Foxglove Bridge 将每个 WebSocket Client 算作 1 个 Subscriber
      const otherSubCount = stats.subCount - (amISubscribing ? 1 : 0)

      // 如果还有其他订阅者 -> 活跃
      if (otherSubCount > 0) {
        newActiveSet.add(name)
      }
    }

    store.activeTopicNames = newActiveSet
    updateTopicList(store)
  }

  // --- 内部：更新 UI 列表 (合并元数据与活跃状态) ---
  const updateTopicList = (store) => {
    const list = Array.from(store.nameToChannel.values()).map((ch) => {
      const name = ch.topic
      // 如果还没收到图更新，默认全显示(乐观模式)；否则查 Set
      const isActive = !store.hasReceivedGraph ? true : store.activeTopicNames.has(name)

      return {
        ...ch,
        isAlive: isActive
      }
    })
    store.topics.value = list
  }

  // --- 内部：重置连接状态 ---
  const resetConnectionState = (backendId) => {
    const store = getDataStore(backendId)

    store.topics.value = []
    store.messages.clear()
    store.nameToChannel.clear()
    store.readers.clear()
    store.subIdToTopicName.clear()

    // 重置图状态
    store.topicGraphState.clear()
    store.activeTopicNames.clear()
    store.hasReceivedGraph = false

    // 重置订阅 ID (保留意图)
    for (const sub of store.subscriptions.values()) {
      sub.subId = null
    }
  }

  // --- 核心：连接逻辑 ---
  const ensureConnection = (backendId) => {
    const robot = robotStore.clients[backendId]
    if (!robot || !robot.ip) return
    if (activeConnections.has(backendId)) return

    const store = getDataStore(backendId)
    const url = `ws://${robot.ip}:8765`
    console.log(`[Foxglove] Connecting to ${url}`)

    const client = new FoxgloveClient({
      ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL])
    })
    activeConnections.set(backendId, client)

    // 1. Open
    client.on('open', () => {
      console.log(`[Foxglove] Connected to ${backendId}`)
      resetConnectionState(backendId)
    })

    // 2. ServerInfo (订阅图更新)
    client.on('serverInfo', (info) => {
      if (info.capabilities && info.capabilities.includes('connectionGraph')) {
        client.subscribeConnectionGraph()
      }
    })

    // 3. Cleanup
    const cleanup = () => {
      console.log(`[Foxglove] Socket closed ${backendId}`)
      activeConnections.delete(backendId)
      resetConnectionState(backendId)
    }
    client.on('error', (e) => console.warn(`[Foxglove] Error:`, e))
    client.on('close', cleanup)

    // 4. Advertise (收到元数据)
    client.on('advertise', (newChannels) => {
      let listChanged = false
      newChannels.forEach((ch) => {
        const topicName = ch.topic
        store.nameToChannel.set(topicName, ch)

        // 创建 Reader
        store.readers.set(topicName, createMessageReader(ch))

        // 自动恢复订阅 (Auto-Resubscribe)
        const pendingSub = store.subscriptions.get(topicName)
        if (pendingSub && pendingSub.count > 0 && pendingSub.subId === null) {
          // 简单类型校验
          if (!pendingSub.expectedType || pendingSub.expectedType === ch.schemaName) {
            try {
              const newSubId = client.subscribe(ch.id)
              pendingSub.subId = newSubId
              store.subIdToTopicName.set(newSubId, topicName)
            } catch (e) {
              console.warn('Resubscribe error', e)
            }
          }
        }
        listChanged = true
      })

      if (listChanged) {
        // 如果有新话题，重新计算活跃状态（因为可能有新话题已经在 graphState 里了）
        recalculateActiveTopics(store)
      }
    })

    // 5. Unadvertise
    client.on('unadvertise', (channelIds) => {
      const idsToRemove = new Set(channelIds)
      let listChanged = false
      for (const [name, ch] of store.nameToChannel.entries()) {
        if (idsToRemove.has(ch.id)) {
          store.nameToChannel.delete(name)
          store.readers.delete(name)
          store.messages.delete(name)
          // 订阅 ID 清理
          const sub = store.subscriptions.get(name)
          if (sub) {
            store.subIdToTopicName.delete(sub.subId)
            sub.subId = null
          }
          listChanged = true
        }
      }
      if (listChanged) recalculateActiveTopics(store)
    })

    // 6. Graph Update (核心活跃检测)
    client.on('connectionGraphUpdate', (graph) => {
      const graphState = store.topicGraphState
      let graphChanged = false

      // 移除
      if (graph.removedTopics) {
        graph.removedTopics.forEach((name) => {
          if (graphState.has(name)) {
            graphState.delete(name)
            graphChanged = true
          }
        })
      }

      // 发布者更新
      if (graph.publishedTopics) {
        graph.publishedTopics.forEach((t) => {
          if (!graphState.has(t.name)) graphState.set(t.name, { pubCount: 0, subCount: 0 })
          graphState.get(t.name).pubCount = t.publisherIds.length
          graphChanged = true
        })
      }

      // 订阅者更新
      if (graph.subscribedTopics) {
        graph.subscribedTopics.forEach((t) => {
          if (!graphState.has(t.name)) graphState.set(t.name, { pubCount: 0, subCount: 0 })
          graphState.get(t.name).subCount = t.subscriberIds.length
          graphChanged = true
        })
      }

      if (graphChanged || !store.hasReceivedGraph) {
        store.hasReceivedGraph = true
        recalculateActiveTopics(store)
      }
    })

    // 7. Message
    client.on('message', ({ subscriptionId, data }) => {
      const topicName = store.subIdToTopicName.get(subscriptionId)
      if (!topicName) return
      const reader = store.readers.get(topicName)
      if (reader) {
        try {
          store.messages.set(topicName, reader(data))
          // eslint-disable-next-line no-unused-vars
        } catch (e) {
          // no use
        }
      }
    })
  }

  // --- 导出 API ---

  const getTopicsRef = (backendId) => {
    ensureConnection(backendId)
    return getDataStore(backendId).topics
  }

  const getMessage = (backendId, topicName) => {
    if (!topicName) return null
    return getDataStore(backendId).messages.get(topicName)
  }

  const subscribe = (backendId, topicName, expectedType = null) => {
    ensureConnection(backendId)
    const store = getDataStore(backendId)
    const client = activeConnections.get(backendId)

    let subInfo = store.subscriptions.get(topicName)
    if (!subInfo) {
      subInfo = { count: 0, subId: null, expectedType }
      store.subscriptions.set(topicName, subInfo)
    } else {
      if (expectedType) subInfo.expectedType = expectedType
    }

    subInfo.count++

    // 立即订阅逻辑
    if (client && subInfo.subId === null) {
      const channel = store.nameToChannel.get(topicName)
      if (channel) {
        try {
          const subId = client.subscribe(channel.id)
          subInfo.subId = subId
          store.subIdToTopicName.set(subId, topicName)
          // 订阅发生变化，重新计算活跃状态 (因为我们的订阅会影响 subCount)
          // 注意：Bridge 之后会发 Graph Update，但我们也可以预判
          setTimeout(() => recalculateActiveTopics(store), 100)
        } catch (e) {
          console.warn('Sub failed', e)
        }
      }
    }

    // Unsubscribe Cleanup
    return () => {
      subInfo.count--
      if (subInfo.count <= 0) {
        if (client && subInfo.subId !== null) {
          client.unsubscribe(subInfo.subId)
          store.subIdToTopicName.delete(subInfo.subId)
        }
        store.subscriptions.delete(topicName)
        // 退订后，该话题可能变成“死”话题，重新计算
        setTimeout(() => recalculateActiveTopics(store), 100)
      }
    }
  }

  return { getTopicsRef, getMessage, subscribe }
}
