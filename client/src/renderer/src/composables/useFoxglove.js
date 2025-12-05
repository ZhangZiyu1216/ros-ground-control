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
      subIdToTopicName: new Map(), // subId -> topic

      // 服务相关数据
      services: ref([]),
      nameToService: new Map() // serviceName -> serviceInfo
    })
  }
  return dataStores.get(backendId)
}
const activeConnections = new Map()

// 辅助函数：根据类型定义生成默认值对象
function generateDefaultValue(type, definitions) {
  // 1. 基础类型处理
  if (['bool'].includes(type)) return false
  if (
    [
      'int8',
      'uint8',
      'int16',
      'uint16',
      'int32',
      'uint32',
      'int64',
      'uint64',
      'float32',
      'float64'
    ].includes(type)
  )
    return 0
  if (['string'].includes(type)) return ''
  if (['time', 'duration'].includes(type)) return { secs: 0, nsecs: 0 }

  // 2. 查找复杂类型定义
  const def = definitions.find((d) => d.name === type)
  if (!def) return {} // 未知类型返回空对象

  const obj = {}
  for (const field of def.definitions) {
    if (field.isConstant) continue // 跳过常量

    let value
    if (field.isArray) {
      // 数组类型：如果是定长数组，生成对应长度的默认值；变长数组返回空数组
      if (field.arrayLength) {
        value = Array(field.arrayLength).fill(generateDefaultValue(field.type, definitions))
      } else {
        value = []
      }
    } else {
      value = generateDefaultValue(field.type, definitions)
    }
    obj[field.name] = value
  }
  return obj
}

// [新增] 更新服务列表 UI
const updateServiceList = (store) => {
  const list = Array.from(store.nameToService.values()).map((s) => ({
    id: s.id,
    name: s.name,
    type: s.type, // e.g. "std_srvs/SetBool"
    requestSchema: s.requestSchema, // Foxglove 协议直接提供了分离的 Schema
    responseSchema: s.responseSchema
  }))
  store.services.value = list
}

// 辅助函数：简单的 ROS 风格 YAML 转换器
// 规则：数组使用 [a, b]，对象使用换行缩进
function objectToYaml(obj, indentLevel = 0) {
  const indent = '  '.repeat(indentLevel)
  let output = ''

  for (const [key, value] of Object.entries(obj)) {
    // 数组处理：保持为行内 [a, b, c] 风格，更加紧凑且不易出错
    if (Array.isArray(value)) {
      const arrStr = JSON.stringify(value)
      output += `${indent}${key}: ${arrStr}\n`
    }
    // 对象处理：递归
    else if (typeof value === 'object' && value !== null) {
      output += `${indent}${key}:\n`
      output += objectToYaml(value, indentLevel + 1)
    }
    // 基础类型处理
    else {
      let displayValue = value
      // 只有空字符串需要显式引号 ''，否则不加引号 (YAML标准)
      if (typeof value === 'string') {
        if (value === '') displayValue = "''"
        // 如果字符串包含特殊字符，也可以在这里处理，但通常 ROS 消息很简单
      }
      output += `${indent}${key}: ${displayValue}\n`
    }
  }
  return output
}

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
      // 1. 清空当前图状态 (Discard old state)
      // 我们直接清空 Map，只保留最新的快照数据
      store.topicGraphState.clear()
      // 2. 重建 Publishers 状态
      if (graph.publishedTopics) {
        graph.publishedTopics.forEach((t) => {
          let stats = store.topicGraphState.get(t.name)
          if (!stats) {
            stats = { pubCount: 0, subCount: 0 }
            store.topicGraphState.set(t.name, stats)
          }
          stats.pubCount = t.publisherIds.length
        })
      }
      // 3. 重建 Subscribers 状态
      if (graph.subscribedTopics) {
        graph.subscribedTopics.forEach((t) => {
          let stats = store.topicGraphState.get(t.name)
          if (!stats) {
            stats = { pubCount: 0, subCount: 0 }
            store.topicGraphState.set(t.name, stats)
          }
          stats.subCount = t.subscriberIds.length
        })
      }
      // 4. 立即触发列表更新
      updateTopicList(store)
    })

    // [新增] 监听服务列表更新
    client.on('advertiseServices', (services) => {
      const store = getDataStore(backendId)
      services.forEach((s) => {
        store.nameToService.set(s.name, s)
      })
      updateServiceList(store)
    })

    client.on('unadvertiseServices', (serviceIds) => {
      const store = getDataStore(backendId)
      const idsToRemove = new Set(serviceIds)
      for (const [name, s] of store.nameToService.entries()) {
        if (idsToRemove.has(s.id)) {
          store.nameToService.delete(name)
        }
      }
      updateServiceList(store)
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

  // 获取话题的消息类型名称和默认消息模板
  const getTopicTypeAndTemplate = (backendId, topicName) => {
    ensureConnection(backendId)
    const store = getDataStore(backendId)
    const channel = store.nameToChannel.get(topicName)

    if (!channel) {
      return { type: '', template: '' }
    }
    try {
      // 1. 解析 Schema
      const definitions = parseRosMsg(channel.schema, { ros2: false })
      // 2. 找到根定义 (通常是数组的第一个，或者与 schemaName 匹配的)
      // 注意：Foxglove 解析出来的 definitions 可能包含依赖项
      const rootDef = definitions.find((d) => d.name === channel.schemaName) || definitions[0]
      if (!rootDef) return { type: channel.schemaName, template: '{}' }
      // 3. 递归生成对象
      const msgObj = generateDefaultValue(rootDef.name, definitions)
      // 4. 转换为 YAML 风格的字符串 (为了不仅好读，且容易被 rostopic pub 接受，其实 JSON 格式最稳妥)
      // 这里我们返回格式化好的 JSON，用户可以编辑，最后提交时我们再压缩
      const templateStr = objectToYaml(msgObj).trimEnd()

      return {
        type: channel.schemaName,
        template: templateStr // 返回带缩进的 JSON 字符串，方便用户编辑
      }
    } catch (e) {
      console.error('[Foxglove] Generate template failed:', e)
      return { type: channel.schemaName || '', template: '' }
    }
  }

  // [新增] 获取服务列表引用
  const getServicesRef = (backendId) => {
    ensureConnection(backendId)
    return getDataStore(backendId).services
  }

  // [新增] 获取服务类型和请求模板
  const getServiceTypeAndTemplate = (backendId, serviceName) => {
    ensureConnection(backendId)
    const store = getDataStore(backendId)
    const service = store.nameToService.get(serviceName)

    if (!service) {
      return { type: '', template: '' }
    }

    try {
      // 1. 解析 Request Schema
      // Foxglove 协议通常分别提供 requestSchema 和 responseSchema
      // 如果没有直接提供，可能需要处理 raw schema，但标准实现都会分离
      const definitions = parseRosMsg(service.requestSchema, { ros2: false })

      // 2. 找到根定义 (通常只有一个，就是 Request 结构体)
      // 注意：服务请求类型的命名通常是 "Pkg/TypeRequest" 或直接是 "TypeRequest"
      const rootDef = definitions[0]

      if (!rootDef) return { type: service.type, template: '' }

      // 3. 生成默认对象
      const msgObj = generateDefaultValue(rootDef.name, definitions)

      // 4. 转为 YAML
      const templateStr = objectToYaml(msgObj).trimEnd()

      return {
        type: service.type,
        template: templateStr
      }
    } catch (e) {
      console.error('[Foxglove] Generate service template failed:', e)
      return { type: service.type || '', template: '' }
    }
  }

  return {
    getTopicsRef,
    getMessage,
    subscribe,
    getTopicTypeAndTemplate,
    getServicesRef,
    getServiceTypeAndTemplate
  }
}
