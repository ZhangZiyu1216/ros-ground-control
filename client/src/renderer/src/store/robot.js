import { defineStore } from 'pinia'
import { ref, computed, reactive } from 'vue'
import { createApi } from '../api/request'
import { ElMessage, ElNotification } from 'element-plus'

// --- 配置常量 ---
const HEALTH_CHECK_INTERVAL = 1000
const MAX_MISSED_HEARTBEATS = 3
const AUTO_DISCONNECT_THRESHOLD = 15
const MIN_CONNECTION_TIME = 500
const HANDSHAKE_TIMEOUT = 500
const MAX_CONNECT_RETRIES = 2

export const useRobotStore = defineStore('robot', () => {
  const clients = reactive({})
  const activeID = ref(null)
  const clipboard = ref(null) // 全局剪贴板，结构: { sourceId: 'uuid', path: '/a.txt', mode: 'copy'|'cut', name: 'a.txt', isDir: boolean }
  const setClipboard = (data) => {
    clipboard.value = data
  }

  const activeClient = computed(() => (activeID.value ? clients[activeID.value] : null))
  const getCacheKey = (id) => `robot_nodes_cache_${id}` // --- 辅助：缓存 Key 生成 ---
  const getSequenceCacheKey = (id) => `robot_sequences_${id}`
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms))

  const sudoPassword = ref('')
  const setSudoPassword = (pwd) => {
    sudoPassword.value = pwd
  }

  // --- 内部工具：通过 mDNS 找回 IP 和 ID---
  const findNewIpByHostname = (targetHostname) => {
    return new Promise((resolve) => {
      console.log(`[Discovery] Looking for IP of hostname: ${targetHostname}`)

      let result = null
      let cleanupListener = null

      const onFound = (service) => {
        const deviceHostname = service.txt?.hostname || service.host || ''
        const deviceId = service.txt?.id // 获取 mDNS 中的 id 字段

        const cleanTarget = targetHostname.toLowerCase().replace('.local', '')
        const cleanDevice = deviceHostname.toLowerCase().replace('.local', '')

        if (cleanTarget && cleanDevice === cleanTarget) {
          const ipv4 = service.addresses?.find((addr) => addr.includes('.'))
          if (ipv4) {
            result = { ip: ipv4, id: deviceId } // 返回对象
            console.log(`[Discovery] Found ${targetHostname}: IP=${ipv4}, ID=${deviceId}`)
          }
        }
      }

      try {
        cleanupListener = window.api.onMdnsServiceFound(onFound)
        window.api.startMdnsScan()
      } catch (e) {
        console.error('[Discovery] Failed to start scan:', e)
        resolve(null)
        return
      }

      setTimeout(() => {
        window.api.stopMdnsScan()
        if (typeof cleanupListener === 'function') cleanupListener()
        resolve(result)
      }, 2500)
    })
  }

  // 内部辅助：等待 ROS 服务就绪
  const ensureRosStackReady = async (client) => {
    if (client.serviceStatus.roscore === 'active' && client.serviceStatus.bridge === 'active') {
      return
    }
    console.log('ROS services not ready, auto-starting...')
    client.isStackLoading = true
    try {
      await client.api.post('/ros/action', { service: 'stack', action: 'start' })

      let retries = 0
      while (retries < 15) {
        await sleep(1000)
        const res = await client.api.get('/proc/list')
        updateServiceStatusFromProcs(client, res.processes)
        if (client.serviceStatus.roscore === 'active' && client.serviceStatus.bridge === 'active') {
          return
        }
        retries++
      }
      throw new Error('ROS 服务启动超时')
    } finally {
      client.isStackLoading = false
    }
  }

  // --- 心跳逻辑 ---
  const startHealthCheck = (id) => {
    // 注意：这里的参数 id 即 key
    const client = clients[id]
    if (!client) return
    if (client.timer) clearInterval(client.timer)

    console.log(`[Heartbeat] Started for ID: ${id} (${client.ip})`)

    client.timer = setInterval(async () => {
      // 0. 安全检查
      if (!clients[id] || client.status === 'disconnected') {
        if (client.timer) clearInterval(client.timer)
        client.timer = null
        return
      }

      try {
        // --- 正常心跳 ---
        const res = await client.api.get('/proc/list')

        // 成功！
        if (client.missedHeartbeats > 0) {
          console.log(`[Heartbeat] Recovered after ${client.missedHeartbeats} misses.`)
        }
        client.missedHeartbeats = 0

        // 如果之前是 Failed (红色)，恢复为 Ready (绿色)
        if (client.status === 'failed') {
          client.status = 'ready'
          ElNotification({
            title: '连接恢复',
            message: `${client.name || client.hostname} 已自动重连`,
            type: 'success'
          })
        }

        updateServiceStatusFromProcs(client, res.processes)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        // --- 失败处理 ---
        if (client.status === 'disconnected') return

        client.missedHeartbeats++
        // 1. 变红 (Failed) - 短暂掉线
        if (client.missedHeartbeats >= MAX_MISSED_HEARTBEATS && client.status === 'ready') {
          client.status = 'failed'
          client.serviceStatus = { roscore: 'unknown', bridge: 'unknown' }
          ElNotification({
            title: '连接中断',
            message: `与 ${client.name} 失去联系，正在尝试重连...`,
            type: 'error',
            duration: 3000
          })
        }

        // 2. 【新增】尝试自动找回 IP (在变红期间)
        // 条件：有 hostname，且处于 failed 状态，且每隔 4 次心跳尝试一次 (防止 mDNS 过于频繁)
        if (client.status === 'failed' && client.hostname && client.missedHeartbeats % 4 === 0) {
          console.log(`[Heartbeat] Attempting to recover IP for ${client.hostname}...`)

          // 不 await，让它异步跑，不要阻塞心跳定时器
          findNewIpByHostname(client.hostname).then((result) => {
            // 如果找到了，且 IP 确实变了
            if (result && result.ip && result.ip !== client.ip) {
              console.log(`[Heartbeat] Auto-recovered IP! ${client.ip} -> ${result.ip}`)

              // 立即更新内部状态
              client.ip = result.ip
              const baseUrl = `http://${result.ip}:8080/api`
              client.api = createApi(baseUrl) // 替换 API 实例
              client.ipChanged = true // 标记变更，通知 UI 保存

              // 重置心跳计数，让下一次 tick 尝试新的 API
              // (我们不手动设为 ready，让下一次真实心跳请求成功来设为 ready)
              client.missedHeartbeats = 0
            }
          })
        }

        // 3. 变灰 (Disconnected) - 超时彻底放弃
        if (client.missedHeartbeats >= AUTO_DISCONNECT_THRESHOLD) {
          if (client.status === 'disconnected') return
          // 先清理定时器，防止后续重复执行
          clearInterval(client.timer)
          client.timer = null

          client.status = 'disconnected'
          ElNotification({
            title: '连接超时',
            message: `${client.name} 已断开。`,
            type: 'info',
            duration: 3000
          })
        }
      }
    }, HEALTH_CHECK_INTERVAL)
  }

  const updateServiceStatusFromProcs = (client, processes) => {
    if (!Array.isArray(processes)) return
    // 1. 更新系统服务灯
    const hasRoscore = processes.some((p) => p.id.includes('roscore') || p.cmd.includes('roscore'))
    client.serviceStatus.roscore = hasRoscore ? 'active' : 'inactive'
    const hasBridge = processes.some(
      (p) =>
        p.id.includes('foxglove') ||
        p.id.includes('bridge') ||
        p.cmd.includes('foxglove') ||
        p.cmd.includes('rosbridge')
    )
    client.serviceStatus.bridge = hasBridge ? 'active' : 'inactive'
    // 2. 更新 Dashboard 节点状态
    // 遍历 client.nodes，看 processes 里有没有 id 匹配的
    if (client.nodes) {
      client.nodes.forEach((node) => {
        // 匹配规则：进程 ID 等于 节点名称 (我们在 startNodeProcess 里是这么传的)
        const proc = processes.find((p) => p.id === node.name)
        const now = Date.now()

        if (proc) {
          // --- 进程存在 ---
          if (node.status === 'stopping') {
            // [新增] 正在停止中，但进程还在
            // 检查超时 (例如 10秒)，大于后端的总超时时间
            if (node._stopTime && now - node._stopTime > 8000) {
              // 超时了还没死，可能卡住了，重置为 running
              node.status = 'running'
              node._stopTime = null
            }
            // 否则保持 'stopping' 状态，等待下一次心跳
          } else {
            // 正常运行
            node.status = 'running'
            node._startTime = null
          }
        } else {
          // --- 进程不存在 ---
          if (node.status === 'starting') {
            // 正在启动中，但进程还没出现
            if (node._startTime && now - node._startTime > 5000) {
              node.status = 'stopped' // 超时
              node._startTime = null
            }
            // 否则保持 'starting'
          } else {
            // 只要进程没了，就视为 stopped (涵盖了 stopping -> stopped 的成功转换)
            node.status = 'stopped'
            node._stopTime = null
          }
        }
      })
    }
  }

  // [新增] 同步初始化占位符 (防止 UI 闪烁)
  const initClientPlaceholder = (settings) => {
    // 这里的 id 就是 settings.id (UUID)
    // 逻辑与 addConnection 开头类似，但它是纯同步的
    let currentKey = settings.id || settings.ip

    if (!clients[currentKey]) {
      clients[currentKey] = {
        id: settings.id,
        ip: settings.ip,
        hostname: settings.hostname,
        name: settings.name,
        status: 'setting_up', // 关键：初始状态设为连接中
        api: null,
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' },
        nodes: [],
        nodeLogs: {},
        terminals: [],
        logWs: null,
        nextTerminalId: 1
      }
    } else {
      // 如果已存在，重置状态
      clients[currentKey].status = 'setting_up'
    }
    activeID.value = currentKey
  }

  // --- addConnection ---
  const addConnection = async (settings) => {
    const targetIp = settings.ip
    const port = settings.port || 8080
    const hostname = settings.hostname

    // 1. 尝试在现有的 clients 中查找是否已经有匹配该 IP 或 Hostname 的对象 (Key 可能是 UUID)
    let existingKey = Object.keys(clients).find((key) => {
      const c = clients[key]
      return c.ip === targetIp || (hostname && c.hostname === hostname)
    })

    // 如果找到了现有对象，且状态是 ready，直接切换
    if (existingKey && clients[existingKey].status === 'ready') {
      activeID.value = existingKey // 这里其实是 activeId
      return
    }

    // 2. 准备 Key
    // 如果找到了现有对象（可能是断开状态），复用它的 Key (UUID)
    // 如果没找到，暂时用 targetIp 作 Key，等握手拿到 ID 后再迁移
    let currentKey = existingKey || targetIp

    // 清理旧定时器
    if (clients[currentKey] && clients[currentKey].timer) clearInterval(clients[currentKey].timer)

    // 初始化/更新对象状态 (保留 usedUrl 等字段如果存在)
    if (!clients[currentKey]) {
      clients[currentKey] = {
        id: null, // 尚未知晓
        ip: targetIp,
        hostname: hostname,
        name: settings.name,
        status: 'setting_up',
        nodes: [],
        api: null,
        sysInfo: {},
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' },
        missedHeartbeats: 0,
        timer: null,
        ipChanged: false,
        isStackLoading: false,
        // 日志与终端相关
        nodeLogs: {}, // nodeLogs结构: { 'node_id': { status: 'running'|'stopped', lines: [] } }
        terminals: [], // terminals结构: [ { id: 1, name: '终端 1' } ]
        logWs: null, // 统一日志 WebSocket 句柄
        nextTerminalId: 1 // 用于生成终端名称
      }
    } else {
      // 复用现有对象，更新状态为 loading
      clients[currentKey].status = 'setting_up'
      clients[currentKey].ip = targetIp // 更新尝试连接的 IP
      clients[currentKey].name = settings.name
      // 确保复用时，如果是个老旧的占位对象，补全字段
      if (!clients[currentKey].nodeLogs) clients[currentKey].nodeLogs = {}
      if (!clients[currentKey].terminals) clients[currentKey].terminals = []
    }

    // 尝试加载本地缓存，防止界面空白
    try {
      const cached = await loadNodesFromCache(currentKey)
      if (cached) {
        clients[currentKey].nodes = cached
      }
    } catch (e) {
      console.warn('Failed to load node cache', e)
    }

    activeID.value = currentKey
    const startTime = Date.now()

    let apiInstance = null
    let finalIp = targetIp
    let finalId = null // 最终拿到的硬件 ID
    let infoRes = {}
    let procRes = { processes: [] }
    let connectError = null

    // 尝试连接，包括直连、mDNS找回
    try {
      // === 阶段 1: 直连尝试 ===
      let attempts = 0
      while (attempts < MAX_CONNECT_RETRIES) {
        try {
          const baseUrl = `http://${targetIp}:${port}/api`
          const tempApi = createApi(baseUrl)
          const reqConfig = { timeout: HANDSHAKE_TIMEOUT }

          const results = await Promise.all([
            tempApi.get('/sys/ping', reqConfig),
            tempApi.get('/sys/info', reqConfig).catch(() => ({})),
            tempApi.get('/proc/list', reqConfig).catch(() => ({ processes: [] }))
          ])

          apiInstance = tempApi
          infoRes = results[1]
          procRes = results[2]
          finalId = infoRes.id // 从 /sys/info 获取 ID
          break
        } catch (e) {
          attempts++
          connectError = e
          await sleep(200)
        }
      }

      // === 阶段 2: mDNS 找回 ===
      if (!apiInstance && hostname && hostname !== 'localhost') {
        console.log(`[Connection] IP ${targetIp} failed. Resolving hostname: ${hostname}...`)
        const result = await findNewIpByHostname(hostname)

        if (result && result.ip) {
          const newIp = result.ip
          // 如果 mDNS 给了 ID，先存一下
          if (result.id) finalId = result.id

          console.log(`[Connection] Resolved new IP: ${newIp}. Retrying...`)
          try {
            const baseUrl = `http://${newIp}:${port}/api`
            const tempApi = createApi(baseUrl)
            const reqConfig = { timeout: HANDSHAKE_TIMEOUT }

            const results = await Promise.all([
              tempApi.get('/sys/ping', reqConfig),
              tempApi.get('/sys/info', reqConfig).catch(() => ({})),
              tempApi.get('/proc/list', reqConfig).catch(() => ({ processes: [] }))
            ])

            apiInstance = tempApi
            finalIp = newIp
            infoRes = results[1]
            procRes = results[2]
            // 再次确认 ID (API 返回的更准)
            if (infoRes.id) finalId = infoRes.id

            // 标记 IP 变更
            if (finalIp !== targetIp) {
              if (clients[currentKey]) clients[currentKey].ipChanged = true
            }
          } catch (e) {
            connectError = e
          }
        }
      }

      // === 阶段 3: 最终判定与 Key 迁移 ===
      if (!apiInstance) {
        if (clients[currentKey]) clients[currentKey].status = 'failed'
        throw connectError || new Error('Connection failed')
      }

      // 补足动画时间
      const elapsed = Date.now() - startTime
      if (elapsed < MIN_CONNECTION_TIME) {
        await sleep(MIN_CONNECTION_TIME - elapsed)
      }

      // --- 关键：Store Key 归一化 (Merge Logic) ---
      // 如果我们拿到了 valid ID，我们要确保数据存储在 clients[finalId] 下
      if (finalId) {
        // 1. 如果当前的 Key (currentKey) 已经是 finalId，无需迁移，直接更新
        if (currentKey === finalId) {
          // just update
        }
        // 2. 如果 currentKey 不是 ID (比如是 IP)，检查 clients[finalId] 是否已存在
        else if (clients[finalId]) {
          // Case: 我们用 IP 连上了，结果发现这个 ID 已经在列表里了 (可能是之前的旧连接)
          console.log(`[Connection] Merging temp session ${currentKey} into existing ID ${finalId}`)

          // 将当前 session 的重要状态合并到那个已存在的对象里
          const target = clients[finalId]
          target.api = apiInstance
          target.ip = finalIp
          target.status = 'ready'
          target.sysInfo = infoRes
          target.serviceStatus = { roscore: 'unknown', bridge: 'unknown' } // 重置一下
          target.hostname = hostname // 确保 hostname 最新
          target.ipChanged = clients[currentKey].ipChanged // 传递标记

          // 此时 activeIp 还是旧的 currentKey，需要删掉旧的，切换 activeIp
          if (currentKey !== finalId) {
            delete clients[currentKey]
          }
          currentKey = finalId // 更新指针
        }
        // 3. clients[finalId] 不存在，我们需要将 currentKey 重命名为 finalId
        else {
          console.log(`[Connection] Promoting temp session ${currentKey} to ID ${finalId}`)
          clients[finalId] = clients[currentKey] // 移动引用
          clients[finalId].id = finalId
          delete clients[currentKey]
          currentKey = finalId
        }
      } else {
        console.warn('[Connection] Agent did not return an ID. Staying with IP/Temp key.')
      }

      // 更新最终对象状态
      const finalClient = clients[currentKey]
      finalClient.api = apiInstance
      finalClient.status = 'ready'
      finalClient.ip = finalIp
      finalClient.sysInfo = infoRes
      syncNodes(currentKey)
      syncSequences(currentKey)

      updateServiceStatusFromProcs(finalClient, procRes.processes)
      startLogStream(currentKey)

      // 切换视图到最终的 ID Key
      activeID.value = currentKey
      startHealthCheck(currentKey)

      // 因为任何原因没有连接成功
    } catch (e) {
      if (clients[currentKey]) clients[currentKey].status = 'failed'
      throw e
    }
  }

  const removeConnection = (id) => {
    const client = clients[id]
    if (client) {
      if (client.timer) clearInterval(client.timer)
      client.status = 'disconnected'
      client.serviceStatus = { roscore: 'unknown', bridge: 'unknown' }
      client.logWs.close()
    }
  }

  const refreshStatus = async (id) => {
    const client = clients[id]
    if (client && client.api && client.status === 'ready') {
      try {
        const res = await client.api.get('/proc/list')
        updateServiceStatusFromProcs(client, res.processes)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        // no use
      }
    }
  }

  const setClientStatus = (id, status) => {
    if (clients[id]) {
      clients[id].status = status
      if (status === 'disconnected' && clients[id].timer) {
        clearInterval(clients[id].timer)
        clients[id].timer = null
      }
    }
  }

  // 1. 同步节点 (Pull & Cache)
  const syncNodes = async (clientId) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    try {
      const res = await client.api.get('/nodes') // 获取列表
      if (Array.isArray(res)) {
        // 映射数据结构 (Agent 返回 args 数组，前端可能需要处理)
        client.nodes = res.map((n) => ({
          ...n,
          status: 'stopped' // 默认状态，后续通过 proc/list 更新真实状态
        }))
        // 更新缓存
        await window.api.setConfig(getCacheKey(clientId), JSON.parse(JSON.stringify(client.nodes)))
      }
    } catch (e) {
      console.error('Failed to sync nodes:', e)
    }
  }

  // 2. 保存/更新节点 (Push & Cache)
  const saveNodeConfig = async (clientId, nodeData) => {
    if (!clients[clientId]) loadNodesFromCache(clientId)
    const client = clients[clientId]
    // 先更新本地 UI (乐观更新)
    const existingIdx = client.nodes.findIndex((n) => n.id === nodeData.id)
    if (existingIdx !== -1) {
      client.nodes[existingIdx] = { ...client.nodes[existingIdx], ...nodeData }
    } else {
      client.nodes.push({ ...nodeData, status: 'stopped' })
    }
    // 更新缓存
    await window.api.setConfig(getCacheKey(clientId), JSON.parse(JSON.stringify(client.nodes))) // 确保去Proxy

    // 如果在线，发送给后端
    if (client.api && client.status === 'ready') {
      try {
        await client.api.post('/nodes', nodeData)
        await syncNodes(clientId) // 重新拉取以确认 ID 等字段
      } catch (e) {
        ElNotification({ title: '同步失败', message: '节点配置未能保存到机器人', type: 'warning' })
        throw e
      }
    }
  }

  // 3. 删除节点
  const deleteNodeConfig = async (clientId, nodeId) => {
    const client = clients[clientId]
    // 本地删除
    client.nodes = client.nodes.filter((n) => n.id !== nodeId)
    await window.api.setConfig(getCacheKey(clientId), JSON.parse(JSON.stringify(client.nodes)))

    // 远程删除
    if (client.api && client.status === 'ready') {
      try {
        await client.api.delete(`/nodes?id=${nodeId}`)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        ElNotification({ title: '同步失败', message: '未能从机器人删除节点', type: 'warning' })
      }
    }
  }

  // 4. 启动节点 (调用 /proc/start)
  const startNodeProcess = async (clientId, node) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    // Agent 要求的 Body: { id, cmd, args }
    // 我们使用配置里的 cmd 和 args
    const payload = {
      id: node.name, // 使用名称作为进程 ID (方便用户识别)
      cmd: node.cmd || 'roslaunch',
      args: node.args || []
    }

    // 乐观更新状态
    const target = client.nodes.find((n) => n.id === node.id)
    if (target) {
      target.status = 'starting'
      target._startTime = Date.now() // [新增] 记录点击启动的时间戳
    }

    // 立即初始化日志 Tab，确保 UI 有反应
    if (!client.nodeLogs) client.nodeLogs = {}
    if (!client.nodeLogs[node.name]) {
      client.nodeLogs[node.name] = {
        status: 'starting',
        lines: [`\x1b[1;33m[SYSTEM] Launching node: ${node.name}...\x1b[0m\r\n`]
      }
    } else {
      client.nodeLogs[node.name].status = 'starting'
      client.nodeLogs[node.name].lines.push(
        `\x1b[1;33m[SYSTEM] Relaunching node: ${node.name}...\x1b[0m\r\n`
      )
    }

    try {
      // [新增] 检查并自动启动 ROS 服务
      await ensureRosStackReady(client)
      // 服务就绪后，再启动节点
      await client.api.post('/proc/start', payload)
    } catch (e) {
      if (target) target.status = 'error'
      // 日志里也记一笔
      if (client.nodeLogs[node.name]) {
        client.nodeLogs[node.name].lines.push(
          `\x1b[1;31m[SYSTEM] Launch Failed: ${e.message}\x1b[0m\r\n`
        )
        client.nodeLogs[node.name].status = 'stopped'
      }
      ElMessage.error(`节点 ${node.name} 启动失败。`)
      throw e
    }
  }

  // 5. 停止节点 (调用 /proc/stop)
  const stopNodeProcess = async (clientId, nodeIdName) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    // [新增] 乐观更新：设为 stopping 并记录时间
    const target = client.nodes.find((n) => n.name === nodeIdName)
    if (target) {
      target.status = 'stopping'
      target._stopTime = Date.now() // 用于超时判断
    }

    await client.api.post('/proc/stop', { id: nodeIdName })
  }

  // 从缓存加载节点 (用于 Dashboard 初始化显示)，顺便加载序列
  const loadNodesFromCache = async (clientId) => {
    if (!clientId) return []
    // 1. 如果 Client 对象不存在，创建一个占位对象（离线模式）
    if (!clients[clientId]) {
      clients[clientId] = {
        id: clientId,
        status: 'disconnected', // 标记为离线
        nodes: [],
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' },
        nodeLogs: {},
        terminals: [],
        logWs: null,
        nextTerminalId: 1
      }
    } else {
      // 如果对象已存在（可能来自 addConnection），确保 nodeLogs 存在
      if (!clients[clientId].nodeLogs) clients[clientId].nodeLogs = {}
      if (!clients[clientId].terminals) clients[clientId].terminals = []
    }

    // 2. 顺便加载序列缓存
    try {
      const seqKey = getSequenceCacheKey(clientId)
      const cachedSeq = await window.api.getConfig(seqKey)
      if (cachedSeq && Array.isArray(cachedSeq)) {
        clients[clientId].sequences = cachedSeq
      } else {
        clients[clientId].sequences = []
      }
    } catch (e) {
      console.warn('Sequence cache load error', e)
    }

    // 3. 读取节点缓存
    try {
      const key = getCacheKey(clientId)
      const cached = await window.api.getConfig(key)
      if (cached && Array.isArray(cached)) {
        clients[clientId].nodes = cached
        return cached
      }
    } catch (e) {
      console.warn('Cache load error:', e)
    }

    return []
  }
  // 控制 ROS 服务栈 (Start/Stop)
  const controlRosStack = async (clientId, action) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    client.isStackLoading = true
    try {
      // [新增] 如果是停止操作，先显式杀掉所有正在运行的节点
      if (action === 'stop' && client.nodes) {
        const stopPromises = []

        client.nodes.forEach((n) => {
          // 针对所有运行中或启动中的节点
          if (n.status === 'running' || n.status === 'starting') {
            // 1. UI 立即反馈为 "stopping"
            n.status = 'stopping'
            n._stopTime = Date.now()

            // 2. 发送停止指令 (/proc/stop)
            // 我们不 await 单个请求，而是收集 Promise 并行发送，提高速度
            stopPromises.push(
              client.api
                .post('/proc/stop', { id: n.name })
                .catch((e) => console.warn(`Failed to stop node ${n.name}:`, e))
            )
          }
        })

        // 等待所有节点停止指令发出
        if (stopPromises.length > 0) {
          await Promise.all(stopPromises)
        }
      }

      // 发送 Stack 指令 (启动/停止 Roscore & Bridge)
      await client.api.post('/ros/action', { service: 'stack', action: action })

      // 轮询等待状态变更
      const targetState = action === 'start' ? 'active' : 'inactive'

      let retries = 0
      while (retries < 15) {
        // 15秒超时
        await sleep(1000)
        try {
          const res = await client.api.get('/proc/list')
          updateServiceStatusFromProcs(client, res.processes)

          const { roscore, bridge } = client.serviceStatus

          if (targetState === 'active') {
            if (roscore === 'active' && bridge === 'active') break
          } else {
            if (roscore !== 'active' && bridge !== 'active') break
          }
          // eslint-disable-next-line no-unused-vars
        } catch (e) {
          /* ignore */
        }
        retries++
      }
    } finally {
      client.isStackLoading = false
    }
  }

  // --- [新增] 序列管理 Actions ---

  const syncSequences = async (clientId) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    try {
      const res = await client.api.get('/sequences') // 调用后端
      if (Array.isArray(res)) {
        // 增加前端运行时状态 _status
        client.sequences = res.map((s) => ({ ...s, _status: 'stopped' }))
        window.api.setConfig(
          getSequenceCacheKey(clientId),
          JSON.parse(JSON.stringify(client.sequences))
        )
      }
    } catch (e) {
      console.warn('Failed to sync sequences:', e)
    }
  }

  // 1. 保存序列
  const saveSequenceConfig = async (clientId, seqData) => {
    const client = clients[clientId]
    // 1. 本地乐观更新 (保持界面流畅)
    if (!client.sequences) client.sequences = []
    const idx = client.sequences.findIndex((s) => s.id === seqData.id)
    // 注意：保存时去掉 _status 这种运行时字段
    const payload = { ...seqData }
    delete payload._status
    if (idx !== -1) client.sequences[idx] = { ...seqData, _status: 'stopped' }
    else client.sequences.push({ ...seqData, _status: 'stopped' })
    // 2. 发送给后端
    if (client.api && client.status === 'ready') {
      try {
        // 1. 移除 _status 等前端专用字段
        // 2. 确保 description 存在
        // 3. 确保 steps 中的 content 都是字符串类型
        const payload = {
          id: seqData.id,
          name: seqData.name,
          description: seqData.description || '', // 补全可选字段
          steps: seqData.steps.map((step) => ({
            type: step.type,
            content: String(step.content) // [关键] 强制转为字符串
          }))
        }

        await client.api.post('/sequences', payload)
        await syncSequences(clientId) // 重新同步
      } catch (e) {
        console.error('Save sequence failed:', e)
        // 从响应中提取错误信息显示
        const errorMsg = e.response?.data?.error || e.message
        ElNotification({ title: '保存失败', message: '无法保存序列: ' + errorMsg, type: 'error' })
      }
    }
  }

  // 2. 删除序列
  const deleteSequenceConfig = async (clientId, seqId) => {
    const client = clients[clientId]
    // 1. 本地删除
    if (client.sequences) {
      client.sequences = client.sequences.filter((s) => s.id !== seqId)
    }
    // 2. 远程删除
    if (client.api && client.status === 'ready') {
      try {
        await client.api.delete(`/sequences?id=${seqId}`)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        ElNotification({ title: '删除失败', message: '无法从机器人删除序列', type: 'error' })
      }
    }
  }

  // 3. 执行序列 (核心逻辑)
  const runSequence = async (clientId, sequence) => {
    const client = clients[clientId]
    if (!client || client.status !== 'ready') throw new Error('未连接')

    // 标记序列正在运行 (UI显示 loading)
    sequence._status = 'running'

    try {
      // 1. 确保 ROS 服务就绪
      // await ensureRosStackReady(client) // 可选：根据需求决定是否强制检查

      // 2. 遍历步骤
      for (const step of sequence.steps) {
        // 如果用户中途手动把状态改了(比如增加了停止按钮)，则中断
        if (sequence._status !== 'running') break

        if (step.type === 'node') {
          // 查找节点对象
          const node = client.nodes.find((n) => n.id === step.content)
          if (node) {
            // 启动节点 (复用已有 Action)
            // 注意：这里不 await startNodeProcess 的完成，
            // 因为 startNodeProcess 只是发指令，且我们有超时保护。
            // 但为了序列的稳定性，我们最好给一点点间隔
            await startNodeProcess(clientId, node)
          }
        } else if (step.type === 'delay') {
          // 延时
          const ms = parseInt(step.content) || 1000
          await sleep(ms)
        }
      }
    } catch (e) {
      console.error('Sequence execution error:', e)
      throw e
    } finally {
      sequence._status = 'stopped'
    }
  }

  // 4. 停止序列 (不仅仅是停止循环，还可以设计为停止序列里包含的所有节点)
  const stopSequenceNodes = async (clientId, sequence) => {
    const client = clients[clientId]
    sequence._status = 'stopped' // 中断循环标志

    // 停止所有关联节点
    const promises = []
    for (const step of sequence.steps) {
      if (step.type === 'node') {
        const node = client.nodes.find((n) => n.id === step.content)
        // 只有正在运行的才停
        if (node && (node.status === 'running' || node.status === 'starting')) {
          promises.push(stopNodeProcess(clientId, node.name))
        }
      }
    }
    await Promise.all(promises)
  }

  // --- 日志流管理 Actions ---

  const startLogStream = (clientId) => {
    const client = clients[clientId]
    if (!client || client.logWs) return

    // 构造 WS URL
    const wsUrl = `ws://${client.ip}:8080/ws/logs`
    console.log(`[LogStream] Connecting to ${wsUrl}`)

    const ws = new WebSocket(wsUrl)
    client.logWs = ws

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data)
        // msg 结构: { process_id, stream, data }
        handleLogMessage(client, msg)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        // 忽略非 JSON 消息或解析错误
      }
    }

    ws.onclose = () => {
      console.log(`[LogStream] Disconnected from ${clientId}`)
      client.logWs = null
      // 可以在这里做自动重连逻辑
    }
  }

  const handleLogMessage = (client, msg) => {
    const { process_id, stream, data } = msg

    // 1. 【修复】过滤掉全局 system 噪音
    // 如果你不希望看到 "system" 这个 Tab，直接忽略它
    if (!process_id || process_id === 'system') return

    // 初始化该节点的日志对象
    if (!client.nodeLogs[process_id]) {
      // 如果收到了日志但 Tab 不存在（可能是非手动启动的后台进程），
      // 可以选择自动创建，也可以选择忽略。这里保持自动创建以便观察隐式错误。
      client.nodeLogs[process_id] = { status: 'stopped', lines: [] }
    }
    const logObj = client.nodeLogs[process_id]

    // 2. 【修复】处理换行问题
    // 强制将 \n 转换为 \r\n，防止 xterm 出现阶梯状文本或不换行
    // 同时保留原始数据中的 \r\n (避免变成 \r\r\n)
    const formattedData = typeof data === 'string' ? data.replace(/\r?\n/g, '\r\n') : data

    // 处理系统事件 (System Stream)
    if (stream === 'system') {
      if (data.includes('Process started')) {
        logObj.status = 'running'
        logObj.lines.push(`\r\n\x1b[1;32m[SYSTEM] Node Started: ${process_id}\x1b[0m\r\n`)
      } else if (data.includes('Process exited')) {
        logObj.status = 'stopped'
        logObj.lines.push(`\r\n\x1b[1;31m[SYSTEM] Node Exited: ${process_id}\x1b[0m\r\n`)
      }
      return
    }

    // 处理标准输出/错误
    logObj.lines.push(formattedData)
  }

  // --- 终端管理 Actions ---

  const addTerminal = (clientId) => {
    const client = clients[clientId]
    if (!client) {
      console.error(`[Store] Client not found for addTerminal: ${clientId}`)
      return
    }

    // 确保 terminals 数组存在 (防止初始化漏掉)
    if (!client.terminals) client.terminals = []
    if (!client.nextTerminalId) client.nextTerminalId = 1

    const id = client.nextTerminalId++
    client.terminals.push({
      id: id,
      name: `终端 ${id}`
    })

    // console.log(`[Store] Added terminal ${id} to ${clientId}`)
    return id
  }

  const removeTerminal = (clientId, terminalId) => {
    const client = clients[clientId]
    if (!client || !client.terminals) return
    client.terminals = client.terminals.filter((t) => t.id !== terminalId)
  }

  const removeNodeLog = (clientId, nodeId) => {
    const client = clients[clientId]
    if (client && client.nodeLogs && client.nodeLogs[nodeId]) {
      delete client.nodeLogs[nodeId]
    }
  }

  // --- 文件系统 (File System) Actions ---

  // 1. 获取侧边栏 (Places)
  const fsGetSidebar = async (clientId) => {
    const client = clients[clientId]
    if (!client || !client.api) return { places: [], bookmarks: [] }

    try {
      const res = await client.api.get('/fs/places')
      const places = []
      const common = res.common || {}

      // 定义标准 XDG 目录映射
      const commonKeys = [
        { key: 'home', name: '主目录', icon: 'House' },
        { key: 'desktop', name: '桌面', icon: 'Monitor' },
        { key: 'documents', name: '文档', icon: 'Document' },
        { key: 'downloads', name: '下载', icon: 'Download' },
        { key: 'music', name: '音乐', icon: 'Headset' },
        { key: 'pictures', name: '图片', icon: 'Picture' },
        { key: 'videos', name: '视频', icon: 'Film' },
        { key: 'trash', name: '回收站', icon: 'Delete' }
      ]

      // 1. 构建常用位置
      commonKeys.forEach((item) => {
        if (common[item.key]) {
          places.push({
            name: item.name,
            path: common[item.key],
            icon: item.icon
          })
        }
      })

      // 兜底策略
      if (places.length === 0) {
        if (common.home) places.push({ name: '主目录', path: common.home, icon: 'House' })
        else places.push({ name: '根目录', path: '/', icon: 'Folder' })
      }

      // 2. 解析书签 (并去重)
      const rawBookmarks = Array.isArray(res.bookmarks) ? res.bookmarks : []
      const commonPaths = new Set(places.map((p) => p.path))
      // 过滤掉那些已经在 places 里出现过的书签
      const uniqueBookmarks = rawBookmarks.filter((b) => !commonPaths.has(b.path))

      const formattedBookmarks = uniqueBookmarks.map((b) => ({
        ...b,
        icon: b.icon || 'Star'
      }))

      return {
        places,
        bookmarks: formattedBookmarks
      }
    } catch (e) {
      console.error('Failed to get sidebar:', e)
      return { places: [], bookmarks: [] }
    }
  }

  // 2. 列出目录 (List)
  const fsListDir = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return []
    try {
      const res = await client.api.get('/fs/list', { params: { path: pathStr } })
      return Array.isArray(res) ? res : []
    } catch (e) {
      console.error('fsListDir error:', e)
      throw e
    }
  }

  // 3. 创建目录 (Mkdir)
  const fsMkdir = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/mkdir', { path: pathStr })
  }

  // 4. 读取文件内容
  const fsReadFile = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) throw new Error('连接未建立')

    // 这里的 responseType 默认为 json，但我们需要 text
    // 不过我们的 request.js 拦截器可能统一处理了 data
    // 建议显式请求 transformResponse 或在后端保证返回格式
    // 假设后端接口 /fs/read 返回 JSON: { is_binary: false, content: "..." }

    const res = await client.api.get('/fs/read', { params: { path: pathStr } })

    if (res.is_binary) {
      throw new Error('无法编辑二进制文件')
    }
    return res.content
  }

  // 5. 写文件
  const fsWriteFile = async (clientId, pathStr, content = '') => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/write', { path: pathStr, content })
  }

  // 6. 重命名/移动 (Rename/Move)
  const fsRename = async (clientId, src, dst) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/rename', { src, dst })
  }

  // 7. 复制 (Copy)
  const fsCopy = async (clientId, src, dst) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/copy', { src, dst })
  }

  // 8. 删除 (Delete/Trash)
  const fsDelete = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/delete', { path: pathStr })
  }

  // 9. 彻底删除
  const fsDeletePermanent = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/delete/permanent', { path: pathStr })
  }

  // 10. 还原回收站文件
  const fsRestore = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/restore', { path: pathStr })
  }
  // 11. 上传文件
  const fsUpload = async (clientId, { file, targetPath, onProgress }) => {
    const client = clients[clientId]
    if (!client || !client.api) throw new Error('Client not connected')

    const formData = new FormData()
    // Agent 要求字段: 'file' (二进制流), 'path' (远程绝对路径)
    formData.append('file', file)
    formData.append('path', targetPath)

    try {
      await client.api.post('/fs/upload', formData, {
        // Axios 原生支持上传进度回调
        onUploadProgress: (progressEvent) => {
          if (onProgress && progressEvent.total) {
            const percent = Math.round((progressEvent.loaded * 100) / progressEvent.total)
            onProgress(percent)
          }
        },
        // 上传时间较长，覆盖默认超时设置
        timeout: 0 // 0 表示无超时
      })
    } catch (e) {
      console.error('Upload failed:', e)
      throw e
    }
  }
  // 12. 智能粘贴 (支持同机/跨机)
  const fsPaste = async (targetId, targetDir) => {
    if (!clipboard.value) return

    const { sourceId, path: srcPath, mode, name, isDir } = clipboard.value
    const targetPath = `${targetDir}/${name}`.replace(/\/+/g, '/') // 简单拼接

    // 情况 1: 同机操作 (直接调用 Agent 的复制/移动接口)
    if (sourceId === targetId) {
      if (mode === 'copy') {
        await fsCopy(targetId, srcPath, targetPath)
      } else if (mode === 'cut') {
        await fsRename(targetId, srcPath, targetPath)
        clipboard.value = null // 剪切后清空
      }
      return
    }

    // 情况 2: 跨机操作 (Client 中转: Download -> Upload)
    // 注意：目前暂不支持跨机剪切(移动)，强制视为复制，或者是复制成功后再删除源文件
    if (isDir) {
      throw new Error('暂不支持跨机器人复制文件夹，请先压缩为文件。')
    }

    const srcClient = clients[sourceId]
    const targetClient = clients[targetId]
    if (!srcClient || !targetClient) throw new Error('源或目标机器已断开')

    // 1. 下载 (获取 Blob)
    // 注意：对于超大文件，这可能会占用前端内存。理想情况是在 Main 进程做 Stream Pipe，
    // 但基于目前的 Axios 架构，Blob 是最快实现方式。
    const downloadRes = await srcClient.api.get('/fs/download', {
      params: { path: srcPath },
      responseType: 'blob', // 关键
      timeout: 0 // 下载大文件不超时
    })

    // 2. 上传
    const formData = new FormData()
    // 创建一个新的 File 对象
    const fileObj = new File([downloadRes], name)
    formData.append('file', fileObj)
    formData.append('path', targetPath)

    await targetClient.api.post('/fs/upload', formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
      timeout: 0
    })

    // 3. 如果是剪切模式，且跨机传输成功，则删除源文件
    if (mode === 'cut') {
      await fsDelete(sourceId, srcPath) // 移入回收站比较安全
      clipboard.value = null
    }
  }

  return {
    clients,
    activeID,
    activeClient,
    sudoPassword,
    clipboard,
    initClientPlaceholder,
    setClipboard,
    setSudoPassword,
    addConnection,
    removeConnection,
    refreshStatus,
    setClientStatus,
    syncNodes,
    saveNodeConfig,
    deleteNodeConfig,
    startNodeProcess,
    stopNodeProcess,
    loadNodesFromCache,
    controlRosStack,
    saveSequenceConfig,
    deleteSequenceConfig,
    runSequence,
    stopSequenceNodes,
    addTerminal,
    removeTerminal,
    removeNodeLog,
    fsGetSidebar,
    fsListDir,
    fsMkdir,
    fsReadFile,
    fsWriteFile,
    fsRename,
    fsCopy,
    fsDelete,
    fsDeletePermanent,
    fsRestore,
    fsUpload,
    fsPaste
  }
})
