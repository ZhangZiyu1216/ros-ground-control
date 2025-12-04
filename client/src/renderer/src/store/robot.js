/* eslint-disable no-unused-vars */
import { defineStore } from 'pinia'
import { ref, computed, reactive } from 'vue'
import { createApi } from '../api/request'
import { ElMessage, ElNotification } from 'element-plus'

// --- 配置常量 ---
const CONFIG = {
  HEALTH_CHECK_INTERVAL: 2000,
  MAX_MISSED_HEARTBEATS: 3,
  AUTO_DISCONNECT_THRESHOLD: 10,
  HANDSHAKE_TIMEOUT: 5000,
  MAX_CONNECT_RETRIES: 1
}

export const useRobotStore = defineStore('robot', () => {
  // ==================================================================================
  // Region 1: State Definition
  // ==================================================================================

  const clients = reactive({}) // Key: UUID (优先) 或 IP
  const activeID = ref(null)
  const sudoPassword = ref('')

  // 剪贴板结构: { sourceId, path, mode: 'copy'|'cut', name, isDir }
  const clipboard = ref(null)
  const hostSystemState = reactive({
    info: {
      hostname: 'Loading...', // 初始占位符
      platform: '',
      arch: '',
      cpuModel: '',
      cpuCores: 0,
      cpuSpeed: 0,
      totalmem: 0,
      freemem: 0,
      uptime: 0,
      networks: []
    },
    cpuUsage: 0
  })
  const updateHostSystemState = async () => {
    try {
      // 调用 IPC
      const data = await window.api.getHostInfo()
      // 更新 State
      // 注意：这里要做深度合并或者属性赋值，保持响应性
      Object.assign(hostSystemState.info, data)
      hostSystemState.cpuUsage = data.cpuUsage || 0
    } catch (e) {
      console.error('[Store] Failed to update host info:', e)
    }
  }

  // === Getters ===
  const activeClient = computed(() => (activeID.value ? clients[activeID.value] : null))

  // === Helpers ===
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms))
  const getCacheKey = (id) => `robot_nodes_${id}`
  const getSeqCacheKey = (id) => `robot_seq_${id}`
  const setSudoPassword = (pwd) => {
    sudoPassword.value = pwd
  }
  const setClipboard = (data) => {
    clipboard.value = data
  }

  // 初始化占位符
  const initClientPlaceholder = (settings) => {
    // 优先使用 ID 作为 Key，其次是 IP
    const key = settings.id || settings.ip
    if (!clients[key]) {
      clients[key] = createClientObject(settings)
    } else {
      // 如果已存在，仅更新状态
      clients[key].status = 'setting_up'
    }
    // 如果没有活跃ID，设为当前
    if (!activeID.value) activeID.value = key
  }

  // 创建标准对象
  const createClientObject = (settings) => ({
    id: settings.id || null,
    ip: settings.ip,
    port: settings.port || 8080,
    hostname: settings.hostname,
    name: settings.name || settings.hostname,
    status: 'setting_up',

    token: null,
    logWs: null,
    api: null,
    timer: null,
    missedHeartbeats: 0,
    ipChanged: false,

    sysInfo: {
      hostname: settings.hostname || 'Unknown',
      os: '-',
      arch: '-',
      num_cpu: 0, // UI 需要读取这个
      id: null // 唯一标识
    },
    stats: {
      cpu_usage: 0,
      mem_usage: 0,
      net_rx_rate: 0,
      net_tx_rate: 0,
      temperature: 0,
      disk_usage: 0
    },
    serviceStatus: { roscore: 'unknown', bridge: 'unknown' },
    nodes: [],
    nodeLogs: {},
    sequences: [],
    terminals: [],
    nextTerminalId: 1,
    recordingId: null,

    playbackId: null, // 后端返回的播放进程 ID
    playbackStatus: 'stopped', // 'stopped' | 'playing' | 'paused'

    isStackLoading: false
  })

  // 供 MainView 调用的离线初始化方法
  const initOfflineClient = (settings) => {
    const key = settings.id || settings.ip
    if (!clients[key]) {
      const client = createClientObject(settings)
      client.status = 'disconnected' // 显式设为离线
      clients[key] = client
    }
    // 如果没有活跃ID，设为当前
    if (!activeID.value) activeID.value = key
  }

  // ==================================================================================
  // Region 2: Connection & Handshake (Core)
  // ==================================================================================

  // 握手逻辑 (WebSocket Auth)
  const handshakeAndConnect = (ip, port) => {
    return new Promise((resolve, reject) => {
      const targetPort = port || 8080
      const wsUrl = `ws://${ip}:${port}/ws/logs`
      console.log(`[Connect] Initiating handshake: ${wsUrl}`)

      const ws = new WebSocket(wsUrl)
      // 状态标志位：防止 timeout 和 onmessage/onclose 竞态
      let isFinished = false
      let handshakeTimeout = null

      // 清理函数
      const cleanup = () => {
        if (handshakeTimeout) clearTimeout(handshakeTimeout)
        isFinished = true
      }

      handshakeTimeout = setTimeout(() => {
        if (isFinished) return
        cleanup()
        console.warn('[Connect] Handshake timeout')
        ws.close() // 这会触发 onclose，我们需要在 onclose 里判断 isFinished
        reject(new Error('连接超时: 握手未完成'))
      }, CONFIG.HANDSHAKE_TIMEOUT)

      ws.onopen = () => {
        console.log('[Connect] WS Open')
      }

      ws.onmessage = (event) => {
        if (isFinished) return // 如果已经超时或已结束，忽略后续消息

        try {
          const msg = JSON.parse(event.data)
          if (msg.stream === 'auth') {
            cleanup() // 标记结束并清除定时器
            const token = msg.data

            // 创建 API
            const api = createApi(`http://${ip}:${targetPort}/api`, {
              getToken: () => token,
              getSudoPassword: () => sudoPassword.value,
              setSudoPassword: (p) => (sudoPassword.value = p),
              onSessionExpired: () => {
                console.warn('Session Expired or Conflict')
                ws.close()
              }
            })

            resolve({ ws, token, api })
          }
        } catch (e) {
          // 忽略非 Auth 消息或解析错误，继续等待
        }
      }

      ws.onerror = () => {}
      ws.onclose = (e) => {
        if (isFinished) return // 如果是因为超时我们主动 close 的，忽略
        cleanup()
        console.warn(`[Connect] WS Closed: code=${e.code}, reason=${e.reason}`)
        if (e.code === 4009 || e.reason === 'Conflict' || e.code === 1006) {
          reject(new Error('连接被拒绝: 设备繁忙(Conflict)或服务未启动'))
        } else {
          reject(new Error(`连接断开 (Code: ${e.code})`))
        }
      }
    })
  }

  const handleWsMessage = (client, event) => {
    try {
      // 第一次解析：解包信封 { stream, data, ... }
      const msg = JSON.parse(event.data)

      // 分流处理
      if (msg.stream === 'system-stats') {
        // [关键步骤] 二次解析：后端说 data 是字符串，必须再次 JSON.parse
        let realData = msg.data
        if (typeof realData === 'string') {
          try {
            realData = JSON.parse(realData)
          } catch (e) {
            console.warn('Failed to parse inner system-stats JSON', e)
            return
          }
        }
        // 更新 Store
        client.stats = realData
      } else {
        // 其他日志流 (auth, system, node logs)
        handleLogMessage(client, msg)
      }
    } catch (e) {
      // ignore invalid json
    }
  }

  // 添加连接 (Connection Entry)
  const addConnection = async (settings) => {
    let currentKey = settings.id || settings.ip
    const targetIp = settings.ip
    const targetPort = settings.port || 8080

    if (!clients[currentKey]) clients[currentKey] = createClientObject(settings)
    const client = clients[currentKey]
    client.status = 'setting_up'

    // 如果是复用现有对象，更新一下 port (如果是手动输入的)
    if (clients[currentKey]) {
      clients[currentKey].port = targetPort
    }

    // 清理旧资源
    if (client.logWs) client.logWs.close()
    if (client.timer) clearInterval(client.timer)

    loadNodesFromCache(currentKey)
    activeID.value = currentKey

    let finalRes = undefined
    let finalIp = targetIp
    let finalPort = targetPort

    try {
      // 1. 尝试直连
      try {
        finalRes = await handshakeAndConnect(targetIp, targetPort)
      } catch (e) {
        // 2. 尝试 mDNS 找回
        if (settings.hostname) {
          console.log(`[Connect] Retry with mDNS: ${settings.hostname}`)
          const discovery = await findNewIpByHostname(settings.hostname)
          if (discovery && discovery.ip) {
            console.log(`[Connect] mDNS resolved: ${discovery.ip}:${discovery.port}`)
            finalRes = await handshakeAndConnect(discovery.ip, discovery.port)
            finalIp = discovery.ip
            finalPort = discovery.port // 更新端口变量
            client.ipChanged = true
          } else {
            throw e
          }
        } else {
          throw e
        }
      }

      if (!finalRes) throw new Error('握手失败')

      // 3. 验证并获取 ID
      const sysInfo = await finalRes.api.get('/sys/info')
      const procList = await finalRes.api.get('/proc/list')
      const realId = sysInfo.id

      // 4. 更新数据
      client.api = finalRes.api
      client.token = finalRes.token
      client.logWs = finalRes.ws
      client.ip = finalIp
      client.port = finalPort
      client.id = realId
      client.sysInfo = sysInfo
      client.status = 'ready'

      // 绑定消息处理
      client.logWs.onmessage = (event) => handleWsMessage(client, event)
      client.logWs.onclose = () => {
        client.status = 'disconnected'
      }

      // 5. Key 迁移 (IP -> UUID)
      let activeKey = currentKey
      if (realId && currentKey !== realId) {
        if (clients[realId]) {
          // 合并
          Object.assign(clients[realId], {
            api: client.api,
            token: client.token,
            logWs: client.logWs,
            ip: client.ip,
            port: finalPort,
            status: 'ready',
            sysInfo: sysInfo,
            ipChanged: client.ipChanged
          })
          clients[realId].logWs.onclose = () => {
            clients[realId].status = 'disconnected'
          }
          clients[realId].logWs.onmessage = (event) => handleWsMessage(clients[realId], event)
          delete clients[currentKey]
          activeKey = realId
        } else {
          // 改名
          clients[realId] = client
          delete clients[currentKey]
          activeKey = realId
        }
      }

      activeID.value = activeKey
      const finalClient = clients[activeKey]

      await Promise.all([syncNodes(activeKey), syncSequences(activeKey)])

      if (procList && procList.processes) {
        updateServiceStatusFromProcs(finalClient, procList.processes)
      }

      startHealthCheck(activeKey)
      broadcastClientState(clients[activeKey])
    } catch (e) {
      console.error('[Connect] Fatal:', e)
      client.status = 'failed'
      broadcastClientState(client) // 失败状态也广播
      if (e.message.includes('繁忙') || e.message.includes('Conflict')) {
        ElNotification({ title: '连接被拒绝', message: '设备已被占用', type: 'warning' })
      } else {
        ElNotification({ title: '连接失败', message: e.message, type: 'error' })
      }
      throw e
    }
  }

  const removeConnection = (id) => {
    const client = clients[id]
    if (!client) return
    if (client.logWs) client.logWs.close()
    if (client.timer) clearInterval(client.timer)
    client.status = 'disconnected'
    // 清理录制状态
    client.recordingId = null
    broadcastClientState(client)
  }

  // [恢复功能] 手动设置状态
  const setClientStatus = (id, status) => {
    if (clients[id]) {
      clients[id].status = status
      if (status === 'disconnected' && clients[id].timer) {
        clearInterval(clients[id].timer)
        clients[id].timer = null
      }
      broadcastClientState(clients[id])
    }
  }

  // [恢复功能] 手动刷新状态
  const refreshStatus = async (id) => {
    const client = clients[id]
    if (client && client.api && client.status === 'ready') {
      try {
        const res = await client.api.get('/proc/list')
        updateServiceStatusFromProcs(client, res.processes)
      } catch (e) {
        /* ignore */
      }
    }
  }

  // ==================================================================================
  // Region 3: Heartbeat & Discovery
  // ==================================================================================

  const startHealthCheck = (id) => {
    const client = clients[id]
    if (!client) return
    if (client.timer) clearInterval(client.timer)
    client.missedHeartbeats = 0

    client.timer = setInterval(async () => {
      if (!clients[id] || client.status === 'disconnected') {
        clearInterval(client.timer)
        return
      }
      // WS 断开视为连接失败
      if (client.logWs && client.logWs.readyState !== WebSocket.OPEN) {
        client.status = 'failed'
      }

      try {
        const res = await client.api.get('/proc/list')

        // 成功回调回来时，如果状态已经被置为 disconnected (可能被用户手动断开)，则不处理
        if (client.status === 'disconnected') return

        if (client.status === 'failed') {
          client.status = 'ready'
          ElNotification({ title: '连接恢复', message: `${client.name} 已重连`, type: 'success' })
          broadcastClientState(client) // 状态变化广播
        }
        client.missedHeartbeats = 0
        updateServiceStatusFromProcs(client, res.processes)
      } catch (e) {
        // 失败回调回来时，如果状态已经是 disconnected，说明已经被处理过（被之前的超时或手动断开）
        // 直接返回，防止重复弹窗
        if (client.status === 'disconnected') return

        client.missedHeartbeats++
        // 1. 变红
        if (client.missedHeartbeats >= CONFIG.MAX_MISSED_HEARTBEATS && client.status === 'ready') {
          client.status = 'failed'
          client.serviceStatus = { roscore: 'unknown', bridge: 'unknown' }
          broadcastClientState(client) // 状态变化广播
        }
        // 2. 尝试 mDNS 找回
        if (client.status === 'failed' && client.hostname && client.missedHeartbeats % 4 === 0) {
          attemptAutoRecovery(client)
        }
        // 3. 彻底断开
        if (client.missedHeartbeats >= CONFIG.AUTO_DISCONNECT_THRESHOLD) {
          // 在执行断开操作前，再次确认状态（虽然上面判断过，但由上至下逻辑更清晰）
          if (client.status !== 'disconnected') {
            removeConnection(id)
            ElNotification({ title: '连接断开', message: `${client.name} 连接超时`, type: 'info' })
          }
        }
      }
    }, CONFIG.HEALTH_CHECK_INTERVAL)
  }

  const attemptAutoRecovery = async (client) => {
    const res = await findNewIpByHostname(client.hostname)
    if (res && res.ip && res.ip !== client.ip) {
      try {
        const { ws, token, api } = await handshakeAndConnect(res.ip, client.port)
        if (client.logWs) client.logWs.close()
        client.ip = res.ip
        client.port = res.port
        client.api = api
        client.token = token
        client.logWs = ws
        client.ipChanged = true
        client.status = 'ready'
        client.missedHeartbeats = 0

        client.logWs.onmessage = (event) => handleLogMessage(client, JSON.parse(event.data))
        client.logWs.onclose = () => {
          client.status = 'disconnected'
        }
        broadcastClientState(client) // 状态变化广播
      } catch (e) {
        /* ignore */
      }
    }
  }

  const findNewIpByHostname = (targetHostname) => {
    return new Promise((resolve) => {
      const cleanTarget = targetHostname.toLowerCase().replace('.local', '')
      let result = null
      let cleanup = null
      const onFound = (service) => {
        const h = service.txt?.hostname || service.host || ''
        if (h.toLowerCase().replace('.local', '') === cleanTarget) {
          const ip = service.addresses?.find((a) => a.includes('.'))
          if (ip) result = { ip, id: service.txt?.id, port: service.port }
        }
      }
      try {
        cleanup = window.api.onMdnsServiceFound(onFound)
        window.api.startMdnsScan()
      } catch (e) {
        resolve(null)
        return
      }
      setTimeout(() => {
        window.api.stopMdnsScan()
        if (cleanup) cleanup()
        resolve(result)
      }, 2500)
    })
  }

  // ==================================================================================
  // Region 4: Process, Logs & Stack
  // ==================================================================================

  const updateServiceStatusFromProcs = (client, processes) => {
    if (!Array.isArray(processes)) return

    // 1. 基础服务状态
    const hasCore = processes.some((p) => p.id.includes('roscore') || p.cmd.includes('roscore'))
    const hasBridge = processes.some(
      (p) => p.id.includes('foxglove') || p.cmd.includes('rosbridge')
    )
    client.serviceStatus.roscore = hasCore ? 'active' : 'inactive'
    client.serviceStatus.bridge = hasBridge ? 'active' : 'inactive'

    // 2. 节点状态同步 (保持不变)
    if (client.nodes) {
      client.nodes.forEach((node) => {
        const proc = processes.find((p) => p.id === node.name)
        const now = Date.now()
        if (proc) {
          if (node.status === 'stopping' && node._stopTime && now - node._stopTime < 8000) {
            /* wait */
          } else {
            node.status = 'running'
            node._startTime = null
          }
        } else {
          if (node.status === 'starting' && node._startTime && now - node._startTime < 5000) {
            /* wait */
          } else {
            node.status = 'stopped'
          }
        }
      })
    }

    // 录制状态的同步与认领 (Adoption)
    // A. 检测当前 ID 是否存活
    if (client.recordingId) {
      if (!processes.some((p) => p.id === client.recordingId)) {
        client.recordingId = null
        ElNotification({ title: '录制结束', message: 'Rosbag 录制已停止', type: 'info' })
      }
    }
    // B. [新增] 自动认领：如果本地没记录，但远程有 bag 进程，认领它！
    else {
      // 查找 cmd 包含 'rosbag record' 的进程
      const activeBag = processes.find(
        (p) => p.id.startsWith('bag-') || p.cmd.includes('rosbag record')
      )
      if (activeBag) {
        console.log('[Sync] Adopted existing recording process:', activeBag.id)
        client.recordingId = activeBag.id
      }
    }

    // 回放状态的同步与认领
    if (client.playbackId) {
      if (!processes.some((p) => p.id === client.playbackId)) {
        client.playbackId = null
        client.playbackStatus = 'stopped'
        ElNotification({ title: '回放结束', message: '回放进程已退出', type: 'info' })
      }
    }
    // B. [新增] 自动认领回放
    else {
      const activePlay = processes.find(
        (p) => p.id.startsWith('play-') || p.cmd.includes('rosbag play')
      )
      if (activePlay) {
        console.log('[Sync] Adopted existing playback process:', activePlay.id)
        client.playbackId = activePlay.id
        // 认领时我们不知道是 paused 还是 playing，暂定 playing，后续可通过心跳优化
        client.playbackStatus = 'playing'
      }
    }
  }

  // [新增] 处理系统监控数据流
  const handleSystemStats = (client, data) => {
    if (!client || !data) return
    // 直接替换 stats 对象，触发 Vue 响应式更新
    client.stats = data
  }

  // [修复] 日志处理：保留对 system stream 的解析以显示节点生命周期
  const handleLogMessage = (client, msg) => {
    if (!msg || typeof msg !== 'object') return

    const { process_id, stream, data } = msg
    if (
      !process_id ||
      process_id.startsWith('sys-') || // 系统进程日志忽略
      process_id.startsWith('auto-comp-') || // 图像话题压缩日志忽略
      process_id.startsWith('play-') || // 播放ROS包日志忽略
      process_id.startsWith('bag-') // 录制ROS包日志忽略
    )
      return

    if (!client.nodeLogs[process_id]) {
      if (process_id === 'system') return
      client.nodeLogs[process_id] = { status: 'stopped', lines: [] }
    }
    const logObj = client.nodeLogs[process_id]
    const formatted = typeof data === 'string' ? data.replace(/\r?\n/g, '\r\n') : data

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
    logObj.lines.push(formatted)
    if (logObj.lines.length > 2000) logObj.lines.shift()
  }

  const startNodeProcess = async (clientId, node) => {
    const client = clients[clientId]
    if (!client?.api) return
    const target = client.nodes.find((n) => n.id === node.id)
    if (target) {
      target.status = 'starting'
      target._startTime = Date.now()
    }

    if (!client.nodeLogs) client.nodeLogs = {}

    if (!client.nodeLogs[node.name]) client.nodeLogs[node.name] = { status: 'starting', lines: [] }
    client.nodeLogs[node.name].lines.push(
      `\r\n\x1b[1;33m[SYS] Launching ${node.name}...\x1b[0m\r\n`
    )

    try {
      await ensureRosStackReady(client)
      await client.api.post('/proc/start', {
        id: node.name,
        cmd: node.cmd || 'roslaunch',
        args: node.args || []
      })
    } catch (e) {
      if (target) target.status = 'error'
      if (client.nodeLogs && client.nodeLogs[node.name]) {
        client.nodeLogs[node.name].lines.push(`\r\n\x1b[1;31m[SYS] Failed: ${e.message}\x1b[0m\r\n`)
      }
      throw e
    }
  }

  const stopNodeProcess = async (clientId, nodeIdName) => {
    const client = clients[clientId]
    if (!client?.api) return
    const target = client.nodes.find((n) => n.name === nodeIdName)
    if (target) {
      target.status = 'stopping'
      target._stopTime = Date.now()
    }
    await client.api.post('/proc/stop', { id: nodeIdName })
  }

  const ensureRosStackReady = async (client) => {
    if (client.serviceStatus.roscore === 'active' && client.serviceStatus.bridge === 'active')
      return
    client.isStackLoading = true
    try {
      await client.api.post('/ros/action', { service: 'stack', action: 'start' })
      let retry = 0
      while (retry < 15) {
        await sleep(1000)
        const res = await client.api.get('/proc/list')
        updateServiceStatusFromProcs(client, res.processes)
        if (client.serviceStatus.roscore === 'active') return
        retry++
      }
      throw new Error('ROS Stack start timeout')
    } finally {
      client.isStackLoading = false
    }
  }

  const controlRosStack = async (clientId, action) => {
    const client = clients[clientId]
    if (!client?.api) return

    client.isStackLoading = true
    try {
      // [防御策略 1] 如果是停止操作，必须先优雅停止依赖进程
      if (action === 'stop') {
        const stopPromises = []

        // A. 停止录制 (至关重要：防止文件损坏)
        if (client.recordingId) {
          console.log('[AutoStop] Stopping active recording to save data...')
          // 不放入 Promise.all，而是 await，确保文件 flush 完成
          try {
            await bagStop(clientId, client.recordingId)
            ElNotification({
              title: '录制已保存',
              message: '在停止服务前已自动结束录制并保存数据。',
              type: 'success'
            })
          } catch (e) {
            console.warn('[AutoStop] Failed to stop recording gracefully:', e)
          }
        }

        // B. 停止回放
        if (client.playbackId) {
          console.log('[AutoStop] Stopping active playback...')
          stopPromises.push(
            bagPlayStop(clientId).catch((e) => console.warn('Stop playback failed', e))
          )
        }

        // C. 停止所有普通节点
        if (client.nodes) {
          client.nodes.forEach((n) => {
            if (n.status === 'running' || n.status === 'starting') {
              n.status = 'stopping'
              n._stopTime = Date.now()
              stopPromises.push(stopNodeProcess(clientId, n.name).catch(() => {}))
            }
          })
        }

        // 并行等待回放和节点停止 (录制已在上面单独 await)
        if (stopPromises.length > 0) {
          await Promise.all(stopPromises)
        }
      }

      // 发送 Stack 指令
      await client.api.post('/ros/action', { service: 'stack', action: action })

      // 轮询等待状态变更 (保持原逻辑)
      const targetState = action === 'start' ? 'active' : 'inactive'
      let retries = 0
      while (retries < 15) {
        await sleep(1000)
        try {
          const res = await client.api.get('/proc/list')
          updateServiceStatusFromProcs(client, res.processes) // 状态更新全靠它
          const { roscore, bridge } = client.serviceStatus

          if (targetState === 'active') {
            if (roscore === 'active' && bridge === 'active') break
          } else {
            if (roscore !== 'active' && bridge !== 'active') break
          }
        } catch (e) {
          /* ignore */
        }
        retries++
      }
    } finally {
      client.isStackLoading = false
    }
  }

  // ==================================================================================
  // Region 5: Configs (Nodes & Sequences)
  // ==================================================================================

  const syncNodes = async (id) => {
    const client = clients[id]
    if (!client?.api) return
    try {
      const res = await client.api.get('/nodes')
      client.nodes = (res || []).map((n) => ({ ...n, status: 'stopped' }))
      window.api.setConfig(getCacheKey(id), JSON.parse(JSON.stringify(client.nodes)))
    } catch (e) {
      /* ignore */
    }
  }

  const saveNodeConfig = async (id, nodeData) => {
    const client = clients[id]
    if (!client) return
    const idx = client.nodes.findIndex((n) => n.id === nodeData.id)
    if (idx !== -1) client.nodes[idx] = { ...client.nodes[idx], ...nodeData }
    else client.nodes.push({ ...nodeData, status: 'stopped' })

    window.api.setConfig(getCacheKey(id), JSON.parse(JSON.stringify(client.nodes)))
    if (client.status === 'ready') {
      await client.api.post('/nodes', nodeData)
      await syncNodes(id)
    }
  }

  const deleteNodeConfig = async (id, nodeId) => {
    const client = clients[id]
    client.nodes = client.nodes.filter((n) => n.id !== nodeId)
    window.api.setConfig(getCacheKey(id), JSON.parse(JSON.stringify(client.nodes)))
    if (client.status === 'ready') await client.api.delete(`/nodes?id=${nodeId}`)
  }

  const loadNodesFromCache = async (id) => {
    try {
      const nodes = await window.api.getConfig(getCacheKey(id))
      if (clients[id] && Array.isArray(nodes)) clients[id].nodes = nodes
      const seqs = await window.api.getConfig(getSeqCacheKey(id))
      if (clients[id] && Array.isArray(seqs)) clients[id].sequences = seqs
    } catch (e) {
      /* ignore */
    }
  }

  const syncSequences = async (id) => {
    const client = clients[id]
    if (!client?.api) return
    try {
      const res = await client.api.get('/sequences')
      client.sequences = (res || []).map((s) => ({ ...s, _status: 'stopped' }))
      window.api.setConfig(getSeqCacheKey(id), JSON.parse(JSON.stringify(client.sequences)))
    } catch (e) {
      /* ignore */
    }
  }

  const saveSequenceConfig = async (id, seqData) => {
    const client = clients[id]
    const idx = client.sequences.findIndex((s) => s.id === seqData.id)
    const uiData = { ...seqData, _status: 'stopped' }
    if (idx !== -1) client.sequences[idx] = uiData
    else client.sequences.push(uiData)

    if (client.status === 'ready') {
      const payload = {
        ...seqData,
        steps: seqData.steps.map((s) => ({ type: s.type, content: String(s.content) }))
      }
      delete payload._status
      await client.api.post('/sequences', payload)
      syncSequences(id)
    }
  }

  const deleteSequenceConfig = async (id, seqId) => {
    const client = clients[id]
    client.sequences = client.sequences.filter((s) => s.id !== seqId)
    if (client.status === 'ready') await client.api.delete(`/sequences?id=${seqId}`)
  }

  const runSequence = async (id, seq) => {
    const client = clients[id]
    if (client.status !== 'ready') throw new Error('Not connected')
    seq._status = 'running'
    try {
      for (const step of seq.steps) {
        if (seq._status !== 'running') break
        if (step.type === 'node') {
          const node = client.nodes.find((n) => n.id === step.content)
          if (node) await startNodeProcess(id, node)
        } else if (step.type === 'delay') {
          await sleep(parseInt(step.content) || 1000)
        }
      }
    } finally {
      seq._status = 'stopped'
    }
  }

  const stopSequenceNodes = async (id, seq) => {
    const client = clients[id]
    seq._status = 'stopped'
    const promises = seq.steps
      .filter((s) => s.type === 'node')
      .map((s) => {
        const node = client.nodes.find((n) => n.id === s.content)
        if (node && (node.status === 'running' || node.status === 'starting')) {
          return stopNodeProcess(id, node.name)
        }
      })
    await Promise.all(promises)
  }

  // ==================================================================================
  // Region 6: File System & Rosbag
  // ==================================================================================

  // [恢复功能] 侧边栏
  const fsGetSidebar = async (id) => {
    const client = clients[id]
    if (!client?.api) return { places: [], bookmarks: [] }

    try {
      const res = await client.api.get('/fs/places')
      const places = []
      const common = res.common || {}

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

      commonKeys.forEach((item) => {
        if (common[item.key]) {
          places.push({ name: item.name, path: common[item.key], icon: item.icon })
        }
      })

      if (places.length === 0) {
        if (common.home) places.push({ name: '主目录', path: common.home, icon: 'House' })
        else places.push({ name: '根目录', path: '/', icon: 'Folder' })
      }

      // 去重书签
      const rawBk = Array.isArray(res.bookmarks) ? res.bookmarks : []
      const placePaths = new Set(places.map((p) => p.path))
      const bookmarks = rawBk
        .filter((b) => !placePaths.has(b.path))
        .map((b) => ({ ...b, icon: b.icon || 'Star' }))

      return { places, bookmarks }
    } catch (e) {
      console.error('[FS] Get Sidebar Failed:', e)
      return { places: [], bookmarks: [] }
    }
  }

  const fsListDir = (id, path) => clients[id]?.api.get('/fs/list', { params: { path } })
  const fsMkdir = (id, path) => clients[id]?.api.post('/fs/mkdir', { path })

  const fsReadFile = async (id, path) => {
    const res = await clients[id]?.api.get('/fs/read', { params: { path } })
    if (res.is_binary) throw new Error('Binary file')
    return res.content
  }

  const fsWriteFile = (id, path, content) => clients[id]?.api.post('/fs/write', { path, content })
  const fsRename = (id, src, dst) => clients[id]?.api.post('/fs/rename', { src, dst })
  const fsCopy = (id, src, dst) => clients[id]?.api.post('/fs/copy', { src, dst })
  const fsDelete = (id, path) => clients[id]?.api.post('/fs/delete', { path })
  // [恢复] 彻底删除/还原
  const fsDeletePermanent = (id, path) => clients[id]?.api.post('/fs/delete/permanent', { path })
  const fsRestore = (id, path) => clients[id]?.api.post('/fs/restore', { path })

  const fsUpload = (id, { file, targetPath, onProgress }) => {
    const fd = new FormData()
    fd.append('file', file)
    fd.append('path', targetPath)
    return clients[id]?.api.post('/fs/upload', fd, {
      timeout: 0,
      onUploadProgress: (e) =>
        onProgress && e.total && onProgress(Math.round((e.loaded * 100) / e.total))
    })
  }

  // 辅助：下载 blob
  const fsDownloadBlob = async (id, path) => {
    const client = clients[id]
    if (!client) throw new Error('Client not connected')
    return await client.api.get('/fs/download', {
      params: { path },
      responseType: 'blob',
      timeout: 0
    })
  }

  // [恢复] 粘贴 (同机/跨机)
  const fsPaste = async (targetId, targetDir) => {
    if (!clipboard.value) return
    const { sourceId, path: srcPath, mode, name, isDir } = clipboard.value
    const targetPath = `${targetDir}/${name}`.replace(/\/+/g, '/')

    // Case 1: 同机
    if (sourceId === targetId) {
      if (mode === 'copy') await fsCopy(targetId, srcPath, targetPath)
      else if (mode === 'cut') {
        await fsRename(targetId, srcPath, targetPath)
        clipboard.value = null
      }
      return
    }

    // Case 2: 跨机 (Client 中转)
    if (isDir) throw new Error('暂不支持跨机器人复制文件夹')

    // Step A: Download
    const blob = await fsDownloadBlob(sourceId, srcPath)

    // Step B: Upload
    const fileObj = new File([blob], name)
    const fd = new FormData()
    fd.append('file', fileObj)
    fd.append('path', targetPath)

    await clients[targetId].api.post('/fs/upload', fd, { timeout: 0 })

    // Step C: If cut
    if (mode === 'cut') {
      await fsDelete(sourceId, srcPath)
      clipboard.value = null
    }
  }

  // Rosbag
  const bagStart = async (id, opts) => {
    const client = clients[id]
    const res = await client.api.post('/bag/start', opts)
    client.recordingId = res.id
    return res
  }
  const bagStop = async (id, recId) => {
    const client = clients[id]
    await client.api.post('/bag/stop', { id: recId })
    if (client.recordingId === recId) client.recordingId = null
  }

  // 开始回放
  const bagPlayStart = async (clientId, options) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    // options: { path, rate, loop, clock, start_time, topics }
    try {
      const res = await client.api.post('/bag/play/start', options)
      // res: { status: "playing", id: "play-..." }
      client.playbackId = res.id
      client.playbackStatus = 'playing'
      return res
    } catch (e) {
      console.error('Bag play start failed:', e)
      throw e
    }
  }

  // 暂停/继续回放
  const bagPlayPause = async (clientId, action) => {
    // action: 'pause' | 'resume'
    const client = clients[clientId]
    if (!client || !client.api || !client.playbackId) return

    try {
      await client.api.post('/bag/play/pause', {
        id: client.playbackId,
        action: action
      })
      // 更新本地状态
      client.playbackStatus = action === 'pause' ? 'paused' : 'playing'
    } catch (e) {
      console.error(`Bag play ${action} failed:`, e)
      throw e
    }
  }

  // 停止回放
  const bagPlayStop = async (clientId) => {
    const client = clients[clientId]
    if (!client || !client.api || !client.playbackId) return

    try {
      await client.api.post('/bag/play/stop', { id: client.playbackId })
    } catch (e) {
      // 即使后端报错（比如进程已死），前端也要清理状态
      console.warn('Bag play stop warning:', e)
    } finally {
      client.playbackId = null
      client.playbackStatus = 'stopped'
    }
  }

  // Terminal
  const addTerminal = (id) => {
    const c = clients[id]
    if (!c) return
    const tid = c.nextTerminalId++
    c.terminals.push({ id: tid, name: `Terminal ${tid}` })
    return tid
  }
  const removeTerminal = (id, tid) => {
    const c = clients[id]
    if (c) c.terminals = c.terminals.filter((t) => t.id !== tid)
  }

  const removeNodeLog = (id, nodeId) => {
    const c = clients[id]
    if (c && c.nodeLogs && c.nodeLogs[nodeId]) delete c.nodeLogs[nodeId]
  }

  // ==================================================================================
  // Region 7: Editor Window Management
  // ==================================================================================

  /**
   * 统一打开文件编辑器的方法
   * 自动注入当前的 Token、Sudo密码和连接配置，防止 409 冲突
   * @param {string} clientId - 机器人的 UUID
   * @param {Object} fileInfo - { name: 'a.txt', path: '/home/a.txt' }
   */
  const openFileInEditor = async (clientId, fileInfo) => {
    const client = clients[clientId]
    if (!client) {
      ElMessage.error('无法打开编辑器：未找到连接会话')
      return
    }

    // 构造完整的 Payload
    const payload = {
      name: fileInfo.name,
      path: fileInfo.path,
      backendId: client.id, // UUID
      backendLabel: client.name || client.ip, // 显示名称

      // === 核心关键：透传凭证 ===
      token: client.token, // 共享 Session Token
      sudoPassword: sudoPassword.value, // 共享 Sudo 密码
      settings: {
        // 共享连接配置
        ip: client.ip,
        port: client.port,
        id: client.id,
        name: client.name,
        hostname: client.hostname
      }
    }

    try {
      // 调用 Preload 暴露的 API
      await window.api.openFileEditor(payload)
    } catch (e) {
      console.error('Failed to open editor window:', e)
      ElMessage.error('无法启动编辑器窗口: ' + e.message)
    }
  }

  // 4. 单纯打开编辑器窗口 (无文件)
  const openEditor = async () => {
    if (window.api && window.api.openEditorWindow) {
      await window.api.openEditorWindow()
      // 打开后立即广播所有活跃连接
      Object.values(clients).forEach((client) => broadcastClientState(client))
    }
  }

  // ==================================================================================
  // Region 8: Token Synchronization (Active Sync)
  // ==================================================================================

  // 辅助：广播单个客户端的状态给编辑器
  const broadcastClientState = (client) => {
    if (!client) return
    window.api.broadcastToken({
      id: client.id, // UUID
      token: client.token,
      status: client.status,
      settings: {
        ip: client.ip,
        port: client.port,
        name: client.name,
        id: client.id, // [核心修复] 必须包含 ID
        hostname: client.hostname
      }
    })
  }

  const setupSyncListeners = () => {
    window.api.onRequestTokens(() => {
      console.log('[Store] Editor requested tokens. Broadcasting...')
      Object.values(clients).forEach((client) => broadcastClientState(client))
    })
  }

  return {
    clients,
    activeID,
    activeClient,
    sudoPassword,
    clipboard,
    hostSystemState,
    updateHostSystemState,
    setClipboard,
    setSudoPassword,

    initClientPlaceholder,
    initOfflineClient,
    addConnection,
    removeConnection,
    refreshStatus,
    setClientStatus,

    syncNodes,
    saveNodeConfig,
    deleteNodeConfig,
    loadNodesFromCache,

    controlRosStack,
    startNodeProcess,
    stopNodeProcess,
    removeNodeLog,

    saveSequenceConfig,
    deleteSequenceConfig,
    runSequence,
    stopSequenceNodes,

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
    fsPaste,

    bagStart,
    bagStop,
    bagPlayStart,
    bagPlayPause,
    bagPlayStop,
    addTerminal,
    removeTerminal,

    openFileInEditor,
    openEditor,
    broadcastClientState,
    setupSyncListeners
  }
})
