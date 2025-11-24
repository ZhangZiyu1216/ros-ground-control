import { defineStore } from 'pinia'
import { ref, computed, reactive } from 'vue'
import { createApi } from '../api/request'
import { ElNotification } from 'element-plus'

// --- 配置常量 ---
const HEALTH_CHECK_INTERVAL = 1000
const MAX_MISSED_HEARTBEATS = 3
const AUTO_DISCONNECT_THRESHOLD = 15
const MIN_CONNECTION_TIME = 600
const HANDSHAKE_TIMEOUT = 2500
const MAX_CONNECT_RETRIES = 2

export const useRobotStore = defineStore('robot', () => {
  const clients = reactive({})
  const activeID = ref(null)

  const activeClient = computed(() => (activeID.value ? clients[activeID.value] : null))
  const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms))

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

  // --- 心跳逻辑 ---
  const startHealthCheck = (id) => {
    const client = clients[id]
    if (!client) return
    if (client.timer) clearInterval(client.timer)

    client.timer = setInterval(async () => {
      if (!clients[id] || client.status === 'disconnected') {
        clearInterval(client.timer)
        client.timer = null
        return
      }
      try {
        const res = await client.api.get('/proc/list')
        client.missedHeartbeats = 0
        if (client.status === 'failed') {
          client.status = 'ready'
          ElNotification({
            title: '连接恢复',
            message: `${client.name || id} 已重连`,
            type: 'success'
          })
        }
        updateServiceStatusFromProcs(client, res.processes)
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        client.missedHeartbeats++
        if (client.missedHeartbeats >= MAX_MISSED_HEARTBEATS && client.status === 'ready') {
          client.status = 'failed'
          client.serviceStatus = { roscore: 'unknown', bridge: 'unknown' }
          ElNotification({
            title: '连接中断',
            message: `与 ${client.name || id} 失去联系，正在尝试重连...`,
            type: 'error',
            duration: 3000
          })
        }
        if (client.missedHeartbeats >= AUTO_DISCONNECT_THRESHOLD) {
          clearInterval(client.timer)
          client.timer = null
          client.status = 'disconnected'
          ElNotification({
            title: '连接超时',
            message: `${client.name || id} 已断开连接。`,
            type: 'info',
            duration: 4000
          })
        }
      }
    }, HEALTH_CHECK_INTERVAL)
  }

  const updateServiceStatusFromProcs = (client, processes) => {
    if (!Array.isArray(processes)) return
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
        api: null,
        sysInfo: {},
        serviceStatus: { roscore: 'unknown', bridge: 'unknown' },
        missedHeartbeats: 0,
        timer: null,
        ipChanged: false
      }
    } else {
      // 复用现有对象，更新状态为 loading
      clients[currentKey].status = 'setting_up'
      clients[currentKey].ip = targetIp // 更新尝试连接的 IP
      clients[currentKey].name = settings.name
    }

    activeID.value = currentKey
    const startTime = Date.now()

    let apiInstance = null
    let finalIp = targetIp
    let finalId = null // 最终拿到的硬件 ID
    let infoRes = {}
    let procRes = { processes: [] }
    let connectError = null

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
          await sleep(500)
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

      updateServiceStatusFromProcs(finalClient, procRes.processes)

      // 切换视图到最终的 ID Key
      activeID.value = currentKey

      startHealthCheck(currentKey)
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

  return {
    clients,
    activeID,
    activeClient,
    addConnection,
    removeConnection,
    refreshStatus,
    setClientStatus
  }
})
