import { defineStore } from 'pinia'
import { ref, computed, reactive } from 'vue'
import { createApi } from '../api/request'
import { ElNotification } from 'element-plus'

// --- 配置常量 ---
const HEALTH_CHECK_INTERVAL = 1000
const MAX_MISSED_HEARTBEATS = 3
const AUTO_DISCONNECT_THRESHOLD = 15
const MIN_CONNECTION_TIME = 600
const HANDSHAKE_TIMEOUT = 500
const MAX_CONNECT_RETRIES = 2

export const useRobotStore = defineStore('robot', () => {
  const clients = reactive({})
  const activeID = ref(null)

  const activeClient = computed(() => (activeID.value ? clients[activeID.value] : null))
  const getCacheKey = (id) => `robot_nodes_cache_${id}` // --- 辅助：缓存 Key 生成 ---
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
        if (proc) {
          node.status = 'running'
        } else {
          // 如果之前是 starting，可能还没启动完，不要立即置为 stopped，除非超时(需额外逻辑，暂简化)
          // 这里简单处理：只要列表里没有，就是 stopped
          // 为了防止 starting 瞬间变 stopped，可以加个 ignore 锁，或者依赖 Agent 响应速度
          if (node.status !== 'starting') {
            node.status = 'stopped'
          }
        }
      })
    }
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
        ipChanged: false
      }
    } else {
      // 复用现有对象，更新状态为 loading
      clients[currentKey].status = 'setting_up'
      clients[currentKey].ip = targetIp // 更新尝试连接的 IP
      clients[currentKey].name = settings.name
    }

    // 尝试加载本地缓存，防止界面空白
    try {
      const cached = localStorage.getItem(getCacheKey(currentKey))
      if (cached) {
        clients[currentKey].nodes = JSON.parse(cached)
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
      syncNodes(currentKey)

      updateServiceStatusFromProcs(finalClient, procRes.processes)

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
        localStorage.setItem(getCacheKey(clientId), JSON.stringify(client.nodes))
      }
    } catch (e) {
      console.error('Failed to sync nodes:', e)
    }
  }

  // 2. 保存/更新节点 (Push & Cache)
  const saveNodeConfig = async (clientId, nodeData) => {
    const client = clients[clientId]
    // 先更新本地 UI (乐观更新)
    const existingIdx = client.nodes.findIndex((n) => n.id === nodeData.id)
    if (existingIdx !== -1) {
      client.nodes[existingIdx] = { ...client.nodes[existingIdx], ...nodeData }
    } else {
      client.nodes.push({ ...nodeData, status: 'stopped' })
    }
    // 更新缓存
    localStorage.setItem(getCacheKey(clientId), JSON.stringify(client.nodes))

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
    localStorage.setItem(getCacheKey(clientId), JSON.stringify(client.nodes))

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
    if (target) target.status = 'starting'

    try {
      await client.api.post('/proc/start', payload)
      // 成功后不需要手动设为 running，心跳包 (proc/list) 会自动更新它
    } catch (e) {
      if (target) target.status = 'error'
      throw e
    }
  }

  // 5. 停止节点 (调用 /proc/stop)
  const stopNodeProcess = async (clientId, nodeIdName) => {
    const client = clients[clientId]
    if (!client || !client.api) return

    await client.api.post('/proc/stop', { id: nodeIdName })
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

  // 4. 写文件 (Write - 用于新建空文件)
  const fsWriteFile = async (clientId, pathStr, content = '') => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/write', { path: pathStr, content })
  }

  // 5. 重命名/移动 (Rename/Move)
  const fsRename = async (clientId, src, dst) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/rename', { src, dst })
  }

  // 6. 复制 (Copy)
  const fsCopy = async (clientId, src, dst) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/copy', { src, dst })
  }

  // 7. 删除 (Delete/Trash)
  const fsDelete = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/delete', { path: pathStr })
  }

  // 8. 彻底删除
  const fsDeletePermanent = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/delete/permanent', { path: pathStr })
  }

  // 9. 还原回收站文件
  const fsRestore = async (clientId, pathStr) => {
    const client = clients[clientId]
    if (!client || !client.api) return
    await client.api.post('/fs/restore', { path: pathStr })
  }
  // 10. 上传文件
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

  return {
    clients,
    activeID,
    activeClient,
    sudoPassword,
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
    fsGetSidebar,
    fsListDir,
    fsMkdir,
    fsWriteFile,
    fsRename,
    fsCopy,
    fsDelete,
    fsDeletePermanent,
    fsRestore,
    fsUpload
  }
})
