import { ref, onUnmounted } from 'vue'

export function useLanScan() {
  const discoveredDevices = ref([])
  let cleanupListener = null

  const start = async () => {
    // 防御性检查
    if (!window.api || !window.api.startMdnsScan) {
      console.warn('[LanScan] API not available')
      return
    }

    discoveredDevices.value = []

    // 清理旧监听器
    if (cleanupListener) {
      cleanupListener()
      cleanupListener = null
    }

    // 注册监听
    cleanupListener = window.api.onMdnsServiceFound((service) => {
      // service 结构: { name, type, host, port, addresses, txt, ip }
      const incomingId = service.txt?.id
      // 优先显示 TXT 中的 hostname，其次是 host 字段，最后是 name
      const displayHostname = service.txt?.hostname || service.host || service.name

      const deviceItem = {
        ...service,
        hostname: displayHostname,
        ip: service.ip || service.addresses?.find((a) => a.includes('.')),
        port: service.port
      }

      // 仅当有有效 IP 时才处理
      if (!deviceItem.ip) return

      // 去重逻辑：优先匹配 ID，其次匹配 IP
      const existsIndex = discoveredDevices.value.findIndex((d) => {
        if (incomingId && d.txt?.id) {
          return d.txt.id === incomingId
        }
        return d.ip === deviceItem.ip && d.port === deviceItem.port
      })

      if (existsIndex !== -1) {
        // 更新现有项 (可能 hostname 变了)
        discoveredDevices.value[existsIndex] = deviceItem
      } else {
        discoveredDevices.value.push(deviceItem)
      }
    })

    await window.api.startMdnsScan()
  }

  const stop = async () => {
    if (window.api && window.api.stopMdnsScan) {
      await window.api.stopMdnsScan()
    }
    if (cleanupListener) {
      cleanupListener()
      cleanupListener = null
    }
  }

  onUnmounted(() => {
    stop()
  })

  return {
    discoveredDevices,
    start,
    stop
  }
}
