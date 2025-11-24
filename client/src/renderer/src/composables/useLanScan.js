import { ref, onUnmounted } from 'vue'

export function useLanScan() {
  const discoveredDevices = ref([])
  let cleanupListener = null

  const start = async () => {
    discoveredDevices.value = []
    if (cleanupListener) {
      cleanupListener()
      cleanupListener = null
    }

    cleanupListener = window.api.onMdnsServiceFound((service) => {
      // 这里的 service 现在是主进程透传过来的完整对象
      // 通常包含: name, type, host, port, addresses, txt, ip

      const incomingId = service.txt?.id
      // 为了 UI 显示友好，我们手动构造一个统一的 hostname 字段
      // 优先显示 TXT 里的 hostname，其次是服务名
      const displayHostname = service.txt?.hostname || service.name || service.host

      // 构造一个 UI 友好的对象
      const deviceItem = {
        ...service,
        hostname: displayHostname, // 覆盖/新增 hostname 字段供 UI 使用
        ip: service.ip || service.addresses?.find((a) => a.includes('.'))
      }

      // 去重
      const exists = discoveredDevices.value.find((d) => {
        if (incomingId && d.txt?.id) {
          return d.txt.id === incomingId
        }
        return d.ip === deviceItem.ip
      })

      if (!exists && deviceItem.ip) {
        discoveredDevices.value.push(deviceItem)
      }
    })

    await window.api.startMdnsScan()
  }

  const stop = async () => {
    await window.api.stopMdnsScan()
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
