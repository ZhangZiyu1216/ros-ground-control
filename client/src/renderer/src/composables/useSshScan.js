import { ref, onUnmounted } from 'vue'

export function useSshScan() {
  const sshDevices = ref([])
  let cleanupListener = null

  const start = async () => {
    if (!window.api || !window.api.startSshScan) return

    sshDevices.value = []

    if (cleanupListener) {
      cleanupListener()
      cleanupListener = null
    }

    // 监听更新
    cleanupListener = window.api.onSshDevicesUpdate((list) => {
      sshDevices.value = list
    })

    await window.api.startSshScan()
  }

  const stop = async () => {
    if (window.api && window.api.stopSshScan) {
      await window.api.stopSshScan()
    }
    if (cleanupListener) {
      cleanupListener() // 调用返回的 removeListener 函数
      cleanupListener = null
    }
  }

  onUnmounted(() => {
    stop()
  })

  return {
    sshDevices,
    start,
    stop
  }
}
