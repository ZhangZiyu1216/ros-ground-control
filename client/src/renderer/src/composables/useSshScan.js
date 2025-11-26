import { ref, onUnmounted } from 'vue'

export function useSshScan() {
  const sshDevices = ref([])
  let cleanupListener = null

  const start = async () => {
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
    await window.api.stopSshScan()
    if (cleanupListener) {
      cleanupListener()
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
