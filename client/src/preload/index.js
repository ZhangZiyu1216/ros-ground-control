import { contextBridge, ipcRenderer } from 'electron'
import { electronAPI } from '@electron-toolkit/preload'

const api = {
  // 窗口控制
  onWindowFocusChanged: (callback) =>
    ipcRenderer.on('window-focus-changed', (_e, value) => callback(value)),

  // mDNS
  startMdnsScan: () => ipcRenderer.invoke('start-mdns-scan'),
  stopMdnsScan: () => ipcRenderer.invoke('stop-mdns-scan'),
  onMdnsServiceFound: (callback) => {
    const subscription = (_event, service) => callback(service)
    ipcRenderer.on('mdns-service-found', subscription)
    return () => {
      ipcRenderer.removeListener('mdns-service-found', subscription)
    }
  },
  getConfig: (key) => ipcRenderer.invoke('get-config', key),
  setConfig: (key, val) => ipcRenderer.invoke('set-config', key, val)
}

if (process.contextIsolated) {
  contextBridge.exposeInMainWorld('electron', electronAPI)
  contextBridge.exposeInMainWorld('api', api)
} else {
  window.electron = electronAPI
  window.api = api
}
