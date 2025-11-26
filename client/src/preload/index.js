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
  setConfig: (key, val) => ipcRenderer.invoke('set-config', key, val),

  // 编辑器专用
  openFileEditor: (info) => ipcRenderer.invoke('open-file-editor', info),
  saveEditorState: (state) => ipcRenderer.invoke('set-config', 'editor_state', state),
  loadEditorState: () => ipcRenderer.invoke('get-config', 'editor_state'),
  editorIsReady: () => ipcRenderer.send('editor-ready-to-open'),

  // 编辑器窗口通信
  onEditorOpenFile: (cb) => ipcRenderer.on('editor:open-file', cb),
  onCheckUnsaved: (cb) => ipcRenderer.on('editor-check-unsaved', cb),
  respondClose: (canClose) => ipcRenderer.invoke('editor-close-response', canClose),
  broadcastBackendStatus: (payload) => ipcRenderer.send('broadcast-backend-status', payload),
  onBackendStatusChanged: (cb) => ipcRenderer.on('backend-status-changed', (_e, val) => cb(val)),

  // SSH 扫描
  startSshScan: () => ipcRenderer.invoke('start-ssh-scan'),
  stopSshScan: () => ipcRenderer.invoke('stop-ssh-scan'),
  onSshDevicesUpdate: (callback) => {
    const subscription = (_event, list) => callback(list)
    ipcRenderer.on('ssh-devices-update', subscription)
    // 返回 removeListener 函数
    return () => {
      ipcRenderer.removeListener('ssh-devices-update', subscription)
    }
  },
  // 部署
  deployAgent: (config) => ipcRenderer.invoke('deploy-agent', config),
  onDeployProgress: (callback) => ipcRenderer.on('deploy-progress', (_e, value) => callback(value))
}

if (process.contextIsolated) {
  contextBridge.exposeInMainWorld('electron', electronAPI)
  contextBridge.exposeInMainWorld('api', api)
} else {
  window.Electron = electronAPI
  window.api = api
}
