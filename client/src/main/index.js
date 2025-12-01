import { app, shell, BrowserWindow, ipcMain } from 'electron'
import { join } from 'path'
import { electronApp, optimizer, is } from '@electron-toolkit/utils'
import { Bonjour } from 'bonjour-service'
import Store from 'electron-store' // 确保安装了 electron-store
import editorWindowManager from './EditorWindowManager.js'
import sshDiscovery from './SshDiscovery'
import { deployAgent } from './AgentDeployer'
import icon from '../../resources/icon.png'

let win
// 修正 Store 初始化 (通常不需要 .default，除非特定的打包配置)
const store = new Store.default()

function createWindow() {
  win = new BrowserWindow({
    width: 1024,
    height: 768,
    minWidth: 1024,
    minHeight: 768,
    show: false,
    autoHideMenuBar: true,
    ...(process.platform === 'linux' ? { icon } : {}),
    webPreferences: {
      preload: join(__dirname, '../preload/index.js'),
      sandbox: false,
      contextIsolation: true,
      webSecurity: false // 允许加载本地资源或跨域
    },
    frame: false,
    titleBarStyle: 'hiddenInset'
  })

  // 强制直连模式，避免局域网 IP 走系统代理导致 WS 连不上
  win.webContents.session.setProxy({ mode: 'direct' }).then(() => {
    console.log('[Main] System proxy bypassed (Direct Mode)')
  })

  win.on('ready-to-show', () => win.show())

  win.on('close', (e) => {
    // 只有当 win 存在且未销毁时才检查
    if (win && !win.isDestroyed()) {
      // 1. 阻止主窗口关闭
      e.preventDefault()

      // 2. 检查编辑器是否有未保存内容
      editorWindowManager.checkSafeToClose().then((canClose) => {
        if (canClose) {
          // 3. 如果编辑器说可以关闭，先强制关闭编辑器
          editorWindowManager.forceClose()

          // 4. 然后销毁主窗口 (绕过 preventDefault loop)
          // 注意：使用 destroy() 比 removeListener + close() 更直接且安全
          win.destroy()
          app.quit()
        }
        // 如果 canClose 为 false，主窗口保持打开，用户需要先处理编辑器弹窗
      })
    }
  })

  // 窗口焦点状态传递
  win.on('focus', () => win.webContents.send('window-focus-changed', true))
  win.on('blur', () => win.webContents.send('window-focus-changed', false))

  win.webContents.setWindowOpenHandler((details) => {
    shell.openExternal(details.url)
    return { action: 'deny' }
  })

  if (is.dev && process.env['ELECTRON_RENDERER_URL']) {
    win.loadURL(process.env['ELECTRON_RENDERER_URL'])
  } else {
    win.loadFile(join(__dirname, '../renderer/index.html'))
  }
}

// 窗口控制 IPC
ipcMain.on('window-min', (event) => BrowserWindow.fromWebContents(event.sender)?.minimize())
ipcMain.on('window-max', (event) => {
  const window = BrowserWindow.fromWebContents(event.sender)
  if (window) {
    window.isMaximized() ? window.unmaximize() : window.maximize()
  }
})
ipcMain.on('window-close', (event) => BrowserWindow.fromWebContents(event.sender)?.close())

app.whenReady().then(() => {
  electronApp.setAppUserModelId('com.ros-desktop.client')

  app.on('browser-window-created', (_, window) => {
    optimizer.watchWindowShortcuts(window)
  })

  createWindow()

  // --- mDNS 服务发现 ---
  const bonjour = new Bonjour()
  let browser = null

  ipcMain.handle('start-mdns-scan', () => {
    console.log('[Main] Starting mDNS scan for: _ros-agent._tcp')
    if (browser) {
      browser.stop()
      browser = null
    }

    browser = bonjour.find({ type: 'ros-agent' }, (service) => {
      // 提取 IPv4
      const ip = service.addresses?.find((addr) => addr.includes('.'))

      // 广播给所有窗口 (主窗口 + 编辑器窗口)
      BrowserWindow.getAllWindows().forEach((w) => {
        if (!w.isDestroyed()) {
          w.webContents.send('mdns-service-found', {
            ...service,
            ip: ip
          })
        }
      })
    })
    return true
  })

  ipcMain.handle('stop-mdns-scan', () => {
    if (browser) {
      console.log('[Main] Stopping mDNS scan')
      browser.stop()
      browser = null
    }
    return true
  })

  // --- 编辑器与配置 ---
  ipcMain.handle('get-config', (event, key) => store.get(key))
  ipcMain.handle('set-config', (event, key, val) => {
    store.set(key, val)
    return true
  })

  // 令牌同步通道：主窗口 -> 编辑器
  ipcMain.on('sync-token-to-editor', (event, payload) => {
    // payload: { id, token, settings, status }
    if (editorWindowManager.editorWindow && !editorWindowManager.editorWindow.isDestroyed()) {
      editorWindowManager.editorWindow.webContents.send('editor:update-token', payload)
    }
  })

  // 状态请求通道：编辑器 -> 主窗口
  ipcMain.on('editor-request-tokens', () => {
    // 转发给主窗口（Main Renderer）
    if (win && !win.isDestroyed()) {
      win.webContents.send('main:request-tokens')
    }
  })

  // 打开编辑器 (支持无文件打开)
  ipcMain.handle('open-file-editor', async (event, fileInfo) => {
    // 如果 fileInfo 为 null，仅打开/聚焦窗口
    if (!fileInfo) {
      await editorWindowManager.openFile(null)
    } else {
      await editorWindowManager.openFile(fileInfo)
    }
  })

  // 状态广播
  ipcMain.on('broadcast-backend-status', (event, payload) => {
    BrowserWindow.getAllWindows().forEach((w) => {
      if (!w.isDestroyed()) {
        w.webContents.send('backend-status-changed', payload)
      }
    })
  })

  // --- SSH 扫描 ---
  ipcMain.handle('start-ssh-scan', () => {
    sshDiscovery.startScan()
    return true
  })
  ipcMain.handle('stop-ssh-scan', () => {
    sshDiscovery.stopScan()
    return true
  })

  // --- 部署 ---
  ipcMain.handle('deploy-agent', async (event, config) => {
    return await deployAgent(config, event.sender)
  })

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })

  app.on('before-quit', () => {
    sshDiscovery.destroy()
    editorWindowManager.forceClose()
  })
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})
