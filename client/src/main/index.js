import { app, shell, BrowserWindow, ipcMain } from 'electron'
import { join } from 'path'
import { electronApp, optimizer, is } from '@electron-toolkit/utils'
import { Bonjour } from 'bonjour-service'
import Store from 'electron-store'

import icon from '../../resources/icon.png'

let win
const store = new Store.default()

function createWindow() {
  win = new BrowserWindow({
    width: 1024,
    height: 768,
    show: false,
    autoHideMenuBar: true,
    ...(process.platform === 'linux' ? { icon } : {}),
    webPreferences: {
      preload: join(__dirname, '../preload/index.js'),
      sandbox: false,
      contextIsolation: true
    },
    frame: false,
    titleBarStyle: 'hidden',
    titleBarOverlay: {
      color: '#2c3e50',
      symbolColor: '#ffffff',
      height: 50
    }
  })

  win.on('ready-to-show', () => win.show())

  // 窗口焦点事件传递给前端
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

app.whenReady().then(() => {
  electronApp.setAppUserModelId('com.ros-desktop.client')

  app.on('browser-window-created', (_, window) => {
    optimizer.watchWindowShortcuts(window)
  })

  createWindow()

  // --- mDNS 服务发现 ---
  const bonjour = new Bonjour()
  let browser = null

  // 监听前端发起的扫描请求
  ipcMain.handle('start-mdns-scan', () => {
    console.log('[Main] Starting mDNS scan for type: ros-agent')

    // 如果已有扫描，先停止
    if (browser) {
      browser.stop()
      browser = null
    }

    // 启动搜索
    // type: 'ros-agent' 会自动匹配 _ros-agent._tcp
    browser = bonjour.find({ type: 'ros-agent' }, (service) => {
      // 在主进程打印日志，确认 Node 是否收到了包
      console.log('[Main] mDNS Found:', service.name, service.txt)

      // 寻找 IPv4 地址
      const ip = service.addresses.find((addr) => addr.includes('.'))

      if (win) {
        // 【关键】透传整个 service 对象，不要手动挑字段，防止漏掉信息
        win.webContents.send('mdns-service-found', {
          ...service, // 展开所有属性 (name, port, type, txt, etc.)
          ip: ip // 显式附带一个方便使用的 ip
        })
      }
    })
    return true
  })

  ipcMain.handle('stop-mdns-scan', () => {
    console.log('[Main] Stopping mDNS scan')
    if (browser) {
      browser.stop()
      browser = null
    }
    return true
  })

  // --- 补全 Config 相关的 IPC ---
  ipcMain.handle('get-config', (event, key) => {
    return store.get(key)
  })
  ipcMain.handle('set-config', (event, key, val) => {
    store.set(key, val)
    return true
  })

  app.on('activate', function () {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})
