import { BrowserWindow, shell, ipcMain } from 'electron'
import { join } from 'path'
import { is } from '@electron-toolkit/utils'

class EditorWindowManager {
  constructor() {
    this.editorWindow = null
    this.pendingFileInfo = null
    this.isForceClosing = false
    this.closePromiseResolver = null // 修复：初始化变量

    // 监听渲染进程回复的 "是否允许关闭"
    ipcMain.handle('editor-close-response', (event, canClose) => {
      if (this.closePromiseResolver) {
        this.closePromiseResolver(canClose)
        this.closePromiseResolver = null
      }
    })
    // 监听渲染进程的“我就绪了”信号
    ipcMain.on('editor-ready-to-open', () => {
      if (this.pendingFileInfo && this.editorWindow) {
        console.log('[Main] Editor is ready, sending pending file:', this.pendingFileInfo.name)
        this.editorWindow.webContents.send('editor:open-file', this.pendingFileInfo)
        this.pendingFileInfo = null // 发送后清空，防止重复打开
      }
    })
  }

  async openFile(fileInfo) {
    // 1. 创建/恢复窗口 (保持不变)
    if (!this.editorWindow || this.editorWindow.isDestroyed()) {
      await this.createWindow()
    }
    if (this.editorWindow.isMinimized()) {
      this.editorWindow.restore()
    }
    this.editorWindow.focus()

    // 总是先存入暂存区
    this.pendingFileInfo = fileInfo
    // 尝试立即发送（针对窗口早已打开的情况）
    // 如果 Vue 还没 Ready，它会忽略，但会在 mounted 后通过 editor-ready-to-open 拉取
    this.editorWindow.webContents.send('editor:open-file', fileInfo)
  }

  async createWindow() {
    this.editorWindow = new BrowserWindow({
      width: 1200,
      height: 800,
      show: false,
      autoHideMenuBar: true,
      webPreferences: {
        preload: join(__dirname, '../preload/index.js'),
        sandbox: false,
        contextIsolation: true,
        webSecurity: false // 关键：禁用跨域策略以支持 Header
      },
      titleBarStyle: 'hidden',
      titleBarOverlay: {
        color: '#f5f7fa',
        symbolColor: '#606266',
        height: 35
      }
    })

    this.editorWindow.on('close', (e) => {
      if (!this.isForceClosing) {
        e.preventDefault()
        this.tryClose().then((canClose) => {
          if (canClose) {
            this.isForceClosing = true
            this.editorWindow.close()
          }
        })
      }
    })

    this.editorWindow.on('ready-to-show', () => {
      this.editorWindow.show()
    })

    this.editorWindow.webContents.setWindowOpenHandler((details) => {
      shell.openExternal(details.url)
      return { action: 'deny' }
    })

    if (is.dev && process.env['ELECTRON_RENDERER_URL']) {
      await this.editorWindow.loadURL(`${process.env['ELECTRON_RENDERER_URL']}/#/editor`)
    } else {
      await this.editorWindow.loadFile(join(__dirname, '../renderer/index.html'), {
        hash: 'editor'
      })
    }
  }

  tryClose() {
    return new Promise((resolve) => {
      if (!this.editorWindow || this.editorWindow.isDestroyed()) {
        resolve(true)
        return
      }
      this.closePromiseResolver = resolve
      this.editorWindow.webContents.send('editor-check-unsaved')
    })
  }

  forceClose() {
    this.isForceClosing = true
    if (this.editorWindow && !this.editorWindow.isDestroyed()) {
      this.editorWindow.close()
    }
  }
}

export default new EditorWindowManager()
