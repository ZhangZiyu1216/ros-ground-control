import { BrowserWindow, shell, ipcMain } from 'electron'
import { join } from 'path'
import { is } from '@electron-toolkit/utils'

class EditorWindowManager {
  constructor() {
    this.editorWindow = null
    this.pendingFileInfo = null
    this.isForceClosing = false
    this.closePromiseResolver = null

    // 监听渲染进程回复
    ipcMain.handle('editor-close-response', (event, canClose) => {
      if (this.closePromiseResolver) {
        this.closePromiseResolver(canClose)
        this.closePromiseResolver = null
      }
    })

    ipcMain.on('editor-ready-to-open', () => {
      if (this.pendingFileInfo && this.editorWindow && !this.editorWindow.isDestroyed()) {
        console.log('[Main] Editor ready, sending file payload...')
        this.editorWindow.webContents.send('editor:open-file', this.pendingFileInfo)
        this.pendingFileInfo = null
      }
    })
  }

  async openFile(fileInfo) {
    if (!this.editorWindow || this.editorWindow.isDestroyed()) {
      await this.createWindow()
    } else {
      if (this.editorWindow.isMinimized()) this.editorWindow.restore()
      this.editorWindow.focus()
    }

    this.pendingFileInfo = fileInfo
    if (!this.editorWindow.webContents.isLoading()) {
      this.editorWindow.webContents.send('editor:open-file', fileInfo)
      this.pendingFileInfo = null
    }
  }

  async createWindow() {
    this.isForceClosing = false
    this.closePromiseResolver = null

    this.editorWindow = new BrowserWindow({
      width: 1024,
      height: 768,
      minWidth: 1024,
      minHeight: 768,
      show: false,
      autoHideMenuBar: true,
      webPreferences: {
        preload: join(__dirname, '../preload/index.js'),
        sandbox: false,
        contextIsolation: true,
        webSecurity: false // 允许加载本地资源或跨域
      },
      frame: false,
      titleBarStyle: 'hiddenInset'
    })

    // [核心修复] 窗口关闭拦截逻辑
    this.editorWindow.on('close', (e) => {
      if (this.isForceClosing) return // 强制关闭时直接放行

      // 1. 阻止默认关闭
      e.preventDefault()

      // 2. 询问渲染进程
      this.checkSafeToClose().then((canClose) => {
        if (canClose) {
          // 3. 如果允许，标记为强制关闭并再次调用 close
          this.isForceClosing = true
          this.editorWindow.close()
        }
      })
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

  /**
   * [新增] 检查编辑器是否安全可关闭（是否有未保存内容）
   * 返回 Promise<boolean>
   */
  checkSafeToClose() {
    return new Promise((resolve) => {
      // 如果窗口不存在或已销毁，直接视为安全
      if (!this.editorWindow || this.editorWindow.isDestroyed()) {
        resolve(true)
        return
      }

      // 存储 resolve，等待 IPC 回复
      // 如果之前的请求还没结束，直接覆盖（防止死锁）
      this.closePromiseResolver = resolve

      // 发送询问指令
      this.editorWindow.webContents.send('editor-check-unsaved')

      // [兜底] 聚焦窗口，防止用户没看到弹窗
      this.editorWindow.focus()
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
