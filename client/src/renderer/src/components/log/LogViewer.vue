<template>
  <div class="viewer-wrapper">
    <div ref="terminalContainer" class="terminal-container"></div>
    <!-- [新增] 断开连接遮罩层 -->
    <div v-if="disabled" class="disconnected-overlay">
      <div class="overlay-content">
        <el-icon size="24"><WarningFilled /></el-icon>
        <span>CONNECTION LOST - READ ONLY</span>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch, nextTick } from 'vue'
import { Terminal } from 'xterm'
import { FitAddon } from 'xterm-addon-fit'
import { WebLinksAddon } from 'xterm-addon-web-links'
import { useRobotStore } from '../../store/robot'
import 'xterm/css/xterm.css'

const props = defineProps({
  mode: { type: String, default: 'log' },
  lines: { type: Array, default: () => [] },
  backendId: { type: String, default: '' },
  disabled: { type: Boolean, default: false }
})

const store = useRobotStore()

const terminalContainer = ref(null)
let terminal = null
let fitAddon = null
let socket = null
let resizeObserver = null
let themeObserver = null // [新增] 用于监听主题变化
let lastLogIndex = 0

// [新增] Xterm 主题配置
const themes = {
  // 浅色模式：高对比度优化
  light: {
    background: '#fafafa', // 柔和的护眼灰白，不刺眼
    foreground: '#333333', // 正文深灰
    cursor: '#333333',
    cursorAccent: '#ffffff',
    selectionBackground: 'rgba(64, 158, 255, 0.2)',

    // ANSI 颜色重映射 (关键：将原本的高亮色变深，适应白底)
    black: '#000000',
    red: '#d32f2f', // 深红
    green: '#2e7d32', // 森林绿 (原标准绿在白底看不清)
    yellow: '#f57f17', // 深琥珀色/橙棕色 (原黄色在白底看不清)
    blue: '#1565c0', // 深蓝
    magenta: '#880e4f', // 深紫
    cyan: '#006064', // 深青
    white: '#616161', // 深灰 (模拟白字)

    // Bright variants (粗体/高亮色)
    brightBlack: '#9e9e9e',
    brightRed: '#f44336',
    brightGreen: '#4caf50', // 稍微亮一点的绿
    brightYellow: '#ff8f00', // 亮橙色
    brightBlue: '#2196f3',
    brightMagenta: '#e91e63',
    brightCyan: '#00bcd4',
    brightWhite: '#eeeeee' // 真正的白 (用于某些背景块)
  },

  // 深色模式：保持经典的控制台风格
  dark: {
    background: '#1e1e1e', // VS Code Dark
    foreground: '#cccccc',
    cursor: '#ffffff',
    cursorAccent: '#000000',
    selectionBackground: 'rgba(255, 255, 255, 0.15)',

    // 标准 ANSI 颜色
    black: '#000000',
    red: '#cd3131',
    green: '#0dbc79',
    yellow: '#e5e510',
    blue: '#2472c8',
    magenta: '#bc3fbc',
    cyan: '#11a8cd',
    white: '#e5e5e5',

    brightBlack: '#666666',
    brightRed: '#f14c4c',
    brightGreen: '#23d18b',
    brightYellow: '#f5f543',
    brightBlue: '#3b8eea',
    brightMagenta: '#d670d6',
    brightCyan: '#29b8db',
    brightWhite: '#e5e5e5'
  }
}

// [新增] 获取当前主题
const getTheme = () => {
  return document.documentElement.classList.contains('dark') ? themes.dark : themes.light
}

// --- Xterm 初始化 ---
const initXterm = () => {
  terminal = new Terminal({
    cursorBlink: props.mode === 'terminal' && !props.disabled,
    disableStdin: props.mode === 'log' || props.disabled,
    convertEol: true,
    fontSize: 13,
    fontFamily: 'Consolas, "Courier New", monospace',
    theme: getTheme(),
    allowProposedApi: true
  })

  fitAddon = new FitAddon()
  terminal.loadAddon(fitAddon)
  terminal.loadAddon(new WebLinksAddon())

  terminal.open(terminalContainer.value)

  // [修改] 优化 Resize 逻辑，确保填满容器
  resizeObserver = new ResizeObserver(() => {
    // 使用 requestAnimationFrame 避免频繁触发
    window.requestAnimationFrame(() => {
      if (!fitAddon) return
      try {
        fitAddon.fit()
        if (props.mode === 'terminal' && socket && socket.readyState === WebSocket.OPEN) {
          sendResize(terminal.cols, terminal.rows)
        }
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        // 忽略 fit 报错 (例如元素不可见时)
      }
    })
  })
  resizeObserver.observe(terminalContainer.value)

  // [新增] 监听 HTML class 变化以切换深色模式
  themeObserver = new MutationObserver((mutations) => {
    mutations.forEach((mutation) => {
      if (mutation.attributeName === 'class') {
        terminal.options.theme = getTheme()
      }
    })
  })
  themeObserver.observe(document.documentElement, { attributes: true, attributeFilter: ['class'] })
}

// [新增] 监听 disabled 状态，动态控制交互
watch(
  () => props.disabled,
  (isDisabled) => {
    if (!terminal) return

    // 1. 禁止输入
    terminal.options.disableStdin = isDisabled
    // 2. 停止光标闪烁 (提示失去焦点)
    terminal.options.cursorBlink = !isDisabled
    // 3. 改变光标样式
    terminal.options.cursorStyle = isDisabled ? 'block' : 'block' // 也可以改成 underline

    // 如果断开，强制刷新一次布局
    if (isDisabled && fitAddon) fitAddon.fit()
  }
)

// --- Log 模式 (完全保留原逻辑) ---
const setupLogMode = () => {
  const writeLines = (linesToWrite) => {
    const text = linesToWrite.join('')
    terminal.write(text)
  }

  if (props.lines.length > 0) {
    writeLines(props.lines)
    lastLogIndex = props.lines.length
  }

  watch(
    () => props.lines.length,
    (newLen) => {
      if (newLen > lastLogIndex) {
        const newLines = props.lines.slice(lastLogIndex)
        writeLines(newLines)
        lastLogIndex = newLen
        // 只有当用户接近底部时才自动滚动，提升体验 (可选优化，这里暂时保持原样)
        terminal.scrollToBottom()
      }
    }
  )
}

// --- Terminal 模式 (完全保留原逻辑) ---
const setupTerminalMode = () => {
  if (!props.backendId) return
  if (props.disabled) {
    terminal.write('\x1b[1;31m[System] Connection is currently unavailable.\x1b[0m\r\n')
    return
  }

  const client = store.clients[props.backendId]
  if (!client || !client.ip) {
    terminal.write('\x1b[1;31m[Error] Client info not found\x1b[0m\r\n')
    return
  }

  const protocol = 'ws://'
  const host = `${client.ip}:${client.port || 8080}`
  const path = '/ws/terminal'
  const query = client.token ? `?token=${encodeURIComponent(client.token)}` : ''
  const wsUrl = `${protocol}${host}${path}${query}`

  socket = new WebSocket(wsUrl)
  socket.binaryType = 'arraybuffer'

  socket.onopen = () => {
    terminal.write('\x1b[1;32m[Connected to Terminal]\x1b[0m\r\n')
    // 连接成功后立即适应一次
    fitAddon.fit()
    sendResize(terminal.cols, terminal.rows)
    terminal.focus()
  }

  socket.onmessage = (ev) => {
    if (ev.data instanceof ArrayBuffer) {
      terminal.write(new Uint8Array(ev.data))
    } else if (typeof ev.data === 'string') {
      terminal.write(ev.data)
    }
  }

  socket.onclose = (e) => {
    let reason = ''
    if (e.code === 1006) reason = ' (Connection Refused/Abnormal)'
    else if (e.code === 4001) reason = ' (Unauthorized)'
    terminal.write(`\r\n\x1b[1;31m[Connection Closed${reason}]\x1b[0m\r\n`)
  }

  socket.onerror = () => {
    terminal.write(`\r\n\x1b[1;31m[Error] Socket Error\x1b[0m\r\n`)
  }

  terminal.onData((data) => {
    if (props.disabled) return
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: 'input', data: data }))
    }
  })

  terminal.onResize((size) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      sendResize(size.cols, size.rows)
    }
  })
}

const sendResize = (cols, rows) => {
  socket.send(JSON.stringify({ type: 'resize', cols, rows }))
}

// --- Lifecycle ---
onMounted(() => {
  // 确保 DOM 渲染后再初始化，避免尺寸计算错误
  nextTick(() => {
    initXterm()
    if (props.mode === 'log') {
      setupLogMode()
    } else {
      setupTerminalMode()
    }

    // 再次延迟调用 fit，确保 Tab 切换动画完成
    setTimeout(() => {
      if (fitAddon) {
        fitAddon.fit()
        if (props.mode === 'terminal' && terminal) {
          // 不发送 resize，避免重复，ResizeObserver 会处理
        }
      }
    }, 200)
  })
})

onUnmounted(() => {
  if (resizeObserver) resizeObserver.disconnect()
  if (themeObserver) themeObserver.disconnect() // 清理主题监听
  if (socket) {
    socket.onclose = null
    socket.close()
  }
  if (terminal) terminal.dispose()
})

// 暴露 fit 方法，供父组件 Tab 切换时调用
defineExpose({
  fit: () => {
    if (fitAddon) {
      // 这里的延时非常重要，因为切换 Tab 时 DOM 尺寸可能还没更新完毕
      setTimeout(() => fitAddon.fit(), 50)
    }
  }
})
</script>

<style scoped>
.viewer-wrapper {
  width: 100%;
  height: 100%;
  position: relative; /* 关键 */
  overflow: hidden;
}

.terminal-container {
  width: 100%;
  height: 100%;
  background-color: var(--bg-color, #ffffff);
  box-sizing: border-box;
  overflow: hidden;
  padding: 4px 8px 4px 4px;
}

:global(html.dark) .terminal-container {
  background-color: #1e1e1e;
}

:deep(.xterm-viewport) {
  width: calc(100% + 8px) !important;
  /* 强制显示滚动条 (可选，视需求而定) */
  overflow-y: scroll !important;
}

/* [新增] 断开连接遮罩 */
.disconnected-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  /* 半透明红色背景条纹，增强警示感 */
  background: repeating-linear-gradient(
    45deg,
    rgba(245, 108, 108, 0.05),
    rgba(245, 108, 108, 0.05) 10px,
    rgba(245, 108, 108, 0.1) 10px,
    rgba(245, 108, 108, 0.1) 20px
  );
  pointer-events: none; /* 允许鼠标滚轮穿透滚动查看日志 */
  display: flex;
  align-items: flex-end; /* 放在底部，避免遮挡中间内容 */
  justify-content: center;
  padding-bottom: 20px;
  z-index: 10;
  /* 加上一点灰度滤镜，让终端看起来变灰了 */
  backdrop-filter: grayscale(80%);
}

.overlay-content {
  background-color: #f56c6c;
  color: white;
  padding: 6px 16px;
  border-radius: 20px;
  font-size: 12px;
  font-weight: bold;
  display: flex;
  align-items: center;
  gap: 8px;
  box-shadow: 0 4px 12px rgba(245, 108, 108, 0.4);
  letter-spacing: 1px;
}

/* 深色模式下的容器背景兜底 */
:global(html.dark) .terminal-container {
  background-color: #1e1e1e;
}

/* 隐藏 Xterm 自身丑陋的滚动条，WebLinksAddon 等有时会产生 */
:deep(.xterm-viewport) {
  overflow-y: auto !important;
}
:deep(.xterm-viewport::-webkit-scrollbar) {
  width: 8px;
}
:deep(.xterm-viewport::-webkit-scrollbar-thumb) {
  background-color: rgba(128, 128, 128, 0.4);
  border-radius: 4px;
}
:deep(.xterm-viewport::-webkit-scrollbar-track) {
  background: transparent;
}
</style>
