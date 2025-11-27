<!-- src/renderer/src/components/LogViewer.vue -->
<template>
  <div ref="terminalContainer" class="terminal-container"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { Terminal } from 'xterm'
import { FitAddon } from 'xterm-addon-fit'
import { WebLinksAddon } from 'xterm-addon-web-links' // 推荐添加，支持点击链接
import 'xterm/css/xterm.css'

const props = defineProps({
  // 模式: 'log' (只读日志) | 'terminal' (交互式终端)
  mode: { type: String, default: 'log' },
  // [Log Mode] 日志行数组
  lines: { type: Array, default: () => [] },
  // [Terminal Mode] 目标 IP (用于建立 WS 连接)
  hostIp: { type: String, default: '' }
})

const terminalContainer = ref(null)
let terminal = null
let fitAddon = null
let socket = null
let resizeObserver = null
let lastLogIndex = 0 // Log模式下的增量指针

// --- 初始化 Xterm ---
const initXterm = () => {
  terminal = new Terminal({
    cursorBlink: props.mode === 'terminal', // 只有交互模式才闪烁光标
    disableStdin: props.mode === 'log', // 日志模式禁止输入
    convertEol: true,
    fontSize: 13,
    fontFamily: 'Consolas, "Courier New", monospace',
    theme: {
      background: '#1e1e1e', // 深色背景更适合终端
      foreground: '#d4d4d4'
    }
  })

  fitAddon = new FitAddon()
  terminal.loadAddon(fitAddon)
  terminal.loadAddon(new WebLinksAddon())

  terminal.open(terminalContainer.value)

  // 监听容器大小变化
  resizeObserver = new ResizeObserver(() => {
    fitAddon.fit()
    // 如果是交互终端，还需要通知后端调整 PTY 大小
    if (props.mode === 'terminal' && socket && socket.readyState === WebSocket.OPEN) {
      sendResize(terminal.cols, terminal.rows)
    }
  })
  resizeObserver.observe(terminalContainer.value)

  // 初始适应
  setTimeout(() => fitAddon.fit(), 50)
}

// --- Log 模式逻辑 ---
const setupLogMode = () => {
  // 初始写入已有日志
  if (props.lines.length > 0) {
    props.lines.forEach((line) => terminal.write(line))
    lastLogIndex = props.lines.length
  }

  // 监听新日志
  watch(
    () => props.lines.length,
    (newLen) => {
      if (newLen > lastLogIndex) {
        const newLines = props.lines.slice(lastLogIndex)
        newLines.forEach((line) => terminal.write(line))
        lastLogIndex = newLen
        // 自动滚动到底部
        terminal.scrollToBottom()
      }
    }
  )
}

// --- Terminal 模式逻辑 ---
const setupTerminalMode = () => {
  if (!props.hostIp) return

  const wsUrl = `ws://${props.hostIp}:8080/ws/terminal`
  socket = new WebSocket(wsUrl)

  // 二进制传输 (Agent 说明: Server->Client 是二进制流)
  socket.binaryType = 'arraybuffer'

  socket.onopen = () => {
    terminal.write('\x1b[1;32m[Connected to Terminal]\x1b[0m\r\n')
    fitAddon.fit()
    sendResize(terminal.cols, terminal.rows)
  }

  socket.onmessage = (ev) => {
    // 写入终端
    // 如果收到的是字符串(JSON)，可能是 resize 回执等，暂时忽略或处理
    // 如果收到的是 ArrayBuffer/Blob，直接写入 xterm
    if (ev.data instanceof ArrayBuffer) {
      terminal.write(new Uint8Array(ev.data))
    } else if (typeof ev.data === 'string') {
      // 容错处理文本数据
      terminal.write(ev.data)
    }
  }

  socket.onclose = () => {
    terminal.write('\r\n\x1b[1;31m[Connection Closed]\x1b[0m\r\n')
  }

  socket.onerror = (err) => {
    terminal.write(`\r\n\x1b[1;31m[Error] ${err.message}\x1b[0m\r\n`)
  }

  // 监听用户输入 -> 发送给后端
  terminal.onData((data) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      // 协议: 输入直接发 JSON { type: 'input', data: ... }
      socket.send(JSON.stringify({ type: 'input', data: data }))
    }
  })

  // 监听 Resize -> 发送给后端
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
  initXterm()
  if (props.mode === 'log') {
    setupLogMode()
  } else {
    setupTerminalMode()
  }
})

onUnmounted(() => {
  if (resizeObserver) resizeObserver.disconnect()
  if (socket) socket.close()
  if (terminal) terminal.dispose()
})

// 暴露 fit 方法给父组件 (Tab 切换时可能需要手动调用)
defineExpose({
  fit: () => fitAddon && fitAddon.fit()
})
</script>

<style scoped>
.terminal-container {
  width: 100%;
  height: 100%;
  background-color: #1e1e1e; /* 避免闪烁白屏 */
  padding: 5px;
  box-sizing: border-box;
}
</style>
