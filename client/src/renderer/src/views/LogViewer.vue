<!-- src/renderer/src/components/LogViewer.vue -->

<template>
  <!-- xterm.js 将会挂载到这个 div 上 -->
  <div ref="terminalContainer" class="terminal-container"></div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { Terminal } from 'xterm'
import { FitAddon } from 'xterm-addon-fit'
import 'xterm/css/xterm.css' // 导入 xterm 的核心 CSS

const props = defineProps({
  logs: {
    type: Array,
    required: true
  }
})

function clearTerminal() {
  if (terminal) {
    terminal.clear()
  }
  lastLogIndex = 0
}

function writeHistory() {
  if (!terminal) return
  props.logs.forEach((log) => terminal.write(log.message))
  lastLogIndex = props.logs.length
}

defineExpose({
  fitTerminal: () => fitAddon?.fit(),
  clearTerminal,
  writeHistory
})

const terminalContainer = ref(null)
let terminal = null
let fitAddon = null
let resizeObserver = null
let lastLogIndex = 0

onMounted(() => {
  // 1. 初始化终端实例
  terminal = new Terminal({
    // 终端配置
    cursorBlink: false,
    disableStdin: true, // 我们只用于显示，不需要用户输入
    convertEol: true, // 自动将 \n 转为 \r\n
    fontSize: 13,
    fontFamily: 'Consolas, "Courier New", monospace',
    theme: {
      background: '#f5f7fa', // 匹配我们UI的浅色背景
      foreground: '#303133', // 默认文字颜色
      cursor: 'transparent', // 隐藏光标
      black: '#000000',
      red: '#F56C6C',
      green: '#67C23A',
      yellow: '#E6A23C',
      blue: '#409EFF',
      magenta: '#c586c0',
      cyan: '#56b6c2',
      white: '#ffffff'
    }
  })

  // 2. 初始化并加载 FitAddon 插件
  fitAddon = new FitAddon()
  terminal.loadAddon(fitAddon)

  // 3. 将终端挂载到 DOM
  if (terminalContainer.value) {
    terminal.open(terminalContainer.value)
    writeHistory()

    // 4. 监听容器尺寸变化，自动调整终端大小
    resizeObserver = new ResizeObserver(() => {
      if (terminal) {
        // 重新设置 rows，触发 xterm 内部的 resize
        const { clientHeight } = terminalContainer.value
        const rowHeight = terminal._core._renderService.dimensions.actualCellHeight || 18 // 兜底高度
        const rows = Math.floor(clientHeight / rowHeight)
        if (rows > 0) terminal.resize(terminal.cols, rows)
      }
      setTimeout(() => fitAddon.fit(), 50)
    })
    resizeObserver.observe(terminalContainer.value)
  }
})

// 6. 组件卸载时，清理资源
onUnmounted(() => {
  if (resizeObserver && terminalContainer.value) {
    resizeObserver.unobserve(terminalContainer.value)
  }
  if (terminal) {
    terminal.dispose()
  }
})

// 监听日志数组的变化，增量写入新日志
watch(
  () => props.logs.length,
  (newLength) => {
    if (!terminal || newLength <= lastLogIndex) return

    // 只写入从上次索引到当前长度之间的新日志
    const newLogs = props.logs.slice(lastLogIndex)
    newLogs.forEach((log) => {
      // xterm.js 会自动处理 \r\n 换行
      terminal.write(log.message)
    })
    lastLogIndex = newLength // 更新索引
  }
)
</script>

<style scoped>
/* 确保容器占满所有可用空间 */
.terminal-container {
  width: 100%;
  height: 100%;
}
</style>
