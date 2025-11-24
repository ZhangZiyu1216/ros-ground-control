<!-- src/renderer/src/components/LogManager.vue -->
<template>
  <div class="log-manager-container">
    <el-empty
      v-if="Object.keys(logsByNodeId).length === 0"
      description="没有可监视的节点日志"
    ></el-empty>
    <el-tabs
      v-else
      v-model="activeTab"
      type="card"
      class="log-tabs"
      @tab-remove="closeLogTab"
      @tab-change="handleTabChange"
    >
      <el-tab-pane
        v-for="nodeId in Object.keys(logsByNodeId)"
        :key="`${props.backendId}-${nodeId}`"
        :label="nodeId"
        :name="nodeId"
        :closable="!runningNodes.includes(nodeId)"
      >
        <LogViewer :ref="(el) => (logViewerRefs[nodeId] = el)" :logs="logsByNodeId[nodeId]" />
      </el-tab-pane>
    </el-tabs>
  </div>
</template>

<script setup>
import { ref, reactive, onMounted, watch, nextTick } from 'vue'
import LogViewer from './LogViewer.vue'

const props = defineProps({
  backendId: {
    type: String,
    default: null // 默认为空，防止没连接时报错
  }
})

const logsByNodeId = reactive({})
const runningNodes = ref([]) // 只存储正在运行的节点ID
const activeTab = ref(null)
const logViewerRefs = reactive({})

onMounted(async () => {
  // 监听进程状态更新，用于动态增删标签页
  window.api.onProcessStatusUpdate((update) => {
    if (update.backendId !== props.backendId) return

    const { id, status } = update
    const index = runningNodes.value.indexOf(id)

    if (status === 'running') {
      if (index === -1) runningNodes.value.push(id) // 添加到运行列表
      if (!logsByNodeId[id]) logsByNodeId[id] = [] // 确保日志数组存在

      // --- 新的日志分隔提示逻辑 ---
      // 无论节点是首次启动还是重启，都执行以下操作：
      const timestamp = new Date().toLocaleString()
      const line = '━'.repeat(80) // 使用更平滑的线字符
      const title = `▶ NODE '${id}' STARTED AT: ${timestamp}`

      // 使用数组 join 的方式构造多行字符串，更清晰
      const separatorMessage = [
        '\r\n', // 在前面空一行
        '\x1b[1;34m', // --- 开启 亮蓝色 ---
        line, // 上边框
        `\r\n  ${title}\r\n`, // 中间带图标和信息的文本
        line, // 下边框
        '\x1b[0m', // --- 重置所有颜色 ---
        '\r\n\r\n' // 在后面空两行，拉开与后续日志的距离
      ].join('')

      const separatorLog = {
        type: 'system',
        source: id,
        message: separatorMessage
      }

      logsByNodeId[id].push(separatorLog)
      if (!activeTab.value) {
        activeTab.value = id
        nextTick(() => handleTabChange(id))
      }
    } else if (status === 'stopped' || status === 'crashed' || status === 'error') {
      if (index !== -1) {
        runningNodes.value.splice(index, 1) // 从运行列表中移除
      }
    }
  })

  // 监听所有日志消息，并路由到对应的数组
  window.api.onLogMessage((log) => {
    if (log.backendId !== props.backendId) return
    const { source } = log
    if (source && logsByNodeId[source]) {
      logsByNodeId[source].push(log)
    }
    if (source) {
      if (!logsByNodeId[source]) {
        logsByNodeId[source] = []
      }
      logsByNodeId[source].push(log)
    }
  })

  window.api.onWindowResized(() => {
    // 当窗口大小变化时，我们也调用 handleTabChange
    // 因为它内部的逻辑（找到当前活动的 viewer 并调用 fit）正是我们需要的
    // 我们传入当前活动的 tab ID
    if (activeTab.value) {
      handleTabChange(activeTab.value)
    }
  })
})

watch(
  () => props.backendId,
  (newId, oldId) => {
    if (newId !== oldId) {
      // 1. 清空数据源 (这会让 v-for 列表暂时清空)
      // 如果 logsByNodeId 是 reactive({})
      for (const key in logsByNodeId) delete logsByNodeId[key]
      // 如果 logsByNodeId 是 ref({})
      // logsByNodeId.value = {}
      // 2. 清空运行状态
      runningNodes.value = []
      // 3. 清空激活状态
      activeTab.value = ''
      // 4. 清空 Refs 引用 (防止持有旧的组件引用)
      logViewerRefs.value = {}
    }
  }
)

function closeLogTab(targetName) {
  const tabKeys = Object.keys(logsByNodeId)
  const currentIndex = tabKeys.indexOf(targetName)

  delete logsByNodeId[targetName]
  delete logViewerRefs[targetName]
  // 从正在运行的节点列表中移除
  const index = runningNodes.value.indexOf(targetName)
  if (index > -1) {
    runningNodes.value.splice(index, 1)
  }
  // 如果关闭的是当前激活的标签页，则切换到下一个
  if (activeTab.value === targetName) {
    const remainingTabs = Object.keys(logsByNodeId)
    if (remainingTabs.length === 0) {
      // 全部关闭了
      activeTab.value = null
    } else if (currentIndex < tabKeys.length - 1) {
      // 不是关闭时的最后一页，则切换到下一个
      activeTab.value = tabKeys[currentIndex + 1] || remainingTabs[remainingTabs.length - 1]
    } else {
      // 是关闭时最后一页，切换到关闭后的最后一个
      activeTab.value = remainingTabs[remainingTabs.length - 1]
    }
  }
}

function handleTabChange(activeTabName) {
  // 当标签页切换时，等待DOM更新完成
  nextTick(() => {
    const viewerInstance = logViewerRefs[activeTabName]
    if (viewerInstance && typeof viewerInstance.fitTerminal === 'function') {
      // 调用子组件暴露出的方法
      viewerInstance.fitTerminal()
    }
  })
}
</script>

<style scoped>
.log-manager-container {
  height: 100%;
  display: flex;
  flex-direction: column;
}

.log-tabs {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
}

.log-tabs :deep(.el-tabs__content) {
  flex-grow: 1;
  padding: 0;
}

.log-tabs :deep(.el-tab-pane) {
  height: 100%;
}

.el-empty {
  flex-grow: 1;
}
</style>
