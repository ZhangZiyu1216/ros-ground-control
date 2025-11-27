<template>
  <div class="log-panel">
    <el-tabs
      v-model="activeTab"
      type="card"
      class="level-2-tabs"
      editable
      @edit="handleTabsEdit"
      @tab-change="handleTabChange"
    >
      <!-- 1. 节点日志 Tabs (数据驱动) -->
      <el-tab-pane
        v-for="nodeId in nodeLogKeys"
        :key="nodeId"
        :name="nodeId"
        :label="nodeId"
        :closable="!isNodeRunning(nodeId)"
      >
        <template #label>
          <span :class="['status-dot', isNodeRunning(nodeId) ? 'running' : 'stopped']"></span>
          {{ nodeId }}
        </template>
        <LogViewer
          :ref="(el) => setViewerRef(nodeId, el)"
          mode="log"
          :lines="getNodeLog(nodeId).lines"
        />
      </el-tab-pane>

      <!-- 2. 交互终端 Tabs -->
      <el-tab-pane
        v-for="term in terminals"
        :key="`term-${term.id}`"
        :name="`term-${term.id}`"
        :label="term.name"
        closable
      >
        <LogViewer
          :ref="(el) => setViewerRef(`term-${term.id}`, el)"
          mode="terminal"
          :host-ip="clientIp"
        />
      </el-tab-pane>

      <!-- 3. “+”号 Tab (利用 Element Plus 的 editable 特性，会自动生成添加按钮) -->
      <!-- 这里不需要显式写一个 Tab-Pane，el-tabs 的 editable="true" 会在头部右侧显示 + 号 -->
    </el-tabs>

    <!-- 空状态指引 (当没有任何 Tab 时显示) -->
    <div v-if="nodeLogKeys.length === 0 && terminals.length === 0" class="empty-guide">
      <el-empty description="暂无活动日志">
        <el-button type="primary" @click="handleTabsEdit(null, 'add')">新建终端</el-button>
      </el-empty>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, nextTick, watch } from 'vue'
import LogViewer from './LogViewer.vue'
import { useLog } from '../../composables/useLog.js'
import { useRobotStore } from '../../store/robot.js'

const props = defineProps({
  backendId: { type: String, required: true }
})

// 使用 Composable
const {
  nodeLogKeys,
  terminals,
  getNodeLog,
  createTerminal,
  closeTerminal,
  closeNodeLog,
  isNodeRunning
} = useLog(() => props.backendId)

const store = useRobotStore()
const activeTab = ref('')
const viewerRefs = ref({})

// 获取 IP 供终端连接
const clientIp = computed(() => {
  const c = store.clients[props.backendId]
  return c ? c.ip : ''
})

const setViewerRef = (key, el) => {
  if (el) viewerRefs.value[key] = el
}

// 处理增删 Tab
const handleTabsEdit = (targetName, action) => {
  if (action === 'add') {
    const id = createTerminal()
    if (id) {
      activeTab.value = `term-${id}`
    }
  } else if (action === 'remove') {
    if (targetName.startsWith('term-')) {
      const id = parseInt(targetName.replace('term-', ''))
      closeTerminal(id)
    } else {
      closeNodeLog(targetName)
    }
  }
}

const handleTabChange = (name) => {
  nextTick(() => {
    const viewer = viewerRefs.value[name]
    if (viewer && viewer.fit) viewer.fit()
  })
}

// 自动激活新 Tab
watch(
  [nodeLogKeys, terminals],
  ([newLogs, newTerms]) => {
    // 如果当前没有激活的，或者激活的被删了
    if (
      !activeTab.value ||
      (!newLogs.includes(activeTab.value) &&
        !newTerms.some((t) => `term-${t.id}` === activeTab.value))
    ) {
      if (newTerms.length > 0) activeTab.value = `term-${newTerms[newTerms.length - 1].id}`
      else if (newLogs.length > 0) activeTab.value = newLogs[0]
    }
  },
  { deep: true }
)
</script>

<style scoped>
.log-panel {
  height: 100%;
  display: flex;
  flex-direction: column;
}
.level-2-tabs {
  height: 100%;
  display: flex;
  flex-direction: column;
}
:deep(.el-tabs__content) {
  flex: 1;
  padding: 0;
  overflow: hidden;
}
:deep(.el-tab-pane) {
  height: 100%;
}
.empty-guide {
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
}
.status-dot {
  display: inline-block;
  width: 6px;
  height: 6px;
  border-radius: 50%;
  margin-right: 5px;
  vertical-align: middle;
}
.status-dot.running {
  background-color: #67c23a;
}
.status-dot.stopped {
  background-color: #909399;
}
</style>
