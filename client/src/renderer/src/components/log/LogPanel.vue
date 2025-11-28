<!-- src/renderer/src/components/LogPanel.vue -->
<template>
  <div class="log-panel">
    <el-tabs
      v-model="activeTab"
      type="card"
      class="level-2-tabs"
      :before-leave="handleBeforeTabLeave"
      @tab-remove="handleTabRemove"
    >
      <!-- 1. 节点日志 Tabs -->
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

      <!-- 3. 【修复】自定义添加按钮 Tab -->
      <!-- 这是一个伪 Tab，不可关闭，点击它会触发添加逻辑但不会真的选中 -->
      <el-tab-pane name="__add_tab__" :closable="false">
        <template #label>
          <span class="add-tab-btn">
            <el-icon><Plus /></el-icon>
          </span>
        </template>
      </el-tab-pane>
    </el-tabs>

    <!-- 空状态指引 -->
    <div v-if="nodeLogKeys.length === 0 && terminals.length === 0" class="empty-guide">
      <el-empty description="暂无活动日志">
        <el-button type="primary" @click="triggerAddTerminal">新建终端</el-button>
      </el-empty>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, nextTick, watch } from 'vue'
import LogViewer from './LogViewer.vue'
import { useLog } from '../../composables/useLog.js'
import { useRobotStore } from '../../store/robot.js'
import { Plus } from '@element-plus/icons-vue'

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

const triggerAddTerminal = () => {
  const id = createTerminal()
  if (id) {
    // 等待 DOM 更新后切换 Tab
    nextTick(() => {
      activeTab.value = `term-${id}`
    })
  }
}

// 【修复】拦截 Tab 切换
// eslint-disable-next-line no-unused-vars
const handleBeforeTabLeave = (activeName, oldActiveName) => {
  // 如果目标是添加按钮
  if (activeName === '__add_tab__') {
    // 执行添加逻辑
    triggerAddTerminal()
    // 返回 false 阻止 Tab 真的切换到 __add_tab__
    return false
  }

  // 正常的切换，顺便 fit 一下目标终端
  nextTick(() => {
    const viewer = viewerRefs.value[activeName]
    if (viewer && viewer.fit) viewer.fit()
  })

  return true
}

// 处理删除 (原 handleTabsEdit 拆分出的逻辑)
const handleTabRemove = (targetName) => {
  if (targetName.startsWith('term-')) {
    const id = parseInt(targetName.replace('term-', ''))
    closeTerminal(id)
  } else {
    closeNodeLog(targetName)
  }
}

// 自动激活新 Tab
watch(
  [nodeLogKeys, terminals],
  ([newLogs, newTerms]) => {
    // 如果当前没有激活的，或者激活的被删了，且不是 '__add_tab__'
    if (
      !activeTab.value ||
      activeTab.value === '__add_tab__' ||
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
.add-tab-btn {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 100%;
  height: 100%;
  font-size: 16px;
}
</style>
