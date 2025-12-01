<!-- src/renderer/src/components/LogPanel.vue -->
<template>
  <div class="log-panel">
    <!-- 空状态指引 -->
    <div v-if="nodeLogKeys.length === 0 && terminals.length === 0" class="empty-guide">
      <div class="guide-content">
        <el-icon class="guide-icon"><Monitor /></el-icon>
        <p>暂无活动日志</p>
        <el-button type="primary" plain round :disabled="!connected" @click="triggerAddTerminal">
          <el-icon><Plus /></el-icon> 新建交互终端
        </el-button>
      </div>
    </div>
    <el-tabs
      v-else
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
          <div class="tab-label-content">
            <!-- 状态点：增加呼吸动画 -->
            <span :class="['status-dot', isNodeRunning(nodeId) ? 'running' : 'stopped']"></span>
            <span class="tab-text">{{ nodeId }}</span>
          </div>
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
        <template #label>
          <div class="tab-label-content">
            <el-icon class="term-icon"><Platform /></el-icon>
            <!-- 需要引入 Platform 图标 -->
            <span class="tab-text">{{ term.name }}</span>
          </div>
        </template>
        <LogViewer
          :ref="(el) => setViewerRef(`term-${term.id}`, el)"
          mode="terminal"
          :backend-id="backendId"
          :disabled="!connected"
        />
      </el-tab-pane>

      <!-- 3. 自定义添加按钮 Tab -->
      <el-tab-pane name="__add_tab__" :closable="false" :disabled="!connected">
        <template #label>
          <el-tooltip :content="connected ? '新建终端' : '连接断开，无法新建'" placement="top">
            <!-- [修复] 这里给内部容器加上样式，模拟 Tab 外观 -->
            <div class="add-tab-btn-wrapper" :class="{ 'is-disabled': !connected }">
              <el-icon><Plus /></el-icon>
            </div>
          </el-tooltip>
        </template>
      </el-tab-pane>
    </el-tabs>
  </div>
</template>

<script setup>
import { ref, nextTick, watch } from 'vue'
import LogViewer from './LogViewer.vue'
import { useLog } from '../../composables/useLog.js'
import { Plus, Monitor, Platform } from '@element-plus/icons-vue'

const props = defineProps({
  backendId: { type: String, required: true },
  connected: { type: Boolean, default: true }
})

// Composable
const {
  nodeLogKeys,
  terminals,
  getNodeLog,
  createTerminal,
  closeTerminal,
  closeNodeLog,
  isNodeRunning
} = useLog(() => props.backendId)

const activeTab = ref('')
const viewerRefs = ref({})

const setViewerRef = (key, el) => {
  if (el) viewerRefs.value[key] = el
}

const triggerAddTerminal = () => {
  if (!props.connected) return
  const id = createTerminal()
  if (id) {
    nextTick(() => {
      activeTab.value = `term-${id}`
    })
  }
}

// Tab 切换拦截
// eslint-disable-next-line no-unused-vars
const handleBeforeTabLeave = (activeName, oldActiveName) => {
  if (activeName === '__add_tab__') {
    if (!props.connected) return
    triggerAddTerminal()
    return false // 阻止选中
  }
  nextTick(() => {
    const viewer = viewerRefs.value[activeName]
    if (viewer && viewer.fit) viewer.fit()
  })
  return true
}

// Tab 删除
const handleTabRemove = (targetName) => {
  if (targetName.startsWith('term-')) {
    const id = parseInt(targetName.replace('term-', ''))
    closeTerminal(id)
  } else {
    closeNodeLog(targetName)
  }
}

// 自动激活逻辑
watch(
  [nodeLogKeys, terminals],
  ([newLogs, newTerms]) => {
    // 如果当前选中的 Tab 消失了，或者是添加按钮，或者初始为空
    const isActiveInvalid =
      !activeTab.value ||
      activeTab.value === '__add_tab__' ||
      (!newLogs.includes(activeTab.value) &&
        !newTerms.some((t) => `term-${t.id}` === activeTab.value))

    if (isActiveInvalid) {
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
  background-color: var(--bg-color); /* 继承背景色 */
  border-bottom-right-radius: 12px;
  overflow: hidden; /* 防止圆角溢出 */
}

/* 覆盖 Element Plus Tabs 样式，使其更现代化 */
.level-2-tabs {
  height: 100%;
  display: flex;
  flex-direction: column;
  --el-tabs-header-height: 40px;
}

:deep(.el-tabs__header) {
  margin: 0;
  background-color: rgba(0, 0, 0, 0.03); /* 顶部 Tab 栏背景 */
  border-bottom: 1px solid var(--divider-color);
  padding-top: 5px;
  padding-left: 10px;
}

:deep(.el-tabs__nav) {
  border: none !important;
}

:deep(.el-tabs__item) {
  border: none !important;
  background: transparent;
  margin-right: 4px;
  border-radius: 8px 8px 0 0;
  color: var(--text-secondary);
  transition: all 0.2s;
}

:deep(.el-tabs__item.is-active) {
  background-color: var(--bg-color); /* 激活时变成背景色，实现无缝连接 */
  color: var(--el-color-primary);
  font-weight: 600;
  box-shadow: 0 -2px 4px rgba(0, 0, 0, 0.02); /* 微微浮起 */
}

:deep(.el-tabs__item:hover:not(.is-active)) {
  background-color: rgba(0, 0, 0, 0.05);
  color: var(--text-primary);
}

:deep(.el-tabs__content) {
  flex: 1;
  height: 0; /* 配合 flex:1 使用，防止被内部内容撑大溢出 */
  padding: 0;
  overflow: hidden;
  background-color: var(--bg-color, #ffffff);
}

:deep(.el-tab-pane) {
  height: 100%; /* 确保 pane 填满 content */
}

/* Tab 标签内容 */
.tab-label-content {
  display: flex;
  align-items: center;
}
.tab-text {
  margin-left: 4px;
}
.term-icon {
  font-size: 14px;
  vertical-align: middle;
}

/* 状态点动画 */
.status-dot {
  display: inline-block;
  width: 8px;
  height: 8px;
  border-radius: 50%;
  margin-right: 6px;
  transition: all 0.3s;
}
.status-dot.running {
  background-color: #67c23a;
  box-shadow: 0 0 6px rgba(103, 194, 58, 0.6);
  animation: dot-pulse 2s infinite;
}
.status-dot.stopped {
  background-color: #909399;
}

@keyframes dot-pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(103, 194, 58, 0.7);
  }
  70% {
    box-shadow: 0 0 0 4px rgba(103, 194, 58, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(103, 194, 58, 0);
  }
}

.add-tab-btn-wrapper {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 28px;
  height: 24px;
  border-radius: 6px;
  background-color: transparent;
  border: 1px solid transparent;
  color: var(--text-secondary);
  transition: all 0.2s;
}

/* Hover 效果 */
.add-tab-btn-wrapper:not(.is-disabled):hover {
  background-color: transparent;
  color: var(--el-color-primary);
}

/* [修复 Bug 2] 禁用样式 */
.add-tab-btn-wrapper.is-disabled {
  opacity: 0.5;
  cursor: not-allowed;
  pointer-events: none; /* 核心：物理禁止点击 */
  background-color: transparent;
  border: 1px dashed var(--divider-color);
}

/* [修复 Bug 4] 空状态居中 */
.empty-guide {
  /* 核心：flex: 1 撑满高度 */
  flex: 1;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-secondary);
}
.guide-content {
  text-align: center;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 15px;
}
.guide-icon {
  font-size: 48px;
  opacity: 0.2;
}

/* 深色模式适配 */
:global(html.dark) :deep(.el-tabs__header) {
  background-color: rgba(0, 0, 0, 0.2);
}
:global(html.dark) :deep(.el-tabs__item:hover:not(.is-active)) {
  background-color: rgba(255, 255, 255, 0.05);
}
</style>
