<template>
  <div class="monitor-card-container">
    <!-- Header: 拖拽手柄 -->
    <div class="card-header drag-handle">
      <div class="header-controls">
        <!-- 1. 机器人选择 -->
        <el-select
          v-model="localConfig.backendId"
          placeholder="Bot"
          size="small"
          class="seamless-select robot-select"
          :teleported="false"
          @change="handleRobotChange"
          @mousedown.stop
        >
          <template #prefix><span class="label-prefix">主机</span></template>
          <el-option
            v-for="robot in activeRobots"
            :key="robot.id"
            :label="robot.name"
            :value="robot.id"
          />
        </el-select>

        <el-divider direction="vertical" />

        <!-- 2. 话题选择 -->
        <el-select
          :key="localConfig.backendId"
          v-model="currentSelectId"
          placeholder="Select Topic"
          size="small"
          filterable
          class="seamless-select topic-select"
          :disabled="!localConfig.backendId"
          :teleported="false"
          @change="handleTopicChange"
          @mousedown.stop
        >
          <template #prefix><span class="label-prefix">话题</span></template>
          <el-option
            v-for="ch in sortedAndFilteredChannels"
            :key="ch.id"
            :label="ch.topic"
            :value="ch.id"
          >
            <span class="option-topic">{{ ch.topic }}</span>
            <span class="option-type">{{ formatType(ch.schemaName) }}</span>
          </el-option>
        </el-select>
      </div>

      <div class="header-actions" @mousedown.stop>
        <el-tag v-if="messageRate > 0" size="small" type="success" effect="dark" class="rate-tag">
          {{ messageRate }} Hz
        </el-tag>
        <div class="close-btn" @click="$emit('remove')">
          <el-icon><Close /></el-icon>
        </div>
      </div>
    </div>

    <!-- Body: 内容显示 -->
    <div class="card-body" @mousedown.stop>
      <div v-if="!localConfig.backendId" class="placeholder-state">
        <el-icon class="icon"><Connection /></el-icon>
        <span>请先选择机器人</span>
      </div>
      <div v-else-if="!localConfig.topicName" class="placeholder-state">
        <el-icon class="icon"><ChatLineSquare /></el-icon>
        <span>请选择监控话题</span>
      </div>

      <component :is="visualizerComponent" v-else :data="visualizedData" :topic-type="topicType" />
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, onUnmounted, reactive } from 'vue'
import { Close, Connection, ChatLineSquare } from '@element-plus/icons-vue'
import { useRobotStore } from '../../store/robot'
import { useFoxglove } from '../../composables/useFoxglove'
import DataVisualizer from './DataVisualizer.vue'
import ImageVisualizer from './ImageVisualizer.vue'

const props = defineProps({
  config: Object
})
const emit = defineEmits(['remove', 'update:config'])

const robotStore = useRobotStore()
const { getTopicsRef, subscribe, getMessage } = useFoxglove()

const localConfig = reactive({
  backendId: props.config.backendId || '',
  topicName: props.config.topicName || '',
  topicType: props.config.topicType || ''
})

const currentSelectId = ref(null)

const activeRobots = computed(() => {
  return Object.entries(robotStore.clients).map(([key, client]) => ({
    id: key,
    name: client.name || client.ip || key
  }))
})

// --- 话题列表 ---
const topicsRef = computed(() => {
  if (!localConfig.backendId) return []
  return getTopicsRef(localConfig.backendId).value
})

const sortedAndFilteredChannels = computed(() => {
  const list = topicsRef.value || []
  return list
    .filter((ch) => {
      // 1. 过滤掉 Raw Image (性能原因)
      if (ch.schemaName === 'sensor_msgs/Image') return false
      // 2. 仅显示“活跃”的话题，或者“当前已选中”的话题
      // 如果话题死了 (pubCount=0)，ch.isAlive 会是 false，此时应该隐藏，除非它正是用户正在看的话题
      return ch.isAlive || ch.topic === localConfig.topicName
    })
    .sort((a, b) => a.topic.localeCompare(b.topic))
})

// --- Watchers for State Sync ---

// 1. 列表变动时，检查当前选中话题是否还存在/活跃
// 1. 列表变动时，检查当前选中话题是否还存在/活跃
watch(
  sortedAndFilteredChannels,
  (list) => {
    if (!localConfig.topicName) return

    const match = list.find((ch) => ch.topic === localConfig.topicName)
    if (match) {
      if (currentSelectId.value !== match.id) {
        currentSelectId.value = match.id
      }
      // 如果话题从死变活，或者 ID 变了，刷新订阅
      setupSubscription()
    } else {
      // [说明] 进入这里意味着话题既不活跃，也不是当前选中的话题（但这不可能，因为 filter 保留了当前选中）
      // 唯一的情况是：后端发了 Unadvertise（彻底移除了定义），此时列表中真的没有了
      // 我们选择清空 select ID 显示，提示用户话题已失效，但保留 config，等待复活
      currentSelectId.value = null
    }
  },
  { deep: true, immediate: true }
)

// 2. 监听 Config Name 变化 (来自 Select 选中)
watch(
  () => localConfig.topicName,
  (newName) => {
    if (!newName) {
      currentSelectId.value = null
      return
    }
    // 仅在手动设置时尝试匹配 ID
    const match = sortedAndFilteredChannels.value.find((ch) => ch.topic === newName)
    if (match) currentSelectId.value = match.id
  }
)

// --- Visualizer ---
const visualizerComponent = computed(() => {
  const type = localConfig.topicType || ''
  if (type.toLowerCase().includes('image') || type.toLowerCase().includes('compressed')) {
    return ImageVisualizer
  }
  return DataVisualizer
})

const formatType = (name) => (name ? name.split('/').pop() : '')

// --- Subscription ---
let cleanupSub = null

function setupSubscription() {
  // 如果已经在订阅同一个，先不乱动，交给 useFoxglove 内部去处理去重
  // 但为了安全，如果 backend 或 topic 变了，必须重建
  if (cleanupSub) {
    cleanupSub()
    cleanupSub = null
  }

  if (localConfig.backendId && localConfig.topicName) {
    cleanupSub = subscribe(localConfig.backendId, localConfig.topicName)
  }
}

// Data Stream
const realTimeMessage = computed(() => {
  return getMessage(localConfig.backendId, localConfig.topicName)
})

const visualizedData = ref(null)
const messageRate = ref(0)
let msgCount = 0
let lastRateTime = Date.now()
let lastRenderTime = 0

watch(realTimeMessage, (newMsg) => {
  if (!newMsg) return
  const now = Date.now()
  msgCount++

  // Hz Calc
  if (now - lastRateTime > 1000) {
    messageRate.value = msgCount
    msgCount = 0
    lastRateTime = now
  }

  // Throttle Render (10 FPS)
  if (now - lastRenderTime > 100) {
    visualizedData.value = newMsg
    lastRenderTime = now
  }
})

// --- Handlers ---
function handleRobotChange() {
  localConfig.topicName = ''
  localConfig.topicType = ''
  currentSelectId.value = null
  if (cleanupSub) cleanupSub()
  updateConfig()
}

function handleTopicChange(newId) {
  // 从 ID 反查 Name
  const ch = sortedAndFilteredChannels.value.find((c) => c.id === newId)
  if (ch) {
    localConfig.topicName = ch.topic
    localConfig.topicType = ch.schemaName
    setupSubscription()
    updateConfig()
  }
}

function updateConfig() {
  emit('update:config', { ...localConfig })
}

onUnmounted(() => {
  if (cleanupSub) cleanupSub()
})
</script>

<style scoped>
.monitor-card-container {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  /* 卡片风格 */
  background: var(--panel-bg-color);
  border-radius: 12px;
  box-shadow: 0 4px 12px var(--panel-shadow);
  border: 1px solid var(--panel-border-color);
  transition: all 0.2s;
}

.monitor-card-container:hover {
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.12);
  transform: translateY(-1px);
}

/* --- Header --- */
.card-header {
  height: 36px;
  /* 渐变底色 */
  background: linear-gradient(to bottom, rgba(0, 0, 0, 0.02), rgba(0, 0, 0, 0.05));
  border-bottom: 1px solid rgba(0, 0, 0, 0.05);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 8px;
  cursor: grab;
  user-select: none;
}
.card-header:active {
  cursor: grabbing;
}

:global(html.dark) .card-header {
  background: #252525;
  border-bottom-color: #333;
}

.header-controls {
  display: flex;
  align-items: center;
  gap: 8px;
  flex: 1;
  min-width: 0; /* 防止撑破 */
}

/* 无边框 Select 样式 */
.seamless-select {
  --el-input-border-color: transparent;
  --el-input-hover-border-color: transparent;
  --el-input-focus-border-color: transparent;
  --el-fill-color-blank: transparent;
}
.seamless-select .label-prefix {
  font-size: 10px;
  font-weight: 800;
  color: #909399;
  margin-right: 4px;
}
/* 调整 Select 内部文字 */
.seamless-select :deep(.el-input__inner) {
  font-size: 12px;
  font-weight: 600;
  color: var(--text-primary, #303133);
}
.robot-select {
  width: 120px;
}
.topic-select {
  flex: 1;
  min-width: 100px;
}

/* 下拉选项样式 */
.option-topic {
  font-weight: 500;
  float: left;
}
.option-type {
  float: right;
  color: #909399;
  font-size: 10px;
  margin-left: 10px;
}

.header-actions {
  display: flex;
  align-items: center;
  gap: 8px;
}

.rate-tag {
  font-family: monospace;
  height: 20px;
  padding: 0 6px;
}

.close-btn {
  width: 20px;
  height: 20px;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #909399;
  cursor: pointer;
  transition: all 0.2s;
}
.close-btn:hover {
  background-color: rgba(245, 108, 108, 0.1);
  color: #f56c6c;
}

/* --- Body --- */
.card-body {
  flex: 1;
  overflow: auto;
  position: relative;
  /* 确保有背景色，防止透明穿透 */
  background: var(--bg-color, #f6f8fa);
}
:global(html.dark) .card-body {
  background: #141414;
}

/* 占位状态 */
.placeholder-state {
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  color: #c0c4cc;
  font-size: 13px;
  gap: 10px;
}
.placeholder-state .icon {
  font-size: 24px;
  opacity: 0.5;
}

/* 滚动条 */
.card-body::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}
.card-body::-webkit-scrollbar-thumb {
  background: rgba(0, 0, 0, 0.1);
  border-radius: 3px;
}
:global(html.dark) .card-body::-webkit-scrollbar-thumb {
  background: rgba(255, 255, 255, 0.1);
}
</style>
