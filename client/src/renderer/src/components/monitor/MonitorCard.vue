<!-- src/renderer/src/components/monitor/MonitorCard.vue -->
<template>
  <div class="monitor-card-container">
    <div class="card-header drag-handle">
      <div class="header-left">
        <!-- 1. 选择机器人 -->
        <el-select
          v-model="localConfig.backendId"
          placeholder="选择机器人"
          size="small"
          class="robot-select"
          @change="handleRobotChange"
          @mousedown.stop
        >
          <el-option
            v-for="robot in activeRobots"
            :key="robot.id"
            :label="robot.name || robot.ip"
            :value="robot.id"
          />
        </el-select>

        <!-- 2. 选择话题 -->
        <el-select
          v-model="localConfig.topicId"
          placeholder="选择话题"
          size="small"
          filterable
          class="topic-select"
          :disabled="!localConfig.backendId"
          @change="handleTopicChange"
          @mousedown.stop
        >
          <el-option v-for="ch in availableChannels" :key="ch.id" :label="ch.topic" :value="ch.id">
            <span>{{ ch.topic }}</span>
          </el-option>
        </el-select>

        <span v-if="messageRate > 0" class="rate-tag">{{ messageRate }} Hz</span>
      </div>

      <div class="header-right" @mousedown.stop>
        <el-icon class="close-btn" @click="$emit('remove')"><Close /></el-icon>
      </div>
    </div>

    <div class="card-body" @mousedown.stop>
      <!-- 如果没连接 -->
      <el-empty v-if="!localConfig.backendId" description="请选择机器人" :image-size="60" />
      <el-empty v-else-if="!localConfig.topicId" description="请选择话题" :image-size="60" />

      <!-- 数据展示 -->
      <component :is="visualizerComponent" v-else :data="messageData" :topic-type="topicType" />
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, onUnmounted, reactive } from 'vue'
import { Close } from '@element-plus/icons-vue'
import { useRobotStore } from '../../store/robot'
import { useFoxglove } from '../../composables/useFoxglove'
import DataVisualizer from './DataVisualizer.vue'
import ImageVisualizer from './ImageVisualizer.vue'

const props = defineProps({
  config: Object // { backendId, topicId, topicName }
})
const emit = defineEmits(['remove', 'update:config'])

const robotStore = useRobotStore()
const { getTopics, subscribe, getMessage } = useFoxglove()

// 本地配置副本
const localConfig = reactive({
  backendId: props.config.backendId || '',
  topicId: null, // ID 每次连接可能变，不从 props 恢复，而是通过 topicName 恢复
  topicName: props.config.topicName || ''
})

// 获取所有已配置的机器人
const activeRobots = computed(() => Object.values(robotStore.clients))

// 获取当前选中机器人的话题列表
const availableChannels = computed(() => {
  if (!localConfig.backendId) return []
  return getTopics(localConfig.backendId)
})

// 自动恢复逻辑：当话题列表加载后，尝试根据 topicName 找回 topicId
watch(
  availableChannels,
  (channels) => {
    if (localConfig.topicName && !localConfig.topicId) {
      const match = channels.find((c) => c.topic === localConfig.topicName)
      if (match) {
        localConfig.topicId = match.id
        setupSubscription() // 恢复订阅
      }
    }
  },
  { deep: true }
)

// 当前话题类型
const topicType = computed(() => {
  if (!localConfig.topicId) return ''
  const ch = availableChannels.value.find((c) => c.id === localConfig.topicId)
  return ch?.schemaName || ''
})

const visualizerComponent = computed(() => {
  const type = (topicType.value || '').toLowerCase()
  if (type.includes('image')) return ImageVisualizer
  return DataVisualizer
})

// 数据订阅管理
let cleanupSub = null

function setupSubscription() {
  if (cleanupSub) cleanupSub()
  if (localConfig.backendId && localConfig.topicId) {
    cleanupSub = subscribe(localConfig.backendId, localConfig.topicId)
  }
}

// 获取实时消息
const messageData = computed(() => {
  return getMessage(localConfig.backendId, localConfig.topicId)
})

// 事件处理
function handleRobotChange() {
  localConfig.topicId = null
  localConfig.topicName = ''
  if (cleanupSub) cleanupSub()
  updateConfig()
}

function handleTopicChange(newId) {
  const ch = availableChannels.value.find((c) => c.id === newId)
  if (ch) {
    localConfig.topicName = ch.topic
    setupSubscription()
    updateConfig()
  }
}

function updateConfig() {
  // 只向上层传需要持久化的数据 (ID, TopicName)
  emit('update:config', {
    backendId: localConfig.backendId,
    topicName: localConfig.topicName
  })
}

// 频率计算
const messageCount = ref(0)
const messageRate = ref(0)
const lastTime = ref(Date.now())
watch(messageData, () => {
  messageCount.value++
  const now = Date.now()
  if (now - lastTime.value > 1000) {
    messageRate.value = messageCount.value
    messageCount.value = 0
    lastTime.value = now
  }
})

onUnmounted(() => {
  if (cleanupSub) cleanupSub()
})
</script>

<style scoped>
/* 复用之前的 CSS，稍微增加Select宽度 */
.monitor-card-container {
  width: 100%;
  height: 100%;
  background: #1e1e1e;
  border: 1px solid #333;
  border-radius: 6px;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}
.card-header {
  height: 36px;
  background: #2d2d2d;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 8px;
  cursor: move;
  border-bottom: 1px solid #333;
}
.header-left {
  display: flex;
  align-items: center;
  gap: 6px;
  flex: 1;
  min-width: 0;
}
.robot-select {
  width: 110px;
}
.topic-select {
  flex: 1;
  min-width: 100px;
}
.rate-tag {
  font-size: 10px;
  color: #67c23a;
  background: #222;
  padding: 2px 4px;
  border-radius: 2px;
  white-space: nowrap;
}
.close-btn {
  cursor: pointer;
  color: #909399;
}
.close-btn:hover {
  color: #f56c6c;
}
.card-body {
  flex: 1;
  overflow: auto;
  background: #141414;
  position: relative;
}
</style>
