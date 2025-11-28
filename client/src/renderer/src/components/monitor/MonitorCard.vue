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
            :label="robot.name"
            :value="robot.id"
          />
        </el-select>

        <!-- 2. 选择话题 -->
        <!-- 增加 :key 强制重渲染 -->
        <el-select
          :key="localConfig.backendId"
          v-model="currentSelectId"
          placeholder="选择话题"
          size="small"
          filterable
          class="topic-select"
          :disabled="!localConfig.backendId"
          @change="handleTopicChange"
          @mousedown.stop
        >
          <el-option
            v-for="ch in sortedAndFilteredChannels"
            :key="ch.id"
            :label="ch.topic"
            :value="ch.id"
          >
            <span style="float: left">{{ ch.topic }}</span>
            <span style="float: right; color: #666; font-size: 10px; margin-left: 10px">{{
              formatType(ch.schemaName)
            }}</span>
          </el-option>
        </el-select>

        <span v-if="messageRate > 0" class="rate-tag">{{ messageRate }} Hz</span>
      </div>

      <div class="header-right" @mousedown.stop>
        <el-icon class="close-btn" @click="$emit('remove')"><Close /></el-icon>
      </div>
    </div>

    <div class="card-body" @mousedown.stop>
      <el-empty v-if="!localConfig.backendId" description="请选择机器人" :image-size="60" />
      <el-empty v-else-if="!localConfig.topicName" description="请选择话题" :image-size="60" />

      <component :is="visualizerComponent" v-else :data="visualizedData" :topic-type="topicType" />
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

// 用于绑定 el-select 的中间变量 (ID)
const currentSelectId = ref(null)

const activeRobots = computed(() => {
  return Object.entries(robotStore.clients).map(([key, client]) => ({
    id: key,
    name: client.name || client.ip || key
  }))
})

// --- 话题列表数据流 ---

// 1. 获取响应式引用
const topicsRef = computed(() => {
  if (!localConfig.backendId) return []
  return getTopicsRef(localConfig.backendId).value
})

// 2. 排序与过滤
const sortedAndFilteredChannels = computed(() => {
  const list = topicsRef.value || []

  const filtered = list.filter((ch) => {
    const isNotRawImage = ch.schemaName !== 'sensor_msgs/Image'

    // 【调试修改】暂时放宽条件
    // 如果 isAlive 为 undefined (旧逻辑) 或者 true (新逻辑)，都显示
    // 甚至可以暂时去掉 isAlive 的判断，只看 isNotRawImage

    // 原逻辑：
    const isAlive = ch.isAlive === true
    const isSelected = ch.topic === localConfig.topicName
    return isNotRawImage && (isAlive || isSelected)

    // 新逻辑 (配合 useFoxglove 的乐观策略)：
    // useFoxglove 保证了如果没有 graph update，isAlive 默认为 true
    //return isNotRawImage && ch.isAlive !== false
  })

  return filtered.sort((a, b) => a.topic.localeCompare(b.topic))
})

// 3. [关键修复] 监听配置变化，同步 Select 显示 ID
watch(
  () => localConfig.topicName,
  (newName) => {
    if (!newName) {
      currentSelectId.value = null
      return
    }
    // 尝试在当前列表中找对应的 ID
    const match = sortedAndFilteredChannels.value.find((ch) => ch.topic === newName)
    if (match) {
      currentSelectId.value = match.id
    } else {
      // 如果名字有，但列表里没有（说明话题还没出现或已消亡），
      // 我们保留 Select 显示为空，或者显示 Name（el-select 不支持 value 不在 option 里）
      // 这里选择置空，等待话题出现
      currentSelectId.value = null
    }
  },
  { immediate: true }
)

// 4. [关键修复] 监听列表变化，自动匹配或清空
watch(
  sortedAndFilteredChannels,
  (newList) => {
    // 如果当前选中的话题名字还在，更新 ID (防止重连后 ID 变了)
    if (localConfig.topicName) {
      const match = newList.find((ch) => ch.topic === localConfig.topicName)
      if (match) {
        currentSelectId.value = match.id
        // 确保订阅处于激活状态 (如果是重连恢复的情况)
        setupSubscription()
      } else {
        // 话题消失了！
        // 这种情况下，保持 topicName 不变（等待恢复），但 Select UI 变空
        currentSelectId.value = null
      }
    }
  },
  { deep: true }
)

// --- 数据展示逻辑 ---

const topicType = computed(() => {
  if (!localConfig.topicName) return ''
  // 优先用实时的类型，其次用配置的
  const ch = sortedAndFilteredChannels.value.find((c) => c.topic === localConfig.topicName)
  return ch?.schemaName || localConfig.topicType || ''
})

const visualizerComponent = computed(() => {
  if (topicType.value.toLowerCase().includes('image')) return ImageVisualizer
  return DataVisualizer
})

const formatType = (name) => (name ? name.split('/').pop() : '')

// --- 订阅管理 ---
let cleanupSub = null

function setupSubscription() {
  // 防抖：如果配置没变且已订阅，跳过 (useFoxglove 内部有计数，重复调用也没事，但为了性能)
  if (cleanupSub) cleanupSub()

  if (localConfig.backendId && localConfig.topicName) {
    cleanupSub = subscribe(localConfig.backendId, localConfig.topicName, localConfig.topicType)
  }
}

const realTimeMessage = computed(() => {
  return getMessage(localConfig.backendId, localConfig.topicName)
})

// --- 渲染节流 ---
const visualizedData = ref(null)
const messageCount = ref(0)
const messageRate = ref(0)
const lastRateCheck = ref(Date.now())
const lastRenderTime = ref(0)
const RENDER_INTERVAL = 100

watch(realTimeMessage, (newMsg) => {
  const now = Date.now()
  messageCount.value++
  if (now - lastRateCheck.value > 1000) {
    messageRate.value = messageCount.value
    messageCount.value = 0
    lastRateCheck.value = now
  }
  if (now - lastRenderTime.value > RENDER_INTERVAL || !newMsg) {
    visualizedData.value = newMsg
    lastRenderTime.value = now
  }
})

// --- 交互处理 ---

function handleRobotChange() {
  localConfig.topicName = ''
  localConfig.topicType = ''
  currentSelectId.value = null
  if (cleanupSub) cleanupSub()
  updateConfig()
}

function handleTopicChange(newId) {
  // 用户在下拉框选了 ID，我们需要存的是 Name
  const ch = sortedAndFilteredChannels.value.find((c) => c.id === newId)
  if (ch) {
    localConfig.topicName = ch.topic
    localConfig.topicType = ch.schemaName
    // setupSubscription 会由 watch(localConfig.topicName) 触发吗？
    // 不会，因为我们是在这里同步修改的。需要手动触发或等待 watch。
    // 为了更直接的响应，手动调用：
    setupSubscription()
    updateConfig()
  }
}

function updateConfig() {
  emit('update:config', {
    backendId: localConfig.backendId,
    topicName: localConfig.topicName,
    topicType: localConfig.topicType
  })
}

// 初始挂载
if (localConfig.topicName) setupSubscription()

onUnmounted(() => {
  if (cleanupSub) cleanupSub()
})
</script>

<style scoped>
/* 保持原有样式 */
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

/* 滚动条 */
.card-body::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}
.card-body::-webkit-scrollbar-thumb {
  background: #444;
  border-radius: 3px;
}
</style>
