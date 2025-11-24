<!-- src/renderer/src/components/TopicMonitor.vue -->
<template>
  <div class="topic-monitor">
    <!-- 控制栏 -->
    <div class="controls">
      <el-select v-model="selectedOptionalTopic" placeholder="从可用话题列表添加监控" filterable>
        <el-option
          v-for="topic in filteredOptionalTopics"
          :key="topic"
          :label="topic"
          :value="topic"
        />
      </el-select>
      <el-button :disabled="!selectedOptionalTopic" @click="addMonitor">添加</el-button>
    </div>

    <!-- 监控卡片网格 -->
    <div class="monitor-grid">
      <el-card v-for="(monitor, topicName) in activeMonitors" :key="topicName" class="monitor-card">
        <template #header>
          <div class="card-header">
            <span class="topic-name">{{ topicName }}</span>
            <el-button type="danger" circle size="small" @click="stopMonitoring(topicName)"
              >X</el-button
            >
          </div>
        </template>
        <!-- Pub/Sub Info -->
        <div>
          <strong>Pubs:</strong>
          <el-tag v-for="p in monitor.publishers" :key="p" size="small">{{ p }}</el-tag>
        </div>
        <div>
          <strong>Subs:</strong>
          <el-tag v-for="s in monitor.subscribers" :key="s" size="small">{{ s }}</el-tag>
        </div>
        <el-divider />
        <!-- Message Data -->
        <div class="message-data">
          <TreeView :data="monitor.message" />
        </div>
      </el-card>
    </div>
  </div>
</template>

<script setup>
import { ref, reactive, onMounted, watch, computed } from 'vue' // 导入 watch
import { useConnection } from '../composables/useConnection.js'
import TreeView from './TreeView.vue'

const props = defineProps({
  currentBackendId: {
    type: String,
    default: null // 默认为空，防止没连接时报错
  }
})

const { rosbridgeStatus } = useConnection()
const activeMonitors = reactive({})
const allAvailableTopics = ref([])
const presetTopicList = ref([])
const pendingTopics = ref(new Set())
const selectedOptionalTopic = ref(null)

const filteredOptionalTopics = computed(() => {
  return allAvailableTopics.value.filter((topic) => !activeMonitors[topic])
})

async function loadConfig() {
  const presetStr = await window.api.getConfig('presetTopics', props.backendId)
  presetTopicList.value = presetStr
    ? presetStr
        .split(',')
        .map((t) => t.trim())
        .filter(Boolean)
    : []
}

async function fetchAllTopics() {
  if (rosbridgeStatus.value !== 'connected') {
    allAvailableTopics.value = []
    return
  }
  allAvailableTopics.value = await window.api.getAllTopics(props.backendId)
}

onMounted(async () => {
  await loadConfig()
  window.api.onTopicMessage((data) => {
    if (activeMonitors[data.topic]) {
      activeMonitors[data.topic].message = data.message
    }
  })
  setInterval(() => {
    updateAllDetails()
    retryPendingSubscriptions()
    fetchAllTopics()
  }, 3000)
  window.api.onConfigUpdated(async (update) => {
    if (update.key === 'presetTopics') {
      await loadConfig()
      presetTopicList.value.forEach((topic) => pendingTopics.value.add(topic))
    }
  })
})

watch(
  rosbridgeStatus,
  (newStatus) => {
    if (newStatus === 'connected') {
      fetchAllTopics()
      presetTopicList.value.forEach((topic) => pendingTopics.value.add(topic))
      retryPendingSubscriptions()
    } else {
      Object.keys(activeMonitors).forEach((topicName) => delete activeMonitors[topicName])
      pendingTopics.value.clear()
      allAvailableTopics.value = []
    }
  },
  { immediate: true }
)

async function startMonitoring(topicName) {
  if (!topicName || rosbridgeStatus.value !== 'connected') return

  // 如果卡片不存在，先创建它
  if (!activeMonitors[topicName]) {
    activeMonitors[topicName] = {
      message: { status: 'Subscribing...' },
      publishers: [],
      subscribers: []
    }
  }

  // 调用后端的订阅函数，并获取结果
  const success = await window.api.subscribeToTopic(topicName, props.backendId)

  if (success) {
    // 订阅成功，从待办列表移除，并更新详情
    pendingTopics.value.delete(topicName)
    await updateDetails(topicName)
  } else {
    // 订阅失败，把它加回待办列表（确保它在里面）
    pendingTopics.value.add(topicName)
  }
}

function retryPendingSubscriptions() {
  if (rosbridgeStatus.value !== 'connected' || pendingTopics.value.size === 0) return
  console.log('Retrying pending subscriptions:', [...pendingTopics.value])
  for (const topicName of pendingTopics.value) {
    startMonitoring(topicName)
  }
}

function addMonitor() {
  if (selectedOptionalTopic.value) {
    startMonitoring(selectedOptionalTopic.value)
    selectedOptionalTopic.value = null
  }
}

async function stopMonitoring(topicName) {
  pendingTopics.value.delete(topicName)
  delete activeMonitors[topicName]
  await window.api.unsubscribeFromTopic(topicName, props.backendId)
}

async function updateDetails(topicName) {
  if (!activeMonitors[topicName] || rosbridgeStatus.value !== 'connected') return
  const details = await window.api.getTopicDetails(topicName, props.backendId)
  activeMonitors[topicName].publishers = details.publishers
  activeMonitors[topicName].subscribers = details.subscribers
}

function updateAllDetails() {
  if (rosbridgeStatus.value !== 'connected') return
  for (const topicName in activeMonitors) {
    updateDetails(topicName)
  }
}
</script>

<style scoped>
/* ... 添加一些美化样式 ... */
.topic-monitor {
  padding: 10px;
}
.controls {
  margin-bottom: 20px;
  display: flex;
  gap: 10px;
}
.monitor-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(350px, 1fr));
  gap: 15px;
}
.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.topic-name {
  font-weight: bold;
  font-family: monospace;
}
.message-data {
  background-color: #2a2a2a;
  padding: 10px;
  border-radius: 4px;
  margin-top: 10px;
  max-height: 300px;
  overflow-y: auto;
}
</style>
