<template>
  <div class="log-manager-root">
    <!-- 情况1: 没有任何连接 -->
    <el-empty v-if="clientKeys.length === 0" description="请先连接机器人" />

    <!-- 情况2: 有连接，显示一级 Tabs (机器人列表) -->
    <el-tabs v-else v-model="activeRobotTab" type="border-card" class="level-1-tabs">
      <el-tab-pane v-for="key in clientKeys" :key="key" :name="key">
        <template #label>
          <span class="robot-tab-label">
            <el-icon><Connection /></el-icon>
            {{ getClientName(key) }}
          </span>
        </template>

        <!-- 嵌入二级 Tab 组件 -->
        <!-- 使用 keep-alive 保持终端连接和日志状态 -->
        <LogPanel :backend-id="key" />
      </el-tab-pane>
    </el-tabs>
  </div>
</template>

<script setup>
import { ref, computed, watch } from 'vue'
import LogPanel from '../components/log/LogPanel.vue'
import { useRobotStore } from '../store/robot.js'
import { Connection } from '@element-plus/icons-vue'

const props = defineProps({
  // 接收当前 MainView 选中的 ID，用于自动聚焦
  currentBackendId: { type: [String, null], default: null }
})

const store = useRobotStore()
const activeRobotTab = ref('')

// 获取所有已连接机器人的 ID (Key)
const clientKeys = computed(() => Object.keys(store.clients))

const getClientName = (key) => {
  const c = store.clients[key]
  return c ? c.name || c.hostname || c.ip : key
}

// 自动跟随 MainView 的选择
watch(
  () => props.currentBackendId,
  (newId) => {
    if (newId && clientKeys.value.includes(newId)) {
      activeRobotTab.value = newId
    } else if (!activeRobotTab.value && clientKeys.value.length > 0) {
      activeRobotTab.value = clientKeys.value[0]
    }
  },
  { immediate: true }
)
</script>

<style scoped>
.log-manager-root {
  height: 100%;
  display: flex;
  flex-direction: column;
}
.level-1-tabs {
  height: 100%;
  display: flex;
  flex-direction: column;
  border: none;
}
:deep(.el-tabs__content) {
  flex: 1;
  padding: 0;
  /* 这里的 padding 0 很重要，让二级 Tab 贴边 */
}
:deep(.el-tab-pane) {
  height: 100%;
}
.robot-tab-label {
  display: flex;
  align-items: center;
  gap: 5px;
}
</style>
