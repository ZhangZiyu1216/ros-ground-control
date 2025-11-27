<!-- src/renderer/src/components/TopicMonitor.vue -->
<template>
  <div class="topic-monitor-container">
    <div class="toolbar">
      <el-button type="primary" size="small" @click="addItem">
        <el-icon><Plus /></el-icon> 新增监控卡片
      </el-button>
      <el-button size="small" @click="saveLayout(true)">保存布局</el-button>
      <el-button size="small" @click="resetLayout">重置</el-button>
      <div class="hint">布局自动保存</div>
    </div>

    <div class="grid-canvas">
      <GridLayout
        v-if="layout.length > 0"
        v-model:layout="layout"
        :col-num="12"
        :row-height="30"
        :is-draggable="true"
        :is-resizable="true"
        :vertical-compact="true"
        :use-css-transforms="true"
      >
        <GridItem
          v-for="item in layout"
          :key="item.i"
          :x="item.x"
          :y="item.y"
          :w="item.w"
          :h="item.h"
          :i="item.i"
          class="grid-item-wrapper"
        >
          <!-- 这里的 config 包含了 backendId 和 topicName -->
          <MonitorCard
            :config="item.config"
            @update:config="(val) => updateItemConfig(item, val)"
            @remove="removeItem(item.i)"
          />
        </GridItem>
      </GridLayout>
      <el-empty v-else description="点击上方新增按钮开始" />
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch } from 'vue'
import { GridLayout, GridItem } from 'grid-layout-plus'
import { Plus } from '@element-plus/icons-vue'
import MonitorCard from '../components/monitor/MonitorCard.vue'
import { ElMessage } from 'element-plus'

// 全局配置 Key
const GLOBAL_CONFIG_KEY = 'topic_monitor_settings'

const layout = ref([])

// 加载布局
async function loadLayout() {
  const saved = await window.api.getConfig(GLOBAL_CONFIG_KEY)
  if (saved && Array.isArray(saved) && saved.length > 0) {
    layout.value = saved
  } else {
    // 默认空布局
    layout.value = []
    addItem() // 默认加一个
  }
}

// 保存布局 (修复 Clone 错误)
async function saveLayout(manual = false) {
  try {
    // 【修复】深拷贝去除 Vue Proxy 和 Grid Layout 的内部属性
    // 仅保留我们需要持久化的数据
    const rawData = layout.value.map((item) => ({
      x: item.x,
      y: item.y,
      w: item.w,
      h: item.h,
      i: item.i,
      config: item.config || {} // 保存卡片内部配置 (backendId, topicName)
    }))

    await window.api.setConfig(GLOBAL_CONFIG_KEY, JSON.parse(JSON.stringify(rawData)))
    if (manual) ElMessage.success('布局已保存')
  } catch (e) {
    console.error('Save failed:', e)
    if (manual) ElMessage.error('保存失败')
  }
}

// 新增卡片
function addItem() {
  layout.value.push({
    x: (layout.value.length * 6) % 12,
    y: 1000, // Grid Layout 会自动放到最下面
    w: 6,
    h: 8,
    i: Date.now().toString(),
    config: { backendId: '', topicName: '' } // 初始空配置
  })
}

function removeItem(id) {
  const idx = layout.value.findIndex((item) => item.i === id)
  if (idx !== -1) layout.value.splice(idx, 1)
}

function updateItemConfig(item, newConfig) {
  item.config = newConfig
}

function resetLayout() {
  layout.value = []
  addItem()
}

// 自动保存
let timer = null
watch(
  layout,
  () => {
    if (timer) clearTimeout(timer)
    timer = setTimeout(() => saveLayout(), 2000)
  },
  { deep: true }
)

onMounted(() => {
  loadLayout()
})
</script>

<style scoped>
.topic-monitor-container {
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  background-color: #111;
}
.toolbar {
  height: 40px;
  background-color: #1e1e1e;
  border-bottom: 1px solid #333;
  display: flex;
  align-items: center;
  padding: 0 15px;
  gap: 10px;
}
.hint {
  font-size: 12px;
  color: #666;
  margin-left: auto;
}
.grid-canvas {
  flex: 1;
  overflow-y: auto;
  overflow-x: hidden;
  padding: 10px;
  background-image: radial-gradient(#333 1px, transparent 1px);
  background-size: 20px 20px;
}
.grid-item-wrapper {
  touch-action: none;
}
</style>
