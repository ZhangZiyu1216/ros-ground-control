<!-- src/renderer/src/components/TopicMonitor.vue -->
<template>
  <div class="topic-monitor-container">
    <!-- 悬浮隐形工具栏 (Hover 触发) -->
    <div class="floating-toolbar">
      <!-- 视觉引导条 (装饰用，提示这里可以下拉) -->
      <div class="pull-indicator"></div>

      <div class="toolbar-content">
        <el-button type="primary" class="modern-btn" round @click="addItem">
          <el-icon><Plus /></el-icon> 新增卡片
        </el-button>

        <el-button class="modern-btn" round @click="saveLayout(true)">
          <el-icon><Check /></el-icon> 保存布局
        </el-button>

        <el-button class="modern-btn" round @click="resetLayout">
          <el-icon><Refresh /></el-icon> 重置
        </el-button>
      </div>
    </div>

    <!-- 网格画布 -->
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
        style="width: 100%"
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
          drag-allow-from=".card-header"
        >
          <MonitorCard
            :config="item.config"
            @update:config="(val) => updateItemConfig(item, val)"
            @remove="removeItem(item.i)"
          />
        </GridItem>
      </GridLayout>

      <!-- 空状态 -->
      <div v-else class="empty-state">
        <el-empty description="工作台空空如也">
          <template #extra>
            <p class="empty-hint">鼠标移至顶部以添加卡片</p>
          </template>
        </el-empty>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, watch, toRaw } from 'vue'
import { GridLayout, GridItem } from 'grid-layout-plus'
import { Plus, Check, Refresh } from '@element-plus/icons-vue'
import MonitorCard from '../components/monitor/MonitorCard.vue'
import { ElMessage } from 'element-plus'

const GLOBAL_CONFIG_KEY = 'topic_monitor_settings'
const layout = ref([])

async function loadLayout() {
  try {
    const saved = await window.api.getConfig(GLOBAL_CONFIG_KEY)
    if (saved && Array.isArray(saved) && saved.length > 0) {
      layout.value = saved
    } else {
      // === 初始化默认布局 ===
      const baseId = Date.now().toString()
      layout.value = [
        // 1. 左上：图像显示 (中等大小)
        {
          x: 0,
          y: 0,
          w: 6,
          h: 10,
          i: `${baseId}-cam`,
          config: { backendId: '', topicName: '', topicType: 'sensor_msgs/Image' } // 预设类型提示
        },
        // 2. 左下1：数据卡片 (小)
        {
          x: 0,
          y: 10,
          w: 3,
          h: 6,
          i: `${baseId}-data1`,
          config: { backendId: '', topicName: '', topicType: '' }
        },
        // 3. 左下2：数据卡片 (小)
        {
          x: 3,
          y: 10,
          w: 3,
          h: 6,
          i: `${baseId}-data2`,
          config: { backendId: '', topicName: '', topicType: '' }
        },
        // 4. 右侧：轨迹/3D地图 (大卡片)
        {
          x: 6,
          y: 0,
          w: 6,
          h: 16,
          i: `${baseId}-map`,
          config: { backendId: '', topicName: '', topicType: '' }
        }
      ]
    }
  } catch (e) {
    console.warn('Load layout failed, using default', e)
    // 失败时的兜底（同样使用新布局）
    resetLayout()
  }
}

async function saveLayout(manual = false) {
  try {
    // 深度克隆纯数据，剥离 Proxy
    const rawData = layout.value.map((item) => ({
      x: item.x,
      y: item.y,
      w: item.w,
      h: item.h,
      i: item.i,
      config: toRaw(item.config || {})
    }))

    await window.api.setConfig(GLOBAL_CONFIG_KEY, JSON.parse(JSON.stringify(rawData)))
    if (manual) ElMessage.success('布局已保存')
  } catch (e) {
    console.error('Save layout failed:', e)
    if (manual) ElMessage.error('保存失败')
  }
}

function addItem() {
  layout.value.push({
    x: (layout.value.length * 6) % 12,
    y: 1000,
    w: 6,
    h: 8,
    i: Date.now().toString(),
    config: { backendId: '', topicName: '', topicType: '' }
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
  const baseId = Date.now().toString()
  layout.value = [
    { x: 0, y: 0, w: 6, h: 10, i: `${baseId}-cam`, config: { backendId: '', topicName: '' } },
    { x: 0, y: 10, w: 3, h: 6, i: `${baseId}-data1`, config: { backendId: '', topicName: '' } },
    { x: 3, y: 10, w: 3, h: 6, i: `${baseId}-data2`, config: { backendId: '', topicName: '' } },
    { x: 6, y: 0, w: 6, h: 16, i: `${baseId}-map`, config: { backendId: '', topicName: '' } }
  ]
}

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
/* ============================================
   全局容器
   ============================================ */
.topic-monitor-container {
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  background-color: var(--bg-color);
  transition: background-color 0.3s;
  overflow: hidden;
  position: relative; /* 关键：为 absolute 子元素定位 */
}

/* ============================================
   悬浮抽屉工具栏 (Updated for better visibility)
   ============================================ */
.floating-toolbar {
  position: absolute;
  top: 0;
  left: 50%;
  /* 初始状态：完全将按钮主体移出屏幕 (-100%)，只留下延伸出的 pull-indicator */
  transform: translateX(-50%) translateY(-100%);
  z-index: 1000;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 20px 60px 40px 60px;
  transition: transform 0.4s cubic-bezier(0.22, 1, 0.36, 1);
}

/* Hover 状态：整体下滑，悬浮在顶部 */
.floating-toolbar:hover {
  transform: translateX(-50%) translateY(0px);
}

/* 按钮容器 (保持不变，但去掉之前的透明度 hack) */
.toolbar-content {
  display: flex;
  gap: 2px;
  padding: 10px 20px;
  border-radius: 50px;
  background: var(--panel-bg-color);
  backdrop-filter: blur(20px);
  -webkit-backdrop-filter: blur(20px);
  border: 1px solid var(--panel-border-color);
  box-shadow: var(--panel-shadow);
}

/* [核心修改] 把手指示器 (The Visible Tab) */
.pull-indicator {
  /* 绝对定位到容器底部外面，形成一个"挂件" */
  position: absolute;
  bottom: -18px; /* 挂在按钮容器下方 */
  width: 90px;
  height: 24px;
  /* 样式：半圆底或圆角梯形，做成实体卡片的样子 */
  background: var(--panel-bg-color); /* 同质感背景 */
  backdrop-filter: blur(12px);
  border: 1px solid var(--panel-border-color);
  border-top: none; /* 去掉上边框，看起来是连在一起的 */
  border-radius: 0 0 12px 12px; /* 底部圆角 */
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.08); /* 明显的阴影 */
  /* 内部装饰线 */
  display: flex;
  align-items: center;
  justify-content: center;
  opacity: 1; /* 常驻显示 */
  cursor: pointer;
  transition:
    opacity 0.1s,
    bottom 0.1s;
}

.pull-indicator::before {
  content: '';
  position: absolute;
  background: transparent;
  z-index: -1;
}

/* 把手内的短横线装饰 */
.pull-indicator::after {
  content: '';
  width: 30px;
  height: 4px;
  background-color: var(--text-secondary);
  border-radius: 2px;
  opacity: 0.5;
}

/* 当工具栏展开时，隐藏把手 (看起来更整洁)，或者保持显示皆可 */
/* 这里选择：展开时把手缩回，或者变淡。为了交互连贯，建议让它淡出或稍微上移 */
.floating-toolbar:hover .pull-indicator {
  opacity: 0; /* 展开后隐藏把手，让用户专注于按钮 */
  bottom: 0; /* 视觉上收回 */
  pointer-events: none;
}

/* 按钮美化 */
.modern-btn {
  font-weight: 600;
  border: none;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.05);
  transition: all 0.2s;
}
.modern-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

/* ============================================
   网格区域 (Grid Canvas)
   ============================================ */
.grid-canvas {
  flex: 1;
  overflow-y: auto;
  overflow-x: hidden;
  /* 移除原本的顶部 padding，让卡片可以铺满 */
  padding: 0 15px 15px 15px;

  /* 点阵背景 */
  background-image: radial-gradient(rgba(144, 147, 153, 0.15) 1px, transparent 1px);
  background-size: 24px 24px;
}

.grid-item-wrapper {
  touch-action: none;
  transition:
    box-shadow 0.2s,
    transform 0.2s;
}

.empty-state {
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  user-select: none;
}

.empty-hint {
  margin-top: 10px;
  color: var(--text-secondary);
  font-size: 14px;
  opacity: 0.6;
}
</style>
