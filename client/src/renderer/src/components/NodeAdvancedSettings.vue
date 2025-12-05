<!-- src/renderer/src/components/NodeAdvancedSettings.vue -->
<template>
  <el-dialog
    :model-value="visible"
    title="节点高级配置"
    width="420px"
    destroy-on-close
    append-to-body
    class="advanced-settings-dialog"
    @update:model-value="emit('update:visible', $event)"
  >
    <div class="settings-container">
      <!-- 1. 优先级设置 -->
      <div class="setting-group">
        <div class="setting-header">
          <span class="setting-title">
            <el-icon class="icon-prefix"><Top /></el-icon>
            进程优先级 (Nice)
          </span>
          <el-switch
            v-model="highPriorityProxy"
            inline-prompt
            active-text="High"
            inactive-text="Norm"
            style="--el-switch-on-color: #e6a23c"
          />
        </div>
        <p class="setting-desc">
          赋予该节点更高的 CPU 调度优先级 (Nice = -10)。
          <br />
          适用于 <b>VINS/SLAM, Lidar Driver, MPC</b> 等对实时性要求极高的核心节点。
        </p>
      </div>

      <el-divider class="setting-divider" />

      <!-- 2. [预留] 话题看门狗 -->
      <div class="setting-group disabled-group">
        <div class="setting-header">
          <span class="setting-title">
            <el-icon class="icon-prefix"><View /></el-icon>
            存活检查 (Topic Watchdog)
          </span>
          <el-switch disabled />
        </div>
        <p class="setting-desc">
          启动后监控指定话题频率。如果话题未发布或频率过低，自动判定为启动失败并重启。
          <span class="tag-dev">(开发中)</span>
        </p>
      </div>
    </div>

    <template #footer>
      <span class="dialog-footer">
        <el-button @click="emit('update:visible', false)">关闭 (暂存)</el-button>
      </span>
    </template>
  </el-dialog>
</template>

<script setup>
import { computed } from 'vue'
import { Top, View } from '@element-plus/icons-vue'

const props = defineProps({
  visible: {
    type: Boolean,
    default: false
  },
  // 接收父组件的 form 对象
  config: {
    type: Object,
    required: true
  }
})

const emit = defineEmits(['update:visible', 'update:item'])
const highPriorityProxy = computed({
  get() {
    // 读取时：直接从 props 获取
    return props.config.highPriority
  },
  set(val) {
    // 写入时：不直接修改 props，而是通知父组件去改
    // 格式：{ key: 字段名, value: 新值 }
    emit('update:item', { key: 'highPriority', value: val })
  }
})
</script>

<style scoped>
/* 这里是对话框内部的样式 */
.settings-container {
  padding: 0 5px;
}

.setting-group {
  padding: 8px 0;
  transition: opacity 0.3s;
}

.setting-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.setting-title {
  font-weight: 600;
  font-size: 15px;
  color: var(--text-primary, #303133);
  display: flex;
  align-items: center;
  gap: 6px;
}

.icon-prefix {
  font-size: 16px;
  color: var(--text-secondary, #909399);
}

.setting-desc {
  font-size: 13px;
  color: var(--text-secondary, #909399);
  line-height: 1.5;
  margin: 0;
  padding-left: 24px; /* 对齐标题文字 */
}

.setting-divider {
  margin: 16px 0;
  border-color: var(--border-color, #dcdfe6);
}

/* 禁用状态 */
.disabled-group {
  opacity: 0.5;
  cursor: not-allowed;
  filter: grayscale(0.8);
}

.tag-dev {
  font-size: 12px;
  color: #909399;
  background: rgba(0, 0, 0, 0.05);
  padding: 2px 4px;
  border-radius: 4px;
  margin-left: 4px;
}
</style>
