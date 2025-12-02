<template>
  <el-dialog
    v-model="visible"
    title="全局设置"
    width="520px"
    :close-on-click-modal="false"
    class="settings-dialog"
    destroy-on-close
    append-to-body
    align-center
  >
    <div class="settings-container">
      <!-- 1. 外观设置 -->
      <div class="setting-group">
        <div class="group-title">外观</div>
        <div class="setting-card">
          <div class="setting-item">
            <div class="item-info">
              <span class="item-label">深色模式</span>
              <span class="item-desc">切换应用程序的浅色/深色主题</span>
            </div>
            <div class="item-control">
              <el-switch
                v-model="isDarkMode"
                inline-prompt
                :active-icon="Moon"
                :inactive-icon="Sunny"
                style="
                  --el-switch-on-color: #409eff;
                  --el-switch-off-color: #f2f2f2;
                  --el-switch-off-text-color: #606266;
                "
              />
            </div>
          </div>
        </div>
      </div>

      <!-- 2. 自动化设置 -->
      <div class="setting-group">
        <div class="group-title">启动与连接</div>
        <div class="setting-card">
          <div class="setting-item border-bottom">
            <div class="item-info">
              <span class="item-label">启动时自动连接</span>
              <span class="item-desc">程序启动时尝试连接上次使用的机器人</span>
            </div>
            <div class="item-control">
              <el-switch v-model="form.autoConnect" />
            </div>
          </div>
          <div class="setting-item">
            <div class="item-info">
              <span class="item-label">连接后自动启动服务</span>
              <span class="item-desc">连接建立后自动启动 Roscore 和 Bridge</span>
            </div>
            <div class="item-control">
              <el-switch v-model="form.autoStartRos" />
            </div>
          </div>
        </div>
      </div>

      <!-- 3. 断开策略 -->
      <div class="setting-group">
        <div class="group-title">断开与清理</div>
        <div class="setting-card">
          <div class="setting-item border-bottom">
            <div class="item-info">
              <span class="item-label">停止 Foxglove Bridge</span>
              <span class="item-desc">断开连接时自动停止数据桥接服务</span>
            </div>
            <div class="item-control">
              <el-switch v-model="form.autoStopBridge" />
            </div>
          </div>
          <div class="setting-item">
            <div class="item-info">
              <span class="item-label warning-text">停止 Roscore</span>
              <span class="item-desc">断开连接时停止主节点 (会影响其他节点)</span>
            </div>
            <div class="item-control">
              <el-switch v-model="form.autoStopRoscore" active-color="#f56c6c" />
            </div>
          </div>
        </div>
      </div>

      <!-- 4. 危险区域 -->
      <div class="setting-group">
        <div class="group-title danger">危险区域</div>
        <div class="setting-card danger-card">
          <div class="setting-item">
            <div class="item-info">
              <span class="item-label danger-text">重置所有配置</span>
              <span class="item-desc">清除所有连接列表、节点配置和习惯</span>
            </div>
            <div class="item-control">
              <el-button
                type="danger"
                plain
                size="small"
                :icon="Delete"
                :disabled="props.isConnected"
                @click="handleReset"
              >
                立即重置
              </el-button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <template #footer>
      <div class="dialog-footer">
        <el-button @click="visible = false">关闭</el-button>
      </div>
    </template>
  </el-dialog>
</template>
<script setup>
import { computed, watch, onMounted } from 'vue'
import { Moon, Sunny, Delete } from '@element-plus/icons-vue'
import { ElMessageBox } from 'element-plus'
import { useDark } from '@vueuse/core'

const props = defineProps({
  modelValue: Boolean,
  settings: { type: Object, required: true },
  isConnected: { type: Boolean, default: false }
})

const emit = defineEmits(['update:modelValue', 'update:settings', 'reset-config'])

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

// 使用 computed 代理 settings，确保双向绑定生效
const form = computed({
  get: () => props.settings,
  set: (val) => emit('update:settings', val)
})

const isDarkMode = useDark({
  // 存储在 localStorage 中的 key，方便 EditorWindow 同步读取
  storageKey: 'vueuse-color-scheme',
  valueDark: 'dark',
  valueLight: 'light'
})

onMounted(() => {
  // 初始化检测
  isDarkMode.value = false
})

// 逻辑联动：停止 Core 必然停止 Bridge
watch(
  () => form.value.autoStopRoscore,
  (newVal) => {
    if (newVal) {
      form.value.autoStopBridge = true
    }
  }
)

const handleReset = () => {
  ElMessageBox.confirm(
    '此操作将清除所有本地保存的连接、节点和偏好设置，且无法恢复。确定要继续吗？',
    '重置确认',
    {
      type: 'warning',
      confirmButtonText: '重置并重启',
      cancelButtonText: '取消',
      confirmButtonClass: 'el-button--danger'
    }
  )
    .then(() => {
      emit('reset-config')
    })
    .catch(() => {})
}
</script>

<style scoped>
/* ============================================
   1. 变量定义
   ============================================ */
.settings-container {
  /* Light Mode */
  --c-bg: #ffffff;
  --c-card-bg: #f5f7fa;
  --c-card-border: #e4e7ed;
  --c-text-main: #303133;
  --c-text-sub: #909399;
  --c-divider: #e4e7ed;
  --c-danger-bg: #fef0f0;
  --c-danger-border: #fde2e2;
  --c-danger-text: #f56c6c;
}

:global(html.dark) .settings-container {
  /* Dark Mode */
  --c-bg: #1e1e20;
  --c-card-bg: #2b2b2d;
  --c-card-border: #414243;
  --c-text-main: #e5eaf3;
  --c-text-sub: #a3a6ad;
  --c-divider: #414243;
  --c-danger-bg: rgba(245, 108, 108, 0.1);
  --c-danger-border: rgba(245, 108, 108, 0.2);
  --c-danger-text: #f78989;
}

.settings-container {
  padding: 0 5px;
}

/* ============================================
   2. 分组与标题
   ============================================ */
.setting-group {
  margin-bottom: 20px;
}

.group-title {
  font-size: 12px;
  font-weight: 600;
  color: var(--c-text-sub);
  margin-bottom: 8px;
  padding-left: 4px;
}
.group-title.danger {
  color: var(--c-danger-text);
}

/* ============================================
   3. 设置卡片 (Card)
   ============================================ */
.setting-card {
  background-color: var(--c-card-bg);
  border: 1px solid var(--c-card-border);
  border-radius: 8px;
  overflow: hidden;
}

.setting-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  min-height: 40px;
}

.setting-item.border-bottom {
  border-bottom: 1px solid var(--c-divider);
}

/* 文本区域 */
.item-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
  flex: 1;
  padding-right: 15px; /* 防止文字挨着开关 */
}

.item-label {
  font-size: 14px;
  font-weight: 500;
  color: var(--c-text-main);
}
.item-label.warning-text {
  color: #e6a23c;
}
.item-label.danger-text {
  color: var(--c-danger-text);
}

.item-desc {
  font-size: 12px;
  color: var(--c-text-sub);
  line-height: 1.4;
}

/* 控件区域 */
.item-control {
  flex-shrink: 0;
}

/* ============================================
   4. 危险区域特殊样式
   ============================================ */
.setting-card.danger-card {
  background-color: var(--c-danger-bg);
  border-color: var(--c-danger-border);
}
.setting-card.danger-card .item-desc {
  color: var(--c-danger-text);
  opacity: 0.8;
}

/* Footer Align */
.dialog-footer {
  display: flex;
  justify-content: flex-end;
}
</style>
