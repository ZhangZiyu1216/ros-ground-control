<template>
  <el-dialog
    v-model="isDialogVisible"
    title="连接配置"
    width="480px"
    :close-on-click-modal="false"
    class="connection-config-dialog"
    destroy-on-close
    align-center
    @close="closeDialog"
  >
    <el-form :model="form" label-position="top" class="conn-form">
      <!-- 1. 模式选择 (瘦身版：水平布局) -->
      <el-form-item label="连接模式" style="margin-bottom: 20px">
        <div class="mode-selector">
          <!-- Local Mode -->
          <div
            class="mode-card"
            :class="{ active: form.mode === 'local' }"
            @click="form.mode = 'local'"
          >
            <div class="card-icon-wrapper">
              <el-icon><Monitor /></el-icon>
            </div>
            <div class="card-text">
              <span class="card-title">本地连接</span>
            </div>
            <div v-if="form.mode === 'local'" class="active-indicator"></div>
          </div>

          <!-- Remote Mode -->
          <div
            class="mode-card"
            :class="{ active: form.mode === 'remote' }"
            @click="form.mode = 'remote'"
          >
            <div class="card-icon-wrapper">
              <el-icon><Connection /></el-icon>
            </div>
            <div class="card-text">
              <span class="card-title">远程主机</span>
            </div>
            <div v-if="form.mode === 'remote'" class="active-indicator"></div>
          </div>
        </div>
      </el-form-item>

      <!-- 2. 动态内容区 -->
      <div class="form-body">
        <transition name="fade-slide" mode="out-in">
          <!-- Case A: Remote -->
          <div v-if="form.mode === 'remote'" key="remote" class="input-group">
            <el-form-item>
              <!-- [需求3] 标签栏两端对齐 -->
              <template #label>
                <div class="host-label-row">
                  <span class="label-text">机器人 IP 地址</span>

                  <!-- [需求4] 恢复 Loading 状态逻辑 -->
                  <div class="scan-status-wrapper">
                    <transition name="el-fade-in">
                      <!-- 状态 A: 已发现 -->
                      <div v-if="discoveredDevices.length > 0" class="scan-status success">
                        <span class="pulse-dot"></span>
                        <span>已发现 {{ discoveredDevices.length }} 台设备</span>
                      </div>
                      <!-- 状态 B: 扫描中 -->
                      <div v-else class="scan-status loading">
                        <el-icon class="is-loading"><Loading /></el-icon>
                        <span>正在扫描局域网...</span>
                      </div>
                    </transition>
                  </div>
                </div>
              </template>

              <el-autocomplete
                ref="hostAutocompleteRef"
                v-model="form.ip"
                :fetch-suggestions="querySearch"
                placeholder="例如: 192.168.1.100"
                style="width: 100%"
                trigger-on-focus
                clearable
                class="modern-input"
                @select="handleSelect"
              >
                <template #prefix>
                  <el-icon class="input-icon"><Search /></el-icon>
                </template>
                <template #default="{ item }">
                  <div class="device-item">
                    <div class="device-main">
                      <el-icon><Platform /></el-icon>
                      <span class="device-hostname">{{ item.hostname }}</span>
                    </div>
                    <span class="device-ip">{{ item.ip }}</span>
                  </div>
                </template>
              </el-autocomplete>
            </el-form-item>

            <!-- [需求2] 去掉 (可选) 标记 -->
            <el-form-item label="连接名称">
              <el-input
                v-model="form.name"
                placeholder="给这个连接起个名字，如: Turtlebot 4"
                class="modern-input"
              >
                <template #prefix>
                  <el-icon class="input-icon"><EditPen /></el-icon>
                </template>
              </el-input>
            </el-form-item>
          </div>

          <!-- Case B: Local -->
          <div v-else key="local" class="local-hint-wrapper">
            <div class="local-hint-box">
              <el-icon class="hint-icon"><InfoFilled /></el-icon>
              <div class="hint-content">
                <p class="hint-title">本地连接模式</p>
                <p class="hint-desc">目标地址: <strong>http://127.0.0.1:8080</strong></p>
              </div>
            </div>
          </div>
        </transition>
      </div>
    </el-form>

    <template #footer>
      <div class="dialog-actions">
        <el-button class="action-btn cancel" @click="closeDialog">取消</el-button>
        <el-button class="action-btn confirm" type="primary" @click="applyAndClose">
          连接
        </el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script setup>
import { ref, reactive, watch } from 'vue'
import { useLanScan } from '../composables/useLanScan'
import {
  Connection,
  Monitor,
  Loading,
  Search,
  Platform,
  EditPen,
  InfoFilled
} from '@element-plus/icons-vue'

const props = defineProps({
  visible: { type: Boolean, default: false },
  initialData: { type: Object, required: true }
})
const emit = defineEmits(['update:visible', 'apply'])

// 本地表单状态
const form = reactive({
  mode: 'local',
  ip: '',
  hostname: '',
  name: ''
})

const hostAutocompleteRef = ref(null)
const isDialogVisible = ref(props.visible) // 本地状态副本，用于 watch 同步
const { discoveredDevices, start, stop } = useLanScan()

// 1. 监听 Props 变化 (打开/关闭弹窗)
watch(
  () => props.visible,
  (newVal) => {
    isDialogVisible.value = newVal
    if (newVal) {
      // 初始化表单
      const data = props.initialData
      form.mode = data.mode || 'local'
      form.ip = data.ip || ''
      form.hostname = data.hostname || ''
      form.name = data.name || ''

      // 如果是远程模式，立即开始扫描
      if (form.mode === 'remote') {
        start()
      }
    } else {
      // 关闭时停止扫描
      stop()
    }
  }
)

// 2. 监听模式切换
watch(
  () => form.mode,
  (newMode) => {
    if (newMode === 'remote') {
      start()
    } else {
      stop()
    }
  }
)

// 3. 监听设备列表变化 (Element Plus Autocomplete 刷新 Hack)
watch(
  discoveredDevices,
  () => {
    if (props.visible && hostAutocompleteRef.value && form.mode === 'remote') {
      // 强制触发建议刷新
      if (typeof hostAutocompleteRef.value.getData === 'function') {
        hostAutocompleteRef.value.getData(form.ip)
      }
    }
  },
  { deep: true }
)

// Autocomplete 过滤逻辑
const querySearch = (queryString, cb) => {
  const results = discoveredDevices.value.map((d) => ({
    value: d.ip, // 选中后填入 Input 的值
    hostname: d.hostname, // 列表显示的辅助信息
    ip: d.ip
  }))

  const filtered = queryString
    ? results.filter(
        (item) =>
          item.value.includes(queryString) || (item.hostname && item.hostname.includes(queryString))
      )
    : results

  cb(filtered)
}

const handleSelect = (item) => {
  form.ip = item.ip
  // 清理 hostname 后缀，确保 Store 中 key 的一致性
  form.hostname = item.hostname ? item.hostname.replace('.local', '') : ''
  // 如果用户还没填名称，自动填入 hostname
  if (!form.name) {
    form.name = form.hostname || form.ip
  }
}

function closeDialog() {
  emit('update:visible', false)
}

function applyAndClose() {
  // 构造最终配置对象
  const result = {
    mode: form.mode,
    ip: form.mode === 'local' ? '127.0.0.1' : form.ip,
    hostname: form.mode === 'local' ? 'localhost' : form.hostname,
    name: form.name
  }
  emit('apply', result)
}
</script>

<style scoped>
/* ============================================
   1. 变量定义
   ============================================ */
.conn-form {
  --c-bg: #ffffff;
  --c-card-bg: #f5f7fa;
  --c-card-border: #e4e7ed;
  --c-card-hover: #ecf5ff;
  --c-primary: #409eff;
  --c-text-main: #303133;
  --c-text-sub: #909399;
}
:global(html.dark) .conn-form {
  --c-bg: #1e1e20;
  --c-card-bg: #2b2b2d;
  --c-card-border: #414243;
  --c-card-hover: rgba(64, 158, 255, 0.1);
  --c-primary: #409eff;
  --c-text-main: #e5eaf3;
  --c-text-sub: #a3a6ad;
}

/* ============================================
   2. 模式选择 (Compact Horizontal Style)
   ============================================ */
.mode-selector {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
}

.mode-card {
  position: relative;
  display: flex;
  align-items: center; /* 垂直居中 */
  height: 40px; /* [需求1] 限制高度，不再喧宾夺主 */
  padding: 0 15px;
  background-color: var(--c-card-bg);
  border: 1px solid var(--c-card-border);
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s ease;
  overflow: hidden;
}

.mode-card:hover {
  background-color: var(--c-card-hover);
  border-color: var(--c-primary);
}

.mode-card.active {
  background-color: rgba(64, 158, 255, 0.08);
  border-color: var(--c-primary);
}

.card-icon-wrapper {
  font-size: 18px;
  color: var(--c-text-sub);
  margin-right: 10px;
  display: flex;
  align-items: center;
}
.mode-card.active .card-icon-wrapper {
  color: var(--c-primary);
}

.card-text {
  display: flex;
  flex-direction: column;
  justify-content: center;
}
.card-title {
  font-weight: 600;
  font-size: 13px;
  color: var(--c-text-main);
}

/* 选中指示条 */
.active-indicator {
  position: absolute;
  left: 0;
  top: 0;
  bottom: 0;
  width: 3px;
  background-color: var(--c-primary);
}

/* ============================================
   3. 标签行对齐修复
   ============================================ */
/* 强制 Label 占满整行 */
:deep(.el-form-item__label) {
  width: 100%;
  display: block; /* 确保是块级，方便内部 flex 布局 */
  line-height: 20px;
  margin-bottom: 8px !important;
}

.host-label-row {
  display: flex;
  justify-content: space-between; /* [需求3] 两端对齐 */
  align-items: center;
  width: 103%;
}
.label-text {
  font-weight: 500;
  color: var(--c-text-main);
}

/* [需求4] 扫描状态样式 */
.scan-status {
  font-size: 12px;
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 2px 8px;
  border-radius: 4px;
}
/* Loading 态 */
.scan-status.loading {
  color: var(--c-text-sub);
  background: rgba(144, 147, 153, 0.1);
}
/* Success 态 */
.scan-status.success {
  color: #67c23a;
  background: rgba(103, 194, 58, 0.1);
}

.pulse-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background-color: #67c23a;
  box-shadow: 0 0 0 2px rgba(103, 194, 58, 0.2);
}

/* ============================================
   4. 通用样式
   ============================================ */
.form-body {
  min-height: 160px;
}
.input-group {
  display: flex;
  flex-direction: column;
  gap: 0px; /* gap由form-item margin控制 */
}

.modern-input :deep(.el-input__wrapper) {
  background-color: var(--c-card-bg);
  box-shadow: none !important;
  border: 1px solid var(--c-card-border);
  border-radius: 6px;
}
.modern-input :deep(.el-input__wrapper.is-focus) {
  background-color: var(--c-bg);
  border-color: var(--c-primary);
  box-shadow: 0 0 0 1px var(--c-primary) !important;
}
.input-icon {
  color: var(--c-text-sub);
}

/* Autocomplete item */
.device-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 4px 0;
}
.device-main {
  display: flex;
  align-items: center;
  gap: 6px;
  font-weight: 500;
  color: var(--c-text-main);
}
.device-ip {
  font-size: 12px;
  color: var(--c-text-sub);
  font-family: monospace;
}

/* Local Hint */
.local-hint-wrapper {
  padding-top: 10px;
}
.local-hint-box {
  display: flex;
  gap: 12px;
  background: rgba(64, 158, 255, 0.05);
  border: 1px solid rgba(64, 158, 255, 0.1);
  padding: 15px;
  border-radius: 6px;
}
.hint-icon {
  font-size: 20px;
  color: var(--c-primary);
}
.hint-title {
  margin: 0 0 4px 0;
  font-weight: 600;
  font-size: 13px;
  color: var(--c-text-main);
}
.hint-desc {
  margin: 0;
  font-size: 12px;
  color: var(--c-text-sub);
}

/* Footer */
.dialog-actions {
  display: flex;
  justify-content: flex-end;
  gap: 2px;
  padding-top: 10px;
}
.action-btn {
  border-radius: 6px;
  padding: 8px 20px;
  font-weight: 500;
}
.action-btn.confirm {
  background: linear-gradient(135deg, #409eff, #337ecc);
  border: none;
  box-shadow: 0 3px 10px rgba(64, 158, 255, 0.3);
}
.action-btn.confirm:hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 14px rgba(64, 158, 255, 0.4);
}
</style>
