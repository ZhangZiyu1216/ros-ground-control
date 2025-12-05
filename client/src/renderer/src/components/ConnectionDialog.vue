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
          <el-tooltip
            :content="isWindows ? 'Windows 不支持 ROS 1 本地节点' : '连接本机的 ROS 环境'"
            placement="top"
            :disabled="!isWindows"
          >
            <div
              class="mode-card"
              :class="{ active: form.mode === 'local', 'is-disabled': isWindows }"
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
          </el-tooltip>

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
            <!-- [修改] 改用 Flex 布局容器，替代 el-row -->
            <div class="remote-row">
              <!-- 左侧：IP 地址 (自适应宽度) -->
              <el-form-item class="ip-item">
                <template #label>
                  <div class="host-label-row">
                    <span class="label-text">机器人 IP 地址</span>
                    <!-- 扫描状态 -->
                    <div class="scan-status-wrapper">
                      <transition name="el-fade-in">
                        <div v-if="discoveredDevices.length > 0" class="scan-status success">
                          <span class="pulse-dot"></span>
                          <span>发现 {{ discoveredDevices.length }} 台</span>
                        </div>
                        <div v-else class="scan-status loading">
                          <el-icon class="is-loading"><Loading /></el-icon>
                          <span>扫描中...</span>
                        </div>
                      </transition>
                    </div>
                  </div>
                </template>

                <el-autocomplete
                  ref="hostAutocompleteRef"
                  v-model="form.ip"
                  :fetch-suggestions="querySearch"
                  placeholder="192.168.1.100"
                  style="width: 100%"
                  trigger-on-focus
                  clearable
                  class="modern-input"
                  @select="handleSelect"
                >
                  <template #prefix
                    ><el-icon class="input-icon"><Search /></el-icon
                  ></template>
                  <template #default="{ item }">
                    <div class="device-item">
                      <div class="device-main">
                        <el-icon><Platform /></el-icon>
                        <span class="device-hostname">{{ item.hostname }}</span>
                      </div>
                      <!-- 下拉列表加宽显示 -->
                      <span class="device-ip">{{ item.ip }}</span>
                    </div>
                  </template>
                </el-autocomplete>
              </el-form-item>

              <!-- 右侧：端口 (固定宽度) -->
              <el-form-item label="端口" class="port-item">
                <!-- [技巧] 使用空白的 host-label-row 撑开高度，防止 Label 高度不一致导致输入框跳动 -->
                <template #label>
                  <div class="host-label-row placeholder">
                    <span class="label-text">端口</span>
                  </div>
                </template>
                <el-input-number
                  v-model="form.port"
                  :min="1"
                  :max="65535"
                  :controls="false"
                  class="modern-number-input"
                  placeholder="8080"
                />
              </el-form-item>
            </div>

            <el-form-item label="连接名称">
              <el-input v-model="form.name" placeholder="例如: Turtlebot 4" class="modern-input">
                <template #prefix
                  ><el-icon class="input-icon"><EditPen /></el-icon
                ></template>
              </el-input>
            </el-form-item>
          </div>

          <!-- Case B: Local -->
          <div v-else key="local" class="local-hint-wrapper">
            <el-form-item>
              <template #label>
                <div class="host-label-row">
                  <span class="label-text">本地服务端口</span>
                  <div class="scan-status-wrapper">
                    <span
                      v-if="localAutoDetected"
                      style="
                        color: #67c23a;
                        font-size: 12px;
                        display: flex;
                        align-items: center;
                        gap: 4px;
                      "
                    >
                      <el-icon><Check /></el-icon> 已自动匹配
                    </span>
                    <span
                      v-else
                      style="
                        color: #e6a23c;
                        font-size: 12px;
                        display: flex;
                        align-items: center;
                        gap: 4px;
                      "
                    >
                      <el-icon class="is-loading"><Loading /></el-icon> 扫描中...
                    </span>
                  </div>
                </div>
              </template>

              <!-- [修改] 限制宽度，左对齐 -->
              <div class="local-port-row">
                <el-input-number
                  v-model="form.port"
                  :min="1"
                  :max="65535"
                  :controls="false"
                  class="modern-number-input local-port-input"
                />
                <span class="local-port-hint">默认 8080，随机端口会自动回填</span>
              </div>
            </el-form-item>
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
import { ref, reactive, watch, onMounted } from 'vue'
import { useLanScan } from '../composables/useLanScan'
import { Connection, Monitor, Loading, Search, Platform, EditPen } from '@element-plus/icons-vue'

const props = defineProps({
  visible: { type: Boolean, default: false },
  initialData: { type: Object, required: true }
})
const emit = defineEmits(['update:visible', 'apply'])

// 本地表单状态
const form = reactive({
  mode: 'local',
  ip: '',
  port: 8080,
  hostname: '',
  name: ''
})

const isWindows = ref(false)

const hostAutocompleteRef = ref(null)
const isDialogVisible = ref(props.visible) // 本地状态副本，用于 watch 同步
const localAutoDetected = ref(false) // 标记是否成功自动发现
const localIps = ref([]) // 存储本机的 IP 列表
const localHostname = ref('') // [新增] 存储本机主机名

// [新增] 获取本机 IP 列表，用于比对
const fetchLocalInfo = async () => {
  try {
    const info = await window.api.getHostInfo()

    // 1. 保存主机名 (转小写，方便比对)
    if (info.hostname) {
      localHostname.value = info.hostname.toLowerCase()
    }

    // 2. 保存 IP 列表
    const ips = []
    if (info && info.network) {
      Object.values(info.network).forEach((iface) => {
        iface.forEach((addr) => {
          if (addr.family === 'IPv4') ips.push(addr.address)
        })
      })
    }
    if (!ips.includes('127.0.0.1')) ips.push('127.0.0.1')
    localIps.value = ips
  } catch (e) {
    console.error('Failed to get host info:', e)
  }
}

// --- 自动匹配逻辑 (提取为独立函数) ---
const tryAutoMatchLocalPort = () => {
  if (form.mode !== 'local') return

  const match = discoveredDevices.value.find((d) => {
    // 规则 1: IP 匹配
    if (localIps.value.includes(d.ip)) return true

    // 规则 2: Hostname 匹配 (更可靠)
    // d.hostname 可能带 .local 后缀，也可能不带
    const scannedName = (d.hostname || '').toLowerCase().replace('.local', '')
    if (scannedName && scannedName === localHostname.value) return true

    return false
  })

  if (match) {
    // 只有当尚未探测到，或者探测到的端口与当前不同时才更新
    if (!localAutoDetected.value || form.port !== match.port) {
      console.log(
        `[Connection] Auto-detected local agent: ${match.hostname} (${match.ip}:${match.port})`
      )
      form.port = match.port
      localAutoDetected.value = true
    }
  }
}

const { discoveredDevices, start, stop } = useLanScan()

// 组件挂载时检测平台
onMounted(async () => {
  if (window.api && window.api.getHostInfo) {
    const info = await window.api.getHostInfo()
    isWindows.value = info.platform === 'win32'
  }
})

// 1. 监听弹窗显隐
watch(
  () => props.visible,
  async (newVal) => {
    isDialogVisible.value = newVal
    if (newVal) {
      // Init Form
      const data = props.initialData
      // 如果是 Windows，强制 Remote；否则用传入的或默认 Local
      form.mode = isWindows.value ? 'remote' : data.mode || 'local'

      form.ip = data.ip || ''
      form.port = data.port || 8080
      form.hostname = data.hostname || ''
      form.name = data.name || ''

      localAutoDetected.value = false
      await fetchLocalInfo()

      // [关键] 无论 Local 还是 Remote，打开弹窗就启动扫描
      // Remote 用来列出列表，Local 用来自动填端口
      start()
    } else {
      stop()
    }
  }
)

// 2. 监听模式切换
watch(
  () => form.mode,
  (newMode) => {
    if (newMode === 'local') {
      localAutoDetected.value = false
      // [修复] 切换回 Local 时，立即检查现有的 discoveredDevices
      tryAutoMatchLocalPort()
    }
  }
)

// 3. 监听设备列表更新
watch(
  discoveredDevices,
  () => {
    // Local 模式：尝试自动匹配
    if (form.mode === 'local') {
      tryAutoMatchLocalPort()
    }

    // Remote 模式：刷新 Autocomplete 下拉框
    if (props.visible && hostAutocompleteRef.value && form.mode === 'remote') {
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
    ip: d.ip,
    port: d.port
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
  if (item.port) {
    form.port = item.port
  }
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

const applyAndClose = async () => {
  // 表单验证
  if (form.mode === 'remote') {
    if (!form.ip) {
      return
    }
  }

  // [核心修复] 数据规范化 (Normalization)
  // 确保传给后端和保存到 Config 的数据结构是标准的
  const payload = {
    // 基础 ID (如果有)
    id: props.initialData?.id || null,

    // 强制规范 IP 和 Mode
    ip: form.mode === 'local' ? '127.0.0.1' : form.ip,
    port: form.port || 8080,
    mode: form.mode, // [关键] 显式保存 mode 字段

    // 智能名称处理
    name: form.name || (form.mode === 'local' ? 'Localhost' : form.ip),
    hostname: form.mode === 'local' ? 'localhost' : '' // 辅助字段
  }

  // 发送事件
  emit('apply', payload)
  closeDialog()
}
</script>

<style scoped>
/* ============================================
   1. 模式选择 (Mode Cards)
   ============================================ */
.mode-selector {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
}

.mode-card {
  position: relative;
  display: flex;
  align-items: center;
  height: 40px;
  padding: 0 15px;
  background-color: var(--panel-bg-color);
  border: 1px solid var(--panel-border-color);
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s ease;
  overflow: hidden;
}

.mode-card:hover {
  background-color: rgba(128, 128, 128, 0.05);
  border-color: #409eff;
}

.mode-card.active {
  background-color: var(--bg-color);
  border-color: #409eff;
}

.mode-card.is-disabled {
  opacity: 0.6;
  cursor: not-allowed;
  filter: grayscale(1);
}

.card-icon-wrapper {
  font-size: 18px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  margin-right: 10px;
  display: flex;
  align-items: center;
}
.mode-card.active .card-icon-wrapper {
  color: #409eff;
}

.card-text {
  display: flex;
  flex-direction: column;
  justify-content: center;
}
.card-title {
  font-weight: 600;
  font-size: 13px;
  color: var(--text-primary); /* [修改] 全局变量 */
}

.active-indicator {
  position: absolute;
  left: 0;
  top: 0;
  bottom: 0;
  width: 3px;
  background-color: #409eff;
}

/* ============================================
   2. 布局修复 (Flex Row & Label Alignment)
   ============================================ */
.form-body {
  min-height: 160px;
}

/* Remote 模式左右布局 */
.remote-row {
  display: flex;
  gap: 15px;
  align-items: flex-start;
}
.ip-item {
  flex: 1;
  min-width: 0;
}
.port-item {
  width: 100px;
  flex-shrink: 0;
}

/* 强制 Label 高度一致，防止跳动 */
.host-label-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
  height: 24px;
}
.label-text {
  font-weight: 500;
  color: var(--text-primary); /* [修改] 全局变量 */
}

/* Local 模式布局 */
.local-port-row {
  display: flex;
  align-items: center;
  gap: 12px;
}
.local-port-input {
  width: 120px !important;
}
.local-port-hint {
  font-size: 12px;
  color: var(--text-secondary); /* [修改] 全局变量 */
}

/* ============================================
   3. 扫描状态灯与动画
   ============================================ */
.scan-status-wrapper {
  height: 20px;
  display: flex;
  align-items: center;
}

.scan-status {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 12px;
  padding: 2px 8px;
  border-radius: 4px;
  white-space: nowrap;
  transition: all 0.3s;
}

/* Loading 态 */
.scan-status.loading {
  background-color: var(--panel-bg-color); /* [修改] 全局变量 */
  color: var(--text-secondary); /* [修改] 全局变量 */
}

/* Success 态 (发现设备) */
.scan-status.success {
  background-color: rgba(103, 194, 58, 0.1);
  color: #67c23a;
}

/* 呼吸圆点 */
.pulse-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background-color: currentColor; /* 跟随文字颜色 */
  box-shadow: 0 0 0 0 currentColor;
  animation: pulse-green 2s infinite;
}

@keyframes pulse-green {
  0% {
    transform: scale(0.95);
    box-shadow: 0 0 0 0 rgba(103, 194, 58, 0.7);
  }
  70% {
    transform: scale(1);
    box-shadow: 0 0 0 4px rgba(103, 194, 58, 0);
  }
  100% {
    transform: scale(0.95);
    box-shadow: 0 0 0 0 rgba(103, 194, 58, 0);
  }
}

/* ============================================
   4. 输入框样式统一 (Universal Input Styling)
   ============================================ */
/* [核心修复] 强制覆盖所有输入组件 - 使用全局变量 */
.conn-form :deep(.el-input__wrapper),
.conn-form :deep(.el-input-number__wrapper) {
  background-color: var(--panel-bg-color) !important; /* [修改] 全局变量 */
  box-shadow: none !important;
  border: 1px solid var(--panel-border-color) !important; /* [修改] 全局变量 */

  border-radius: 6px;
  padding: 0 11px;
  height: 32px; /* 固定高度 */
  transition: all 0.2s;

  color: var(--text-primary) !important; /* [修改] 全局变量 */
}

/* Hover */
.conn-form :deep(.el-input__wrapper:hover),
.conn-form :deep(.el-input-number__wrapper:hover) {
  background-color: var(--bg-color) !important; /* [修改] 全局变量 */
  border-color: var(--text-secondary) !important; /* [修改] 全局变量 */
}

/* Focus */
.conn-form :deep(.el-input__wrapper.is-focus),
.conn-form :deep(.el-input-number__wrapper.is-focus) {
  background-color: var(--bg-color) !important; /* [修改] 全局变量 */
  border-color: #409eff !important;
  box-shadow: 0 0 0 1px #409eff !important;
}

/* 修复数字输入框按钮 */
.modern-number-input :deep(.el-input__wrapper) {
  padding-left: 10px !important;
  padding-right: 10px !important;
}
.modern-number-input :deep(.el-input__inner) {
  text-align: left;
}

.input-icon {
  color: var(--text-secondary); /* [修改] 全局变量 */
}

/* ============================================
   5. 其他组件 (Autocomplete, Footer)
   ============================================ */
/* 下拉项 */
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
  color: var(--text-primary); /* [修改] 全局变量 */
}
.device-ip {
  font-size: 12px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  font-family: monospace;
}

/* 底部按钮 */
.dialog-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
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
  color: white;
  box-shadow: 0 3px 10px rgba(64, 158, 255, 0.3);
}
.action-btn.confirm:hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 14px rgba(64, 158, 255, 0.4);
}

/* 强制 Label 占满整行 */
:deep(.el-form-item__label) {
  width: 100%;
  display: block;
  line-height: 20px;
  margin-bottom: 6px !important;
  color: var(--text-primary); /* [修改] 确保 Label 颜色也跟随主题 */
}
</style>
