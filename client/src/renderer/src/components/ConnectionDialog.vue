<template>
  <el-dialog
    v-model="isDialogVisible"
    title="连接配置"
    width="450px"
    :close-on-click-modal="false"
    @close="closeDialog"
  >
    <el-form :model="form" label-position="top">
      <el-form-item label="连接模式">
        <el-radio-group v-model="form.mode">
          <el-radio-button label="local">Localhost</el-radio-button>
          <el-radio-button label="remote">远程 (Agent)</el-radio-button>
        </el-radio-group>
      </el-form-item>

      <!-- 仅在 Remote 模式下显示 -->
      <div v-if="form.mode === 'remote'">
        <el-form-item>
          <template #label>
            <div class="host-label-row">
              <span class="label-text">机器人 IP 地址</span>
              <!-- 复用原有的探测 UI -->
              <div class="scan-status">
                <span v-if="discoveredDevices.length > 0" class="status-text success">
                  <el-icon><Connection /></el-icon>
                  已探测到 {{ discoveredDevices.length }} 台
                </span>
                <span v-else class="status-text loading">
                  <el-icon class="is-loading"><Loading /></el-icon>
                  正在探测 mDNS...
                </span>
              </div>
            </div>
          </template>

          <!-- Autocomplete 逻辑适配 -->
          <el-autocomplete
            ref="hostAutocompleteRef"
            v-model="form.ip"
            :fetch-suggestions="querySearch"
            placeholder="e.g., 192.168.1.100"
            style="width: 100%"
            trigger-on-focus
            clearable
            @select="handleSelect"
          >
            <template #default="{ item }">
              <div class="device-item">
                <!-- 显示 Hostname 和 IP -->
                <span class="device-hostname">{{ item.hostname }}</span>
                <span class="device-ip">{{ item.ip }}</span>
              </div>
            </template>
          </el-autocomplete>
        </el-form-item>

        <!-- 新增：名称备注 -->
        <el-form-item label="名称 (可选)">
          <el-input v-model="form.name" placeholder="例如: Turtlebot 4"></el-input>
        </el-form-item>
      </div>

      <!-- Local 模式提示 -->
      <div v-if="form.mode === 'local'" style="color: #909399; font-size: 13px">
        将连接到运行在本机的 Agent (http://127.0.0.1:8080)
      </div>
    </el-form>
    <template #footer>
      <span class="dialog-footer">
        <el-button @click="closeDialog">取消</el-button>
        <el-button type="primary" @click="applyAndClose">确定</el-button>
      </span>
    </template>
  </el-dialog>
</template>

<script setup>
import { ref, reactive, watch } from 'vue'
import { useLanScan } from '../composables/useLanScan' // 引入钩子
import { Connection, Loading } from '@element-plus/icons-vue'

const props = defineProps({
  visible: { type: Boolean, default: false },
  initialData: { type: Object, required: true }
})
const emit = defineEmits(['update:visible', 'apply'])

// 2. form 是一个纯本地的响应式对象。它不再使用 useStorage。
const form = reactive({
  mode: 'local',
  ip: '',
  hostname: '',
  name: ''
})

const hostAutocompleteRef = ref(null)
const isDialogVisible = ref(props.visible)
const { discoveredDevices, start, stop } = useLanScan()

// 3. 监听 visible prop 的变化
watch(
  () => props.visible,
  (newVal) => {
    isDialogVisible.value = newVal
    if (newVal) {
      const data = props.initialData
      form.mode = data.mode || 'local'
      form.ip = data.ip || ''
      form.hostname = data.hostname || ''
      form.name = data.name || ''

      // 如果是 Remote，开启扫描
      if (form.mode === 'remote') {
        start()
      }
    } else {
      stop()
    }
  }
)
// 监听 mode 变化，如果是 local 就不扫了，切回 remote 再扫
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
// 监听设备列表变化，实现实时刷新
watch(
  () => discoveredDevices.value,
  () => {
    // 只有当对话框显示、且组件引用存在时才执行
    if (isDialogVisible.value && hostAutocompleteRef.value) {
      // console.log(`[ConnectionDialog] Devices updated (${newDevices.length}), refreshing list...`)
      // 3. 手动触发搜索
      // getData 是 Element Plus Autocomplete 内部用于获取建议的方法
      // 我们传入当前输入框的值 (form.ip)，让它重新跑一遍 querySearch
      if (typeof hostAutocompleteRef.value.getData === 'function') {
        hostAutocompleteRef.value.getData(form.ip)
      }
    }
  },
  { deep: true } // 深度监听
)

// Autocomplete 的搜索逻辑
const querySearch = (queryString, cb) => {
  const results = discoveredDevices.value.map((d) => {
    return {
      value: d.ip, // 选中后填入输入框的是 IP
      hostname: d.hostname, // 显示用
      ip: d.ip // 显示用
    }
  })
  const filtered = queryString
    ? results.filter(
        (item) => item.value.includes(queryString) || item.hostname.includes(queryString)
      )
    : results

  cb(filtered)
}

function closeDialog() {
  emit('update:visible', false)
}

function applyAndClose() {
  const result = {
    mode: form.mode,
    ip: form.mode === 'local' ? '127.0.0.1' : form.ip,
    hostname: form.mode === 'local' ? 'localhost' : form.hostname,
    name: form.name
  }
  emit('apply', result)
}

const handleSelect = (item) => {
  form.ip = item.ip
  // 保存 hostname (去除 .local 后缀，方便存储)
  form.hostname = item.hostname.replace('.local', '')
  if (!form.name) {
    form.name = form.hostname
  }
}
</script>

<style scoped>
.device-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}
.device-hostname {
  font-weight: bold;
  color: #333;
}
.device-ip {
  font-size: 12px;
  color: #999;
}
.scan-hint {
  margin-top: 5px;
  font-size: 12px;
  color: #909399;
  display: flex;
  align-items: center;
  gap: 5px;
}
:deep(.el-form-item--label-top .el-form-item__label) {
  width: 100%;
}
.host-label-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%; /* 确保占满整行宽度 */
}

.scan-status {
  font-weight: normal;
  font-size: 12px;
}

.status-text {
  display: flex;
  align-items: center;
  gap: 4px;
}

.status-text.loading {
  color: #909399; /* 灰色 */
}

.status-text.success {
  color: #67c23a; /* 绿色，提示用户已发现设备 */
}

/* 保持原有的 device-item 样式 */
.device-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}
.device-hostname {
  font-weight: bold;
  color: #333;
}
.device-ip {
  font-size: 12px;
  color: #999;
}
</style>
