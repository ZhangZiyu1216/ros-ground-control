<template>
  <el-dialog
    v-model="visible"
    title="全局设置"
    width="500px"
    :close-on-click-modal="false"
    append-to-body
  >
    <el-form label-position="left" label-width="200px">
      <el-divider content-position="left">启动与连接</el-divider>

      <el-form-item label="程序启动时自动连接">
        <template #label>
          <span>启动时自动连接</span>
          <el-tooltip content="启动程序时自动尝试连接上次使用的机器人" placement="top">
            <el-icon class="help-icon"><QuestionFilled /></el-icon>
          </el-tooltip>
        </template>
        <el-switch v-model="form.autoConnect" />
      </el-form-item>

      <el-form-item label="连接后自动启动服务">
        <template #label>
          <span>连接后自动启动服务</span>
          <el-tooltip content="连接建立后，自动发送指令启动 Roscore 和 Bridge" placement="top">
            <el-icon class="help-icon"><QuestionFilled /></el-icon>
          </el-tooltip>
        </template>
        <el-switch v-model="form.autoStartRos" />
      </el-form-item>

      <el-divider content-position="left">断开与清理</el-divider>

      <el-form-item label="断开时停止 Bridge">
        <template #label>
          <span>断开时停止 Bridge</span>
          <el-tooltip content="手动断开连接时，自动停止 Foxglove Bridge 数据桥" placement="top">
            <el-icon class="help-icon"><QuestionFilled /></el-icon>
          </el-tooltip>
        </template>
        <el-switch v-model="form.autoStopBridge" />
      </el-form-item>

      <el-form-item label="断开时停止 Roscore">
        <template #label>
          <span>断开时停止 Roscore</span>
          <el-tooltip
            content="手动断开连接时，自动停止 Roscore (警告：会影响其他节点)"
            placement="top"
          >
            <el-icon class="help-icon"><QuestionFilled /></el-icon>
          </el-tooltip>
        </template>
        <el-switch v-model="form.autoStopRoscore" />
      </el-form-item>

      <el-divider content-position="left">危险区域</el-divider>

      <div class="danger-zone">
        <p>清除所有保存的连接列表、节点配置和使用习惯。</p>
        <el-button type="danger" plain size="small" @click="handleReset">重置所有配置</el-button>
      </div>
    </el-form>

    <template #footer>
      <el-button @click="visible = false">关闭</el-button>
    </template>
  </el-dialog>
</template>

<script setup>
import { computed, watch } from 'vue'
import { QuestionFilled } from '@element-plus/icons-vue'
import { ElMessageBox } from 'element-plus'

const props = defineProps({
  modelValue: Boolean,
  settings: Object
})

const emit = defineEmits(['update:modelValue', 'update:settings', 'reset-config'])

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

// 直接绑定 props.settings (因为是对象引用，父组件 reactive 会自动更新)
// 或者可以使用 computed get/set 来 emit update
const form = computed({
  get: () => props.settings,
  set: (val) => emit('update:settings', val)
})
// 逻辑联动：如果开启停止 Roscore，意味着必须停止 Bridge (因为 Bridge 依赖 Roscore)
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
    { type: 'warning', confirmButtonText: '重置并重启', cancelButtonText: '取消' }
  )
    .then(() => {
      emit('reset-config')
    })
    .catch(() => {})
}
</script>

<style scoped>
.help-icon {
  margin-left: 4px;
  color: #909399;
  cursor: help;
  vertical-align: middle;
}
.danger-zone {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background-color: #fef0f0;
  padding: 10px;
  border-radius: 4px;
}
.danger-zone p {
  margin: 0;
  font-size: 12px;
  color: #f56c6c;
}
</style>
