<template>
  <el-dialog
    v-model="visible"
    title="首次部署 Agent"
    width="520px"
    :close-on-click-modal="false"
    class="setup-dialog"
    destroy-on-close
    append-to-body
    align-center
  >
    <!-- [修复2] hide-required-asterisk 去除红色星号 -->
    <el-form
      ref="formRef"
      :model="form"
      label-position="top"
      class="setup-form"
      :hide-required-asterisk="true"
    >
      <!-- 1. 顶部提示 -->
      <div class="info-card">
        <el-icon class="info-icon"><InfoFilled /></el-icon>
        <div class="info-text">
          此功能将通过 SSH 连接机器人，自动检测环境、上传二进制文件并配置 Systemd 守护进程。
        </div>
      </div>

      <!-- 2. IP 地址 -->
      <el-form-item prop="host" :rules="{ required: true, message: 'IP 地址不能为空' }">
        <template #label>
          <!-- [修复4] 确保标签行占满宽度，两端对齐 -->
          <div class="label-row full-width">
            <span class="label-text">机器人 IP 地址</span>

            <!-- 扫描状态 -->
            <div class="scan-status-wrapper">
              <transition name="fade">
                <!-- 状态 A: 发现设备 -->
                <div v-if="sshDevices.length > 0" class="scan-status success">
                  <div class="status-dot"></div>
                  <span>发现 {{ sshDevices.length }} 个 SSH 设备</span>
                </div>
                <!-- 状态 B: 扫描中/未发现 -->
                <div v-else class="scan-status loading">
                  <el-icon class="is-loading"><Loading /></el-icon>
                  <span>正在扫描 SSH...</span>
                </div>
              </transition>
            </div>
          </div>
        </template>

        <el-autocomplete
          v-model="form.host"
          :fetch-suggestions="querySearch"
          placeholder="例如: 192.168.1.10"
          style="width: 100%"
          trigger-on-focus
          clearable
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
              <span class="device-ip">{{ item.value }}</span>
            </div>
          </template>
        </el-autocomplete>
      </el-form-item>

      <!-- 3. 端口与架构 -->
      <div class="form-row">
        <el-form-item label="SSH 端口" prop="port" class="half-item">
          <el-input-number v-model="form.port" :min="1" :max="65535" controls-position="right" />
        </el-form-item>

        <el-form-item label="目标架构" prop="arch" class="half-item">
          <el-select v-model="form.arch" placeholder="选择架构">
            <template #prefix
              ><el-icon class="input-icon"><Cpu /></el-icon
            ></template>
            <el-option label="ARM64 (树莓派/Jetson)" value="arm64" />
            <el-option label="AMD64 (PC/NUC)" value="amd64" />
          </el-select>
        </el-form-item>
      </div>

      <!-- 4. 认证信息 -->
      <el-form-item
        label="用户名"
        prop="username"
        :rules="{ required: true, message: '用户名不能为空' }"
      >
        <el-input v-model="form.username" placeholder="例如: ubuntu 或 root">
          <template #prefix
            ><el-icon class="input-icon"><User /></el-icon
          ></template>
        </el-input>
      </el-form-item>

      <el-form-item label="Sudo 密码" prop="password">
        <el-input
          v-model="form.password"
          type="password"
          show-password
          placeholder="用于获取安装权限"
        >
          <template #prefix
            ><el-icon class="input-icon"><Key /></el-icon
          ></template>
        </el-input>
      </el-form-item>

      <!-- 5. 功能卡片 -->
      <div
        class="feature-card"
        :class="{ active: form.installFoxglove }"
        @click="form.installFoxglove = !form.installFoxglove"
      >
        <div class="feature-icon">
          <el-icon><Download /></el-icon>
        </div>
        <div class="feature-content">
          <div class="feature-title">安装 Foxglove Bridge</div>
          <div class="feature-desc">自动配置 ROS 1 Noetic 的 WebSocket 桥接服务</div>
        </div>
        <div class="feature-check">
          <el-checkbox v-model="form.installFoxglove" @click.stop />
        </div>
      </div>
    </el-form>

    <template #footer>
      <div class="dialog-footer-row">
        <!-- [新增] 左侧卸载按钮 -->
        <el-popconfirm
          title="确定要从目标机器卸载 Agent 吗？这将停止服务并删除相关文件。"
          confirm-button-text="确认卸载"
          cancel-button-text="取消"
          confirm-button-type="danger"
          width="260"
          @confirm="handleUninstall"
        >
          <template #reference>
            <el-button type="danger" link :disabled="isDeploying">卸载 Agent</el-button>
          </template>
        </el-popconfirm>
        <div class="dialog-actions">
          <el-button class="action-btn cancel" :disabled="isDeploying" @click="visible = false"
            >取消</el-button
          >
          <el-button
            class="action-btn confirm"
            type="primary"
            :loading="isDeploying"
            @click="handleDeploy"
          >
            连接并部署
          </el-button>
        </div>
      </div>
    </template>

    <!-- [修复1] 部署进度遮罩：移到最外层，并使用 fixed 定位确保覆盖 -->
    <transition name="fade">
      <div v-if="isDeploying" class="progress-overlay-fixed">
        <div class="progress-card">
          <el-progress type="circle" :percentage="progress.percent" :width="80" :stroke-width="6" />
          <h3 class="progress-title">正在部署...</h3>
          <p class="progress-desc">{{ progress.message }}</p>
        </div>
      </div>
    </transition>
  </el-dialog>
</template>

<script setup>
import { ref, reactive, computed, watch, toRaw, onMounted } from 'vue'
import { useSshScan } from '../composables/useSshScan'
import { InfoFilled, Search, Platform, User, Key, Cpu, Download } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

const props = defineProps({
  modelValue: Boolean,
  isDeploying: Boolean
})
const emit = defineEmits(['update:modelValue', 'deploy'])

// 1. SSH 扫描
const { sshDevices, start, stop } = useSshScan()

// 2. 表单数据
const formRef = ref(null)
const form = reactive({
  host: '',
  port: 22,
  username: '',
  password: '',
  arch: 'arm64',
  installFoxglove: true
})
const localIsBusy = ref(false)
const progress = reactive({ percent: 0, message: '准备中...' })
const isDeploying = computed(() => props.isDeploying || localIsBusy.value)

// 3. 显隐控制
const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

// 4. 弹窗打开时启动扫描
watch(
  () => props.modelValue,
  (val) => {
    if (val) {
      start()
    } else {
      stop()
    }
  }
)

// 5. 自动补全逻辑
const querySearch = (queryString, cb) => {
  const results = sshDevices.value.map((d) => ({
    value: d.ip,
    hostname: d.hostname || d.ip
  }))
  const filtered = queryString
    ? results.filter(
        (i) => i.value.includes(queryString) || (i.hostname && i.hostname.includes(queryString))
      )
    : results
  cb(filtered)
}

const handleSelect = (item) => {
  form.host = item.value
}

// 6. 提交部署
const handleDeploy = () => {
  if (!formRef.value) return

  formRef.value.validate((valid) => {
    if (valid) {
      // 传递普通对象，去除 Vue Proxy 包装
      emit('deploy', toRaw(form))
    }
  })
}

// 7. 卸载动作 (内部处理)
const handleUninstall = async () => {
  // 校验表单 (至少需要 IP, Port, User, Pass)
  const isValid = await formRef.value.validate().catch(() => false)
  if (!isValid) return

  localIsBusy.value = true
  progress.percent = 0
  progress.message = '正在初始化卸载...'
  progress.status = ''

  try {
    const result = await window.api.uninstallAgent(toRaw(form))

    if (result.success) {
      progress.percent = 100
      progress.message = '卸载成功'
      progress.status = 'success'
      ElMessage.success('Agent 已成功卸载')
      setTimeout(() => {
        localIsBusy.value = false
        visible.value = false
      }, 1500)
    } else {
      throw new Error(result.message)
    }
  } catch (e) {
    progress.status = 'exception'
    progress.message = '卸载失败: ' + e.message
    ElMessage.error(e.message)
    setTimeout(() => {
      localIsBusy.value = false
    }, 3000)
  }
}

// 8. 监听进度 (从 Preload 接收)
onMounted(() => {
  if (window.api && window.api.onDeployProgress) {
    window.api.onDeployProgress((data) => {
      progress.percent = data.percent
      progress.message = data.message
    })
  }
})
</script>

<style scoped>
/* ============================================
   1. 遮罩层 (全覆盖)
   ============================================ */
/* 强制 dialog 容器相对定位 */
:deep(.el-dialog) {
  position: relative !important;
  overflow: hidden !important;
}

.progress-overlay-fixed {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  z-index: 9999;
  background: rgba(255, 255, 255, 0.85); /* 保持浅色半透明，或使用 panel-bg */
  backdrop-filter: blur(8px);
  display: flex;
  justify-content: center;
  align-items: center;
}
:global(html.dark) .progress-overlay-fixed {
  background: rgba(0, 0, 0, 0.75);
}

.progress-card {
  text-align: center;
  background: var(--bg-color); /* [修改] 全局变量 */
  padding: 30px;
  border-radius: 12px;
  border: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.1);
}
.progress-title {
  margin: 15px 0 5px;
  font-size: 18px;
  color: var(--text-primary); /* [修改] 全局变量 */
}
.progress-desc {
  font-size: 14px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  margin: 0;
}

/* ============================================
   2. 输入框样式统一
   ============================================ */
.setup-form :deep(.el-input__wrapper),
.setup-form :deep(.el-select__wrapper),
.setup-form :deep(.el-textarea__inner) {
  background-color: var(--panel-bg-color) !important; /* [修改] 全局变量 */
  box-shadow: none !important;
  border: 1px solid var(--panel-border-color) !important; /* [修改] 全局变量 */
  border-radius: 6px;
  padding: 4px 8px;
  height: 28px;
  transition: all 0.2s;
  color: var(--text-primary) !important; /* [修改] 确保文字颜色正确 */
}

/* Hover */
.setup-form :deep(.el-input__wrapper:hover),
.setup-form :deep(.el-select__wrapper:hover) {
  background-color: var(--bg-color) !important; /* [修改] 全局变量 */
  border-color: var(--text-secondary) !important; /* [修改] 全局变量 */
}

/* Focus */
.setup-form :deep(.el-input__wrapper.is-focus),
.setup-form :deep(.el-select__wrapper.is-focused) {
  background-color: var(--bg-color) !important; /* [修改] 全局变量 */
  border-color: #409eff !important;
  box-shadow: 0 0 0 1px #409eff !important;
}

/* 数字输入框 */
.setup-form :deep(.el-input-number) {
  width: 100%;
}
.setup-form :deep(.el-input-number .el-input__wrapper) {
  padding-left: 12px !important;
  padding-right: 50px !important;
  padding-top: 1px !important;
  padding-bottom: 1px !important;
}
.setup-form :deep(.el-input-number__decrease),
.setup-form :deep(.el-input-number__increase) {
  background: transparent;
  border-left: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
  color: var(--text-secondary); /* [修改] 全局变量 */
  width: 32px;
}
.setup-form :deep(.el-input-number__increase) {
  border-bottom: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
}
.setup-form :deep(.el-input-number__decrease:hover),
.setup-form :deep(.el-input-number__increase:hover) {
  color: #409eff;
  background-color: rgba(64, 158, 255, 0.1);
}

/* Select */
.setup-form :deep(.el-select__wrapper) {
  padding: 4px 12px;
}

/* Icons */
.setup-form :deep(.el-input__prefix-inner),
.setup-form :deep(.el-select__prefix) {
  color: var(--text-secondary); /* [修改] 全局变量 */
}

/* ============================================
   3. 标签与 Loading
   ============================================ */
:deep(.el-form-item__label) {
  width: 102.5%;
  display: block;
  line-height: 20px;
  margin-bottom: 8px !important;
  color: var(--text-primary); /* [修改] 全局变量 */
}

.label-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}
.label-text {
  font-weight: 500;
  color: var(--text-primary); /* [修改] 全局变量 */
}

.scan-status {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 12px;
  padding: 2px 8px;
  border-radius: 4px;
  background: var(--panel-bg-color); /* [修改] 全局变量 */
  color: var(--text-secondary); /* [修改] 全局变量 */
}
.scan-status.success {
  background: rgba(103, 194, 58, 0.1);
  color: #67c23a;
}
.scan-status.loading {
  background: rgba(144, 147, 153, 0.1);
  color: var(--text-secondary);
}

.status-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background-color: currentColor;
  box-shadow: 0 0 4px currentColor;
}

/* ============================================
   4. 其他组件
   ============================================ */
/* Info Card */
.info-card {
  display: flex;
  gap: 12px;
  background: rgba(64, 158, 255, 0.08);
  border: 1px solid rgba(64, 158, 255, 0.15);
  padding: 12px 16px;
  border-radius: 8px;
  margin-bottom: 24px;
}
.info-icon {
  color: #409eff;
  font-size: 18px;
  margin-top: 2px;
}
.info-text {
  font-size: 13px;
  color: var(--text-primary);
  line-height: 1.5;
} /* [修改] 全局变量 */

/* Layout */
.form-row {
  display: flex;
  gap: 16px;
}
.half-item {
  flex: 1;
}
.input-icon {
  color: var(--text-secondary);
} /* [修改] 全局变量 */

/* Feature Card */
.feature-card {
  display: flex;
  align-items: center;
  padding: 12px 16px;
  background-color: var(--panel-bg-color); /* [修改] 全局变量 */
  border: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  margin-top: 10px;
}
.feature-card:hover {
  border-color: #409eff;
}
.feature-card.active {
  background-color: rgba(64, 158, 255, 0.08);
  border-color: #409eff;
}
.feature-icon {
  font-size: 20px;
  color: var(--text-secondary);
  margin-right: 12px;
  display: flex;
  align-items: center;
}
.feature-card.active .feature-icon {
  color: #409eff;
}
.feature-content {
  flex: 1;
}
.feature-title {
  font-size: 14px;
  font-weight: 500;
  color: var(--text-primary);
  margin-bottom: 2px;
} /* [修改] 全局变量 */
.feature-desc {
  font-size: 12px;
  color: var(--text-secondary);
} /* [修改] 全局变量 */

/* Footer */
.dialog-footer-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.dialog-actions {
  display: flex;
  justify-content: flex-end;
  gap: 2px;
}
.action-btn {
  border-radius: 6px;
  padding: 8px 20px;
  font-weight: 500;
}
.action-btn.confirm {
  background: linear-gradient(135deg, #409eff, #337ecc);
  border: none;
  color: white; /* [修改] 确保白字 */
  box-shadow: 0 3px 10px rgba(64, 158, 255, 0.3);
}
.action-btn.confirm:hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 14px rgba(64, 158, 255, 0.4);
}

/* Autocomplete Item */
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
  color: var(--text-primary);
} /* [修改] 全局变量 */
.device-ip {
  font-size: 12px;
  color: var(--text-secondary);
  font-family: monospace;
} /* [修改] 全局变量 */

.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s;
}
.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}
</style>
