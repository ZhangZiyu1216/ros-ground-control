<template>
  <el-dialog
    v-model="visible"
    title="首次部署 Agent"
    width="480px"
    :close-on-click-modal="false"
    append-to-body
  >
    <el-form ref="formRef" :model="form" label-position="top">
      <el-alert type="info" :closable="false" style="margin-bottom: 20px">
        此功能将通过 SSH 连接到你的机器人，自动上传并安装 Agent 守护进程。
      </el-alert>

      <!-- IP 地址 (带自动补全) -->
      <el-form-item
        label="机器人 IP 地址"
        prop="host"
        :rules="{ required: true, message: 'IP 地址不能为空' }"
      >
        <template #label>
          <div style="display: flex; justify-content: space-between; align-items: center">
            <span>机器人 IP 地址</span>
            <span v-if="sshDevices.length > 0" style="color: #67c23a; font-size: 12px">
              <el-icon style="vertical-align: middle"><Connection /></el-icon> 发现
              {{ sshDevices.length }} 个 SSH 设备
            </span>
            <span v-else style="color: #909399; font-size: 12px">
              <el-icon class="is-loading" style="vertical-align: middle"><Loading /></el-icon>
              正在扫描 SSH...
            </span>
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
          <template #default="{ item }">
            <div style="display: flex; flex-direction: column; line-height: 1.2; padding: 4px 0">
              <span style="font-weight: bold">{{ item.hostname }}</span>
              <span style="font-size: 12px; color: #999">{{ item.value }}</span>
            </div>
          </template>
        </el-autocomplete>
      </el-form-item>

      <el-row :gutter="20">
        <el-col :span="12">
          <el-form-item label="SSH 端口" prop="port">
            <el-input-number
              v-model="form.port"
              :min="1"
              :max="65535"
              controls-position="right"
              style="width: 100%"
            />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="目标架构" prop="arch">
            <el-select v-model="form.arch" placeholder="选择架构">
              <el-option label="ARM64 (树莓派/Jetson)" value="arm64" />
              <el-option label="AMD64 (PC/NUC)" value="amd64" />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-form-item
        label="用户名"
        prop="username"
        :rules="{ required: true, message: '用户名不能为空' }"
      >
        <el-input v-model="form.username" placeholder="例如: ubuntu 或 root" />
      </el-form-item>

      <el-form-item label="密码" prop="password">
        <el-input
          v-model="form.password"
          type="password"
          show-password
          placeholder="请输入 Sudo 密码 (用于安装服务)"
        />
      </el-form-item>

      <!-- 附加组件安装选项 -->
      <el-form-item>
        <el-checkbox v-model="form.installFoxglove" border>
          同时安装 Foxglove Bridge (ROS Noetic)
        </el-checkbox>
        <div style="font-size: 12px; color: #909399; line-height: 1.2; margin-top: 5px">
          * 目标主机需要有互联网连接
        </div>
      </el-form-item>
    </el-form>

    <div v-if="isDeploying" class="progress-overlay">
      <div class="progress-content">
        <el-progress type="circle" :percentage="progress.percent" />
        <p class="progress-text">{{ progress.message }}</p>
      </div>
    </div>

    <template #footer>
      <span class="dialog-footer">
        <el-button @click="visible = false">取消</el-button>
        <el-button type="primary" :loading="isDeploying" @click="handleDeploy">
          连接并部署
        </el-button>
      </span>
    </template>
  </el-dialog>
</template>

<script setup>
import { ref, reactive, computed, watch, toRaw, onMounted } from 'vue'
import { useSshScan } from '../composables/useSshScan'
import { Connection, Loading } from '@element-plus/icons-vue'

const props = defineProps({
  modelValue: Boolean,
  isDeploying: Boolean
})
const emit = defineEmits(['update:modelValue', 'deploy'])

// 1. SSH 扫描逻辑
const { sshDevices, start, stop } = useSshScan()

// 2. 表单状态
const formRef = ref(null)
const form = reactive({
  host: '',
  port: 22,
  username: '',
  password: '',
  arch: 'arm64',
  installFoxglove: true
})
const progress = reactive({ percent: 0, message: '准备中...' })

// 3. 弹窗显示状态绑定
const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

// 4. 监听弹窗开关以控制扫描
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

// 5. 自动补全搜索逻辑
const querySearch = (queryString, cb) => {
  const results = sshDevices.value.map((d) => ({
    value: d.ip, // 填入输入框的值
    hostname: d.hostname // 显示用的辅助信息
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

// 6. 部署动作
const handleDeploy = () => {
  formRef.value.validate((valid) => {
    if (valid) {
      // 传递纯对象给父组件
      emit('deploy', toRaw(form))
    }
  })
}

onMounted(() => {
  window.api.onDeployProgress((data) => {
    progress.percent = data.percent
    progress.message = data.message
  })
})
</script>

<style scoped>
.progress-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(255, 255, 255, 0.9);
  z-index: 10;
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 4px;
}
.progress-content {
  text-align: center;
}
.progress-text {
  margin-top: 15px;
  font-size: 14px;
  color: #606266;
  font-weight: 500;
}
</style>
