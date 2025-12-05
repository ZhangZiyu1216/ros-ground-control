<template>
  <el-dialog
    v-model="visible"
    title="添加 ROS 指令"
    width="540px"
    append-to-body
    :close-on-click-modal="false"
    class="command-dialog"
    destroy-on-close
    align-center
  >
    <div class="dialog-body">
      <!-- 1. 顶部：类型选择器 (固定不动) -->
      <div class="type-selector">
        <div
          class="type-item"
          :class="{ active: cmdType === 'rosrun' }"
          @click="cmdType = 'rosrun'"
        >
          <el-icon><VideoPlay /></el-icon>
          <span>rosrun</span>
        </div>
        <div
          class="type-item"
          :class="{ active: cmdType === 'rostopic' }"
          @click="cmdType = 'rostopic'"
        >
          <el-icon><Promotion /></el-icon>
          <span>rostopic pub</span>
        </div>
        <div
          class="type-item"
          :class="{ active: cmdType === 'rosservice' }"
          @click="cmdType = 'rosservice'"
        >
          <el-icon><Connection /></el-icon>
          <span>rosservice call</span>
        </div>
        <div
          class="type-item warning"
          :class="{ active: cmdType === 'bash' }"
          @click="cmdType = 'bash'"
        >
          <el-icon><SetUp /></el-icon>
          <span>Bash</span>
        </div>
      </div>

      <!-- 2. 中间：表单滚动区 (只有这里会滚动) -->
      <div class="form-scroll-area">
        <el-form label-position="top" class="cmd-form">
          <transition name="fade-slide" mode="out-in">
            <!-- Case: rosrun -->
            <div v-if="cmdType === 'rosrun'" key="rosrun" class="form-group">
              <div class="info-tip">
                <el-icon><Box /></el-icon>
                <span>运行包内的可执行节点，请确保当前目录在对应工作空间内</span>
              </div>
              <div class="form-row">
                <el-form-item label="功能包 (Package)" class="half">
                  <el-input v-model="params.pkg" placeholder="e.g. turtlesim" class="modern-input">
                    <template #prefix
                      ><el-icon class="input-icon"><Box /></el-icon
                    ></template>
                  </el-input>
                </el-form-item>
                <el-form-item label="节点名 (Executable)" class="half">
                  <el-input
                    v-model="params.exe"
                    placeholder="e.g. turtlesim_node"
                    class="modern-input"
                  >
                    <template #prefix
                      ><el-icon class="input-icon"><Cpu /></el-icon
                    ></template>
                  </el-input>
                </el-form-item>
              </div>
              <el-form-item label="启动参数 (Args)">
                <el-input
                  v-model="params.args"
                  placeholder="_param:=value __name:=my_node"
                  class="modern-input"
                >
                  <template #prefix><span class="text-icon">_</span></template>
                </el-input>
              </el-form-item>
            </div>

            <!-- Case: rostopic -->
            <div v-else-if="cmdType === 'rostopic'" key="rostopic" class="form-group">
              <el-form-item label="话题名称">
                <el-autocomplete
                  v-model="params.topic"
                  :fetch-suggestions="querySearchTopics"
                  placeholder="/cmd_vel"
                  class="modern-input"
                  trigger-on-focus
                  clearable
                  @select="handleTopicSelect"
                  @blur="handleTopicChange"
                >
                  <template #prefix>
                    <el-icon class="input-icon"><ChatDotRound /></el-icon>
                  </template>
                  <!-- 自定义下拉选项模板 -->
                  <template #default="{ item }">
                    <div class="topic-suggestion-item">
                      <span class="topic-name">{{ item.value }}</span>
                      <span class="topic-type">{{ item.type }}</span>
                    </div>
                  </template>
                </el-autocomplete>
              </el-form-item>
              <el-form-item label="消息类型">
                <el-input
                  v-model="params.type"
                  placeholder="geometry_msgs/Twist"
                  class="modern-input"
                >
                  <template #prefix><span class="text-icon">T</span></template>
                </el-input>
              </el-form-item>
              <el-form-item label="消息内容 (YAML Dictionary)">
                <el-input
                  v-model="params.yaml"
                  type="textarea"
                  :rows="3"
                  placeholder="{linear: {x: 1.0, y: 0.0, z: 0.0}, ...}"
                  class="modern-textarea"
                />
              </el-form-item>
              <div class="loop-control-row">
                <el-checkbox v-model="params.loop" label="循环发布 (Rate)" />
                <transition name="fade-slide">
                  <div v-if="params.loop" class="rate-input-wrapper">
                    <span class="rate-label">频率(Hz):</span>
                    <el-input-number
                      v-model="params.rate"
                      :min="1"
                      :max="100"
                      size="small"
                      controls-position="right"
                      class="rate-number-input"
                    />
                  </div>
                </transition>
              </div>
            </div>

            <!-- Case: rosservice -->
            <div v-else-if="cmdType === 'rosservice'" key="rosservice" class="form-group">
              <el-form-item label="服务名称">
                <!-- [修改] 自动补全服务名 -->
                <el-autocomplete
                  v-model="params.service"
                  :fetch-suggestions="querySearchServices"
                  placeholder="/spawn"
                  class="modern-input"
                  trigger-on-focus
                  clearable
                  @select="handleServiceSelect"
                  @blur="handleServiceChange"
                >
                  <template #prefix
                    ><el-icon class="input-icon"><MagicStick /></el-icon
                  ></template>
                  <template #default="{ item }">
                    <div class="topic-suggestion-item">
                      <span class="topic-name">{{ item.value }}</span>
                      <span class="topic-type">{{ item.type }}</span>
                    </div>
                  </template>
                </el-autocomplete>
              </el-form-item>

              <el-form-item label="请求参数 (YAML Dictionary)">
                <!-- [修改] 改为 Textarea 以支持多行 YAML 显示 -->
                <el-input
                  v-model="params.serviceArgs"
                  type="textarea"
                  :rows="3"
                  placeholder="x: 1.0&#10;y: 2.0&#10;name: 'turtle2'"
                  class="modern-textarea"
                >
                </el-input>
              </el-form-item>
            </div>

            <!-- Case: bash -->
            <div v-else key="bash" class="form-group">
              <div class="info-tip warning">
                <el-icon><WarnTriangleFilled /></el-icon>
                <span>直接执行系统 Shell 命令，请确保指令安全。</span>
              </div>
              <el-form-item label="完整指令">
                <el-input
                  v-model="params.raw"
                  type="textarea"
                  :rows="4"
                  placeholder="e.g. python3 my_script.py --arg1"
                  class="modern-textarea"
                />
              </el-form-item>
            </div>
          </transition>
        </el-form>
      </div>

      <!-- 3. 底部：终端预览 (固定不动) -->
      <div class="terminal-preview">
        <div class="terminal-header">
          <span class="dot red"></span>
          <span class="dot yellow"></span>
          <span class="dot green"></span>
          <span class="terminal-title">命令预览</span>
        </div>
        <div class="terminal-body">
          <span class="prompt">$</span>
          <span v-if="fullPreview" class="cmd-text">{{ fullPreview }}</span>
          <span v-else class="placeholder">等待参数输入...</span>
          <span class="cursor">_</span>
        </div>
      </div>
    </div>

    <template #footer>
      <div class="dialog-actions">
        <el-button class="action-btn cancel" @click="visible = false">取消</el-button>
        <el-button
          class="action-btn confirm"
          type="primary"
          :disabled="!fullPreview"
          @click="confirm"
        >
          确认添加
        </el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script setup>
import { ref, reactive, computed } from 'vue'
import {
  VideoPlay,
  Promotion,
  Connection,
  WarnTriangleFilled,
  Box,
  Cpu,
  ChatDotRound,
  MagicStick,
  SetUp
} from '@element-plus/icons-vue'
import { useFoxglove } from '../composables/useFoxglove'

const props = defineProps({
  modelValue: Boolean,
  currentBackendId: { type: String, default: '' }
})
const emit = defineEmits(['update:modelValue', 'confirm'])
const { getTopicTypeAndTemplate, getTopicsRef, getServicesRef, getServiceTypeAndTemplate } =
  useFoxglove()
const topicsList = getTopicsRef(props.currentBackendId)

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

const cmdType = ref('rosrun')
const params = reactive({
  pkg: '',
  exe: '',
  args: '',
  topic: '',
  type: '',
  yaml: '',
  loop: false,
  rate: 10,
  service: '',
  serviceArgs: '',
  raw: ''
})

// 自动填充逻辑
// 当话题输入框失去焦点或被选中时触发
const handleTopicChange = () => {
  if (cmdType.value !== 'rostopic' || !params.topic || !props.currentBackendId) return
  const info = getTopicTypeAndTemplate(props.currentBackendId, params.topic)
  if (info.type) params.type = info.type
  // 如果内容为空，填入 YAML 模板
  if (info.template && !params.yaml) {
    params.yaml = info.template
  }
}

// 话题搜索逻辑
const querySearchTopics = (queryString, cb) => {
  const list = topicsList.value || []
  const results = queryString
    ? list.filter((t) => t.topic.toLowerCase().includes(queryString.toLowerCase()))
    : list

  // 转换为 autocomplete 需要的格式
  // value 是显示在输入框里的值
  cb(
    results.map((t) => ({
      value: t.topic,
      type: t.schemaName
    }))
  )
}

// 选中话题后的回调
const handleTopicSelect = (item) => {
  params.topic = item.value
  // 立即触发自动填充逻辑
  handleTopicChange()
}

// [新增] 服务列表与搜索
const servicesList = getServicesRef(props.currentBackendId)

const querySearchServices = (queryString, cb) => {
  const list = servicesList.value || []
  const results = queryString
    ? list.filter((s) => s.name.toLowerCase().includes(queryString.toLowerCase()))
    : list

  cb(
    results.map((s) => ({
      value: s.name,
      type: s.type
    }))
  )
}

// [新增] 服务选择回调
const handleServiceSelect = (item) => {
  params.service = item.value
  handleServiceChange()
}

// [新增] 服务自动填充
const handleServiceChange = () => {
  if (cmdType.value !== 'rosservice' || !params.service || !props.currentBackendId) return

  const info = getServiceTypeAndTemplate(props.currentBackendId, params.service)

  // 自动填入参数 (Service 不需要填 type 字段到 UI 上，因为 rostopic 命令行需要 type 而 rosservice 不需要)
  // rosservice call /service_name "args"
  if (info.template && !params.serviceArgs) {
    params.serviceArgs = info.template
  }
}

// 1. 生成参数字符串 (不包含命令本身)
const argsPart = computed(() => {
  switch (cmdType.value) {
    case 'rosrun':
      if (!params.pkg || !params.exe) return null
      return `${params.pkg} ${params.exe} ${params.args}`.trim()

    case 'rostopic': {
      if (!params.topic || !params.type) return null
      let rawYaml = params.yaml || ''
      // 1. 如果 YAML 为空，默认为空对象 (通常 ROS 需要至少一个空字典 {})
      if (!rawYaml.trim()) {
        rawYaml = '{}'
      }
      // 2. 转义处理：
      const safeYaml = rawYaml
        .replace(/\\/g, '\\\\') // 先转义反斜杠自身
        .replace(/"/g, '\\"') // 转义双引号
        .replace(/\$/g, '\\$') // 转义美元符
        .replace(/`/g, '\\`') // 转义反引号
      const modeFlag = params.loop ? `-r ${params.rate}` : '-1'
      // 3. 使用双引号包裹，保留内部换行
      return `pub ${modeFlag} ${params.topic} ${params.type} "${safeYaml}"`
    }

    case 'rosservice': {
      if (!params.service) return null
      let rawServiceArgs = params.serviceArgs || ''
      // 1. 如果为空，默认为空字典 {}
      if (!rawServiceArgs.trim()) {
        rawServiceArgs = '{}'
      }
      // 2. 转义处理 (针对双引号包裹环境)
      const safeServiceArgs = rawServiceArgs
        .replace(/\\/g, '\\\\') // 转义反斜杠
        .replace(/"/g, '\\"') // 转义双引号
        .replace(/\$/g, '\\$') // 转义美元符
        .replace(/`/g, '\\`') // 转义反引号
      // 3. 使用双引号包裹
      return `call ${params.service} "${safeServiceArgs}"`
    }

    case 'bash':
      if (!params.raw) return null
      return params.raw
    default:
      return null
  }
})

// 生成预览命令
const fullPreview = computed(() => {
  if (!argsPart.value) return '等待输入参数...'

  if (cmdType.value === 'bash') return `${argsPart.value}`
  else return `${cmdType.value} ${argsPart.value}`
})

const confirm = () => {
  // 发送拆解后的数据
  emit('confirm', {
    cmd: cmdType.value, // e.g., 'rosrun'
    argsStr: argsPart.value // e.g., 'turtlesim turtlesim_node'
  })
  visible.value = false
}
</script>

<style scoped>
/* ============================================
   1. 布局容器
   ============================================ */
.dialog-body {
  display: flex;
  flex-direction: column;
  height: 480px; /* 固定高度 */
  padding-bottom: 0;
  gap: 15px;

  /* [修改] 不再定义本地变量，直接使用 App.vue 的全局变量 */
  color: var(--text-primary);
}

/* 表单滚动区 */
.form-scroll-area {
  flex: 1;
  overflow-y: auto;
  overflow-x: hidden;
  padding-right: 5px;
  min-height: 0;
  padding-left: 2px; /* 防止 focus 阴影被切 */
  padding-top: 2px;
}
.form-scroll-area::-webkit-scrollbar {
  width: 4px;
}
.form-scroll-area::-webkit-scrollbar-thumb {
  background: var(--divider-color); /* 使用全局分割线颜色 */
  border-radius: 2px;
}

/* ============================================
   2. 类型选择器 (Segmented Control)
   ============================================ */
.type-selector {
  display: flex;
  /* [修改] 使用全局面板背景 */
  background-color: var(--panel-bg-color);
  padding: 4px;
  border-radius: 8px;
  /* [修改] 使用全局面板边框 */
  border: 1px solid var(--panel-border-color);
  flex-shrink: 0;
  margin-bottom: 0;
}

.type-item {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  height: 32px;
  border-radius: 6px;
  font-size: 13px;
  font-weight: 500;
  color: var(--text-secondary); /* 全局次要文字 */
  cursor: pointer;
  transition: all 0.2s;
}

.type-item:hover {
  color: var(--text-primary);
  background-color: rgba(128, 128, 128, 0.1); /* 通用 hover */
}

.type-item.active {
  /* [修改] 激活态背景使用全局背景色 (形成凹凸感) */
  background-color: var(--bg-color);
  color: #409eff;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  font-weight: 600;
}
.type-item.warning.active {
  color: #e6a23c;
}

/* ============================================
   3. 表单样式与输入框覆盖
   ============================================ */
:deep(.el-form-item) {
  margin-bottom: 16px;
}
:deep(.el-form-item__label) {
  font-size: 13px;
  color: var(--text-primary); /* 全局主要文字 */
  padding-bottom: 6px !important;
  line-height: 1.2;
}

/* [核心修复] 输入框统一覆盖 - 使用全局变量 */
.cmd-form :deep(.el-input__wrapper),
.cmd-form :deep(.el-textarea__inner) {
  /* [关键] 背景色使用 panel-bg-color (深色模式下是深灰，浅色下是浅灰) */
  background-color: var(--panel-bg-color) !important;
  box-shadow: none !important;
  /* [关键] 边框使用 panel-border-color */
  border: 1px solid var(--panel-border-color) !important;

  border-radius: 6px;
  transition: all 0.2s;
  padding: 5px 11px !important;
  min-height: 32px;
  line-height: 1.5;

  /* [关键] 文字颜色 */
  color: var(--text-primary) !important;
}

/* Hover */
.cmd-form :deep(.el-input__wrapper:hover),
.cmd-form :deep(.el-textarea__inner:hover) {
  background-color: var(--bg-color) !important; /* 悬浮稍微变亮/变暗 */
  border-color: var(--text-secondary) !important;
}

/* Focus */
.cmd-form :deep(.el-input__wrapper.is-focus),
.cmd-form :deep(.el-textarea__inner:focus) {
  background-color: var(--bg-color) !important;
  border-color: #409eff !important;
  box-shadow: 0 0 0 1px #409eff !important;
}

.input-icon {
  font-size: 16px;
  color: var(--text-secondary);
}
.text-icon {
  font-weight: bold;
  color: var(--text-secondary);
  font-family: monospace;
}

/* 布局类 */
.form-row {
  display: flex;
  gap: 15px;
}
.half {
  flex: 1;
}

/* Info Tip */
.info-tip {
  display: flex;
  align-items: center;
  gap: 8px;
  background: rgba(64, 158, 255, 0.08);
  color: #409eff;
  padding: 8px 12px;
  border-radius: 6px;
  font-size: 12px;
  margin-bottom: 15px;
}
.info-tip.warning {
  background: rgba(230, 162, 60, 0.1);
  color: #e6a23c;
}

/* Loop Control */
.loop-control-row {
  display: flex;
  align-items: center;
  margin-top: -5px;
  height: 32px;
}
.rate-input-wrapper {
  display: flex;
  align-items: center;
  margin-left: 15px;
  gap: 8px;
}
.rate-label {
  font-size: 12px;
  color: var(--text-secondary);
}

/* 数字输入框微调 */
.rate-number-input {
  width: 100px;
}
.rate-number-input :deep(.el-input__wrapper) {
  padding: 2px 8px !important;
  height: 24px;
  min-height: 24px;
}
.rate-number-input :deep(.el-input__inner) {
  height: 24px;
  line-height: 24px;
  font-size: 12px;
}

/* YAML 编辑器字体 */
.modern-textarea :deep(.el-textarea__inner) {
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 12px;
  white-space: pre;
  overflow-x: auto;
}

/* 下拉建议项 */
.topic-suggestion-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
  padding: 4px 0;
}
.topic-name {
  font-weight: 500;
  color: var(--text-primary);
  font-size: 13px;
}
.topic-type {
  font-size: 11px;
  color: var(--text-secondary);
  font-family: monospace;
  margin-left: 10px;
}

/* ============================================
   4. 终端预览 (Terminal)
   ============================================ */
.terminal-preview {
  flex-shrink: 0;
  margin-top: 0;
  /* 终端始终保持深色，不随主题变白，所以这里硬编码深色值 */
  background-color: #1e1e1e;
  border-radius: 8px;
  overflow: auto;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  border: 1px solid var(--panel-border-color);
  height: 100px;
}

.terminal-header {
  background: rgba(255, 255, 255, 0.1);
  padding: 6px 10px;
  display: flex;
  align-items: center;
  gap: 6px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
}
.dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}
.red {
  background: #ff5f56;
}
.yellow {
  background: #ffbd2e;
}
.green {
  background: #27c93f;
}
.terminal-title {
  margin-left: 10px;
  font-size: 12px;
  color: rgba(255, 255, 255, 0.3);
  font-weight: bold;
  letter-spacing: 1px;
}

.terminal-body {
  padding: 15px;
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 13px;
  line-height: 1.5;
  color: #50fa7b; /* 终端绿，始终保持 */
  word-break: break-all;
  min-height: 40px;
}
.prompt {
  color: #bd93f9;
  margin-right: 8px;
  font-weight: bold;
}
.placeholder {
  color: rgba(255, 255, 255, 0.3);
  font-style: italic;
}
.cursor {
  display: inline-block;
  width: 8px;
  height: 14px;
  background-color: #50fa7b;
  vertical-align: middle;
  animation: blink 1s step-end infinite;
  opacity: 0.7;
}
@keyframes blink {
  0%,
  100% {
    opacity: 0.7;
  }
  50% {
    opacity: 0;
  }
}

/* ============================================
   5. Footer
   ============================================ */
.dialog-actions {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}
.action-btn {
  border-radius: 6px;
  padding: 8px 20px;
  font-weight: 500;
}
.action-btn.confirm {
  background: linear-gradient(135deg, #409eff, #337ecc);
  border: none;
  color: white; /* 确保文字是白色 */
  box-shadow: 0 3px 10px rgba(64, 158, 255, 0.3);
}
.action-btn.confirm:hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 14px rgba(64, 158, 255, 0.4);
}
.action-btn.confirm:disabled {
  background: #a0cfff;
  box-shadow: none;
  transform: none;
}

.fade-slide-enter-active,
.fade-slide-leave-active {
  transition: all 0.25s;
}
.fade-slide-enter-from {
  opacity: 0;
  transform: translateX(5px);
}
.fade-slide-leave-to {
  opacity: 0;
  transform: translateX(-5px);
}
</style>
