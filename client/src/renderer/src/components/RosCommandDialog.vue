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
                <el-input v-model="params.service" placeholder="/spawn" class="modern-input">
                  <template #prefix
                    ><el-icon class="input-icon"><MagicStick /></el-icon
                  ></template>
                </el-input>
              </el-form-item>
              <el-form-item label="请求参数 (YAML List)">
                <el-input
                  v-model="params.serviceArgs"
                  placeholder="[x, y, theta, name]"
                  class="modern-input"
                >
                  <template #prefix><span class="text-icon">{}</span></template>
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
const { getTopicTypeAndTemplate, getTopicsRef } = useFoxglove()
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
      const safeArgs = params.serviceArgs.replace(/'/g, "'\\''")
      return `call ${params.service} "${safeArgs}"`
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
  return `${cmdType.value} ${argsPart.value}`
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
   1. 变量定义
   ============================================ */
.dialog-body {
  /* [核心] 使用 Flex 列布局 */
  display: flex;
  flex-direction: column;
  /* [核心] 给定一个合适的高度，防止跳动 */
  height: 400px;
  padding-bottom: 0; /* 底部padding由gap处理 */
  gap: 15px; /* 各个大块之间的间距 */
  /* Light Mode */
  --c-bg: #ffffff;
  --c-card-bg: #f5f7fa;
  /* [修改] 加深浅色模式边框，确保肉眼可见 */
  --c-card-border: #dcdfe6;
  --c-primary: #409eff;
  --c-text-main: #303133;
  --c-text-sub: #909399;
  /* 终端配色 (Light 模式下也保持深色终端) */
  --c-term-bg: #1e1e1e;
  --c-term-text: #50fa7b;
}

/* Dark Mode 适配 */
:global(html.dark) .dialog-body {
  --c-bg: #1e1e20;
  --c-card-bg: #2b2b2d;
  --c-card-border: #414243;
  --c-primary: #409eff;
  --c-text-main: #e5eaf3;
  --c-text-sub: #a3a6ad;
  --c-term-bg: #000000;
  --c-term-text: #50fa7b;
}

.form-scroll-area {
  flex: 1; /* 占据剩余空间 */
  overflow-y: auto; /* 超出则滚动 */
  overflow-x: hidden;
  padding-right: 5px; /* 预留滚动条空间 */
  min-height: 0; /* 防止 Flex 子项溢出 */

  /* 增加一点内边距防止输入框阴影被切 */
  padding-left: 2px;
  padding-top: 2px;
}

/* 美化滚动条 */
.form-scroll-area::-webkit-scrollbar {
  width: 4px;
}
.form-scroll-area::-webkit-scrollbar-thumb {
  background: var(--c-card-border);
  border-radius: 2px;
}

/* ============================================
   2. 类型选择器 (Segmented Control)
   ============================================ */
.type-selector {
  display: flex;
  background-color: var(--c-card-bg);
  padding: 4px;
  border-radius: 8px;
  margin-bottom: 20px;
  border: 1px solid var(--c-card-border);
  flex-shrink: 0; /* 禁止压缩 */
  margin-bottom: 0; /* 由 gap 控制 */
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
  color: var(--c-text-sub);
  cursor: pointer;
  transition: all 0.2s;
}

.type-item:hover {
  color: var(--c-text-main);
  background-color: rgba(0, 0, 0, 0.03);
}
:global(html.dark) .type-item:hover {
  background-color: rgba(255, 255, 255, 0.05);
}

.type-item.active {
  background-color: var(--c-bg);
  color: var(--c-primary);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  font-weight: 600;
}
.type-item.warning.active {
  color: #e6a23c;
}

/* ============================================
   3. 表单样式
   ============================================ */
:deep(.el-form-item) {
  margin-bottom: 16px; /* 减小表单项间距 */
}
/* 强制 Label 样式 */
:deep(.el-form-item__label) {
  font-size: 13px;
  color: var(--c-text-main);
  padding-bottom: 6px !important;
  line-height: 1.2;
}

/* 输入框统一覆盖 */
.cmd-form :deep(.el-input__wrapper),
.cmd-form :deep(.el-textarea__inner) {
  background-color: var(--c-card-bg) !important;
  box-shadow: none !important;
  border: 1px solid var(--c-card-border) !important;
  border-radius: 6px;
  transition: all 0.2s;
  padding: 5px 11px !important;
  min-height: 32px;
  line-height: 1.5;
}
.cmd-form :deep(.el-input__wrapper:hover),
.cmd-form :deep(.el-textarea__inner:hover) {
  background-color: var(--c-bg) !important;
  border-color: var(--c-text-sub) !important;
}
.cmd-form :deep(.el-input__wrapper.is-focus),
.cmd-form :deep(.el-textarea__inner:focus) {
  background-color: var(--c-bg) !important;
  border-color: var(--c-primary) !important;
  box-shadow: 0 0 0 1px var(--c-primary) !important;
}

.input-icon {
  font-size: 16px;
  color: var(--c-text-sub);
}
.text-icon {
  font-weight: bold;
  color: var(--c-text-sub);
  font-family: monospace;
}

/* 布局 */
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
  color: var(--c-primary);
  padding: 8px 12px;
  border-radius: 6px;
  font-size: 12px;
  margin-bottom: 15px;
}
.info-tip.warning {
  background: rgba(230, 162, 60, 0.1);
  color: #e6a23c;
}

/* Checkbox */
.checkbox-wrapper {
  margin-top: -10px;
}

.loop-control-row {
  display: flex;
  align-items: center;
  margin-top: -5px;
  height: 32px; /* 占位高度防止跳动 */
}

.rate-input-wrapper {
  display: flex;
  align-items: center;
  margin-left: 15px;
  gap: 8px;
}

.rate-label {
  font-size: 12px;
  color: var(--c-text-sub);
}

/* 频率数字输入框微调 */
.rate-number-input {
  width: 100px;
}
.rate-number-input :deep(.el-input__wrapper) {
  padding: 2px 8px !important; /* 进一步压缩高度 */
  height: 24px;
  min-height: 24px;
}
.rate-number-input :deep(.el-input__inner) {
  height: 24px;
  line-height: 24px;
  font-size: 12px;
}
.modern-textarea :deep(.el-textarea__inner) {
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 12px;
  white-space: pre; /* 保留空白和换行 */
  overflow-x: auto; /* 允许横向滚动 */
}

.topic-suggestion-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
  padding: 4px 0;
}

.topic-name {
  font-weight: 500;
  color: var(--c-text-main);
  font-size: 13px;
}

.topic-type {
  font-size: 11px;
  color: var(--c-text-sub);
  font-family: monospace; /* 消息类型用等宽字体更好看 */
  margin-left: 10px;
}

/* ============================================
   4. 终端预览 (Terminal)
   ============================================ */
.terminal-preview {
  flex-shrink: 0; /* 禁止压缩 */
  margin-top: 0; /* 由 gap 控制 */
  /* [核心] 强制应用背景色变量，若变量失效则回退到黑色 */
  background-color: var(--c-term-bg, #1e1e1e);
  border-radius: 8px;
  overflow: auto;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  border: 1px solid var(--c-card-border);
  height: 80px;
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
  color: var(--c-term-text);
  word-break: break-all;
  min-height: 40px; /* 防止内容为空时高度塌陷 */
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

/* [核心] 光标动画修复 */
.cursor {
  display: inline-block;
  width: 8px;
  height: 14px;
  background-color: var(--c-term-text); /* 实心光标比下划线更明显 */
  vertical-align: middle;
  animation: blink 1s step-end infinite; /* 使用 step-end 让闪烁更像终端 */
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

<style>
html.dark .el-autocomplete-suggestion__wrap {
  background-color: #2b2b2d; /* 与 --c-card-bg 一致 */
  border: 1px solid #414243;
}
html.dark .el-autocomplete-suggestion__list li {
  color: #e5eaf3;
}
html.dark .el-autocomplete-suggestion__list li:hover {
  background-color: rgba(64, 158, 255, 0.1);
}
</style>
