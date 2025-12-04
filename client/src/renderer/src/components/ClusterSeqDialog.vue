<template>
  <el-dialog
    v-model="visible"
    title="编辑集群启动序列"
    width="680px"
    :close-on-click-modal="false"
    class="cluster-seq-dialog"
    destroy-on-close
    append-to-body
    align-center
  >
    <div class="editor-container">
      <!-- 1. 顶部：序列名称 -->
      <div class="name-section">
        <span class="section-label">序列名称</span>
        <el-input v-model="form.name" placeholder="例如: 全场复位与启动" class="modern-input">
          <template #prefix
            ><el-icon class="input-icon"><EditPen /></el-icon
          ></template>
        </el-input>
      </div>

      <div class="divider"></div>

      <!-- 2. 中部：步骤时间轴 -->
      <div class="steps-section">
        <div class="section-header">
          <span class="section-label">执行流程</span>
          <span class="step-count">{{ form.steps.length }} Steps</span>
        </div>

        <div class="timeline-wrapper">
          <el-scrollbar max-height="400px" view-style="padding: 10px 20px;">
            <transition-group name="list">
              <div v-for="(step, index) in form.steps" :key="index" class="step-item">
                <!-- 连接线 -->
                <div v-if="index < form.steps.length" class="step-line"></div>

                <!-- 图标标记 -->
                <div class="step-marker" :class="step.type === 'delay' ? 'delay' : 'action'">
                  <el-icon v-if="step.type === 'delay'"><Timer /></el-icon>
                  <el-icon v-else><VideoPlay /></el-icon>
                </div>

                <!-- 内容卡片 (紧凑行布局) -->
                <div class="step-card compact-row">
                  <!-- 左侧标签 -->
                  <div class="row-label">
                    <span class="step-type">{{
                      step.type === 'delay' ? '等待延时' : '执行动作'
                    }}</span>
                  </div>

                  <!-- 中间控件区 -->
                  <div class="row-input">
                    <!-- A. 延时模式 -->
                    <div v-if="step.type === 'delay'" class="delay-input-group">
                      <el-input-number
                        v-model="step.content"
                        :min="100"
                        :step="500"
                        controls-position="right"
                        class="modern-number compact"
                      />
                      <span class="unit">ms</span>
                    </div>

                    <!-- B. 动作模式 (三段式级联) -->
                    <div v-else class="action-group">
                      <!-- 1. 选择机器人 -->
                      <el-select
                        v-model="step.agentId"
                        placeholder="选择机器人"
                        class="modern-select compact agent-select"
                        @change="handleAgentChange(step)"
                      >
                        <template #prefix
                          ><el-icon class="input-icon"><Monitor /></el-icon
                        ></template>
                        <el-option
                          v-for="agent in activeAgents"
                          :key="agent.id"
                          :label="agent.name || agent.ip"
                          :value="agent.id"
                        />
                      </el-select>

                      <span class="arrow-separator"
                        ><el-icon><Right /></el-icon
                      ></span>

                      <!-- 2. 选择类型 -->
                      <el-select
                        v-model="step.targetType"
                        placeholder="类型"
                        class="modern-select compact type-select"
                        :disabled="!step.agentId"
                        @change="handleTypeChange(step)"
                      >
                        <el-option label="启动节点" value="node" />
                        <el-option label="执行序列" value="sequence" />
                      </el-select>

                      <span class="arrow-separator"
                        ><el-icon><Right /></el-icon
                      ></span>

                      <!-- 3. 选择具体目标 -->
                      <el-select
                        v-model="step.targetId"
                        placeholder="选择目标"
                        class="modern-select compact target-select"
                        :disabled="!step.agentId || !step.targetType"
                        filterable
                      >
                        <el-option
                          v-for="item in getAgentItems(step)"
                          :key="item.id"
                          :label="item.name"
                          :value="item.id"
                        />
                      </el-select>
                    </div>
                  </div>

                  <!-- 右侧删除 -->
                  <el-button link type="danger" class="delete-btn" @click="removeStep(index)">
                    <el-icon><Close /></el-icon>
                  </el-button>
                </div>
              </div>
            </transition-group>

            <!-- 底部添加栏 -->
            <div class="add-bar">
              <div class="add-btn action" @click="addStep('action')">
                <el-icon><VideoPlay /></el-icon> 添加动作
              </div>
              <div class="add-divider"></div>
              <div class="add-btn delay" @click="addStep('delay')">
                <el-icon><Timer /></el-icon> 添加延时
              </div>
            </div>
          </el-scrollbar>
        </div>
      </div>
    </div>

    <template #footer>
      <div class="dialog-footer">
        <el-button class="action-btn cancel" @click="visible = false">取消</el-button>
        <el-button class="action-btn confirm" type="primary" @click="handleSave"
          >保存序列</el-button
        >
      </div>
    </template>
  </el-dialog>
</template>

<script setup>
import { reactive, computed, watch } from 'vue'
import { useRobotStore } from '../store/robot'
import { VideoPlay, Timer, EditPen, Close, Monitor, Right } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

const props = defineProps({
  modelValue: Boolean,
  editingData: Object // null for new
})
const emit = defineEmits(['update:modelValue', 'save'])

const store = useRobotStore()

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

const form = reactive({
  id: '',
  name: '',
  steps: []
})

// 获取所有在线机器人 (供选择)
const activeAgents = computed(() => {
  return Object.values(store.clients).filter((c) => c.status === 'ready')
})

// 初始化
watch(
  () => props.modelValue,
  (val) => {
    if (val) {
      if (props.editingData) {
        form.id = props.editingData.id
        form.name = props.editingData.name
        // 深拷贝步骤
        form.steps = JSON.parse(JSON.stringify(props.editingData.steps))
      } else {
        form.id = `cluster-${Date.now()}`
        form.name = ''
        form.steps = []
      }
    }
  }
)

// --- Helpers ---
const getAgentItems = (step) => {
  if (!step.agentId) return []
  const client = store.clients[step.agentId]
  if (!client) return []
  return step.targetType === 'node' ? client.nodes : client.sequences
}

// --- Handlers ---
const addStep = (type) => {
  if (type === 'delay') {
    form.steps.push({ type: 'delay', content: 1000 })
  } else {
    // 默认选中第一个在线机器人
    const firstAgent = activeAgents.value.length > 0 ? activeAgents.value[0].id : ''
    form.steps.push({
      type: 'action',
      agentId: firstAgent,
      targetType: 'node',
      targetId: ''
    })
  }
}

const removeStep = (index) => {
  form.steps.splice(index, 1)
}

const handleAgentChange = (step) => {
  step.targetId = '' // 重置目标
}
const handleTypeChange = (step) => {
  step.targetId = '' // 重置目标
}

const handleSave = () => {
  if (!form.name) return ElMessage.warning('请输入序列名称')

  // 校验
  for (let i = 0; i < form.steps.length; i++) {
    const step = form.steps[i]
    if (step.type === 'action') {
      if (!step.agentId || !step.targetId) {
        return ElMessage.warning(`第 ${i + 1} 步配置不完整，请选择机器人和目标`)
      }
    }
  }

  emit('save', JSON.parse(JSON.stringify(form)))
  visible.value = false
}
</script>

<style scoped>
/* ============================================
   1. 变量映射 (Fix: 使用 global 选择器确保变量在 Dialog 根节点生效)
   ============================================ */
:global(.cluster-seq-dialog) {
  /* 引用全局主题变量 (App.vue) */
  --s-bg: var(--bg-color);
  --s-card-bg: var(--panel-bg-color);
  --s-card-border: var(--glass-border);
  --s-text-main: var(--text-primary);
  --s-text-sub: var(--text-secondary);
  --s-line: var(--divider-color);

  /* 本地功能色 */
  --s-primary: #409eff;
  --s-warning: #e6a23c;
  --s-success: #67c23a; /* Action Color */
  --s-danger: #f56c6c;
}

/* 确保内部容器继承文字颜色 */
.editor-container {
  color: var(--s-text-main);
  padding: 0 5px;
}

/* ============================================
   2. 基础元素样式
   ============================================ */
.section-label {
  display: block;
  font-size: 12px;
  font-weight: 600;
  color: var(--s-text-sub);
  margin-bottom: 8px;
  text-transform: uppercase;
}

.divider {
  height: 1px;
  background-color: var(--s-line);
  margin: 20px 0;
  opacity: 0.6;
}

.section-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 4px;
}
.step-count {
  font-size: 12px;
  color: var(--s-text-sub);
  background: var(--s-card-bg);
  padding: 2px 6px;
  border-radius: 10px;
  border: 1px solid var(--s-card-border);
}

/* ============================================
   3. 输入框样式 (Modern Input Universal)
   ============================================ */
/* 使用 !important 覆盖 Element 默认样式，并引用变量 */
.modern-input :deep(.el-input__wrapper),
.modern-select :deep(.el-select__wrapper),
.modern-number :deep(.el-input__wrapper) {
  background-color: var(--s-card-bg) !important;
  box-shadow: none !important;
  border: 1px solid var(--s-card-border) !important;
  border-radius: 8px;
  padding: 8px 12px;
  transition: all 0.2s;
}

.modern-input :deep(.el-input__wrapper:hover),
.modern-select :deep(.el-select__wrapper:hover),
.modern-number :deep(.el-input__wrapper:hover) {
  background-color: var(--s-bg) !important;
  border-color: var(--s-text-sub) !important;
}

.modern-input :deep(.el-input__wrapper.is-focus),
.modern-select :deep(.el-select__wrapper.is-focused),
.modern-number :deep(.el-input__wrapper.is-focus) {
  background-color: var(--s-bg) !important;
  border-color: var(--s-primary) !important;
  box-shadow: 0 0 0 1px var(--s-primary) !important;
}

.input-icon {
  color: var(--s-text-sub);
}

/* Compact Input Overrides (用于列表内) */
.modern-select.compact :deep(.el-select__wrapper),
.modern-number.compact :deep(.el-input__wrapper) {
  padding: 4px 8px !important;
  min-height: 28px !important;
  font-size: 12px;
  background-color: transparent !important;
  border-color: transparent !important;
}
.step-card:hover .modern-select.compact :deep(.el-select__wrapper),
.step-card:hover .modern-number.compact :deep(.el-input__wrapper) {
  background-color: var(--s-bg) !important;
  border-color: var(--s-card-border) !important;
}

/* Input Number Controls */
.modern-number :deep(.el-input-number__decrease),
.modern-number :deep(.el-input-number__increase) {
  background: transparent;
  border-left: 1px solid var(--s-card-border);
  color: var(--s-text-sub);
}
.modern-number :deep(.el-input-number__increase) {
  border-bottom: 1px solid var(--s-card-border);
}

/* ============================================
   4. 时间轴样式
   ============================================ */
.timeline-wrapper {
  position: relative;
}
.timeline-wrapper::before {
  content: '';
  position: absolute;
  top: 30px;
  bottom: 30px;
  left: 24px;
  width: 2px;
  background-color: var(--s-line);
  z-index: 0;
}

.step-item {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-left: -25px;
  margin-bottom: 10px;
  position: relative;
  z-index: 1;
}

.step-marker {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  border: 2px solid var(--s-bg);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.1);
  font-size: 16px;
}
.step-marker.action {
  background-color: var(--s-success);
  color: white;
}
.step-marker.delay {
  background-color: var(--s-warning);
  color: white;
}

/* 卡片容器 */
.step-card {
  flex: 1;
  background-color: var(--s-card-bg);
  border: 1px solid var(--s-card-border);
  border-radius: 8px;
  overflow: hidden;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  padding: 8px 12px;
  gap: 12px;
  min-height: 46px;
}
.step-card:hover {
  border-color: var(--s-primary);
  background-color: var(--s-bg);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05);
}

.row-label {
  flex-shrink: 0;
  width: 60px;
}
.step-type {
  font-size: 12px;
  font-weight: 700;
  color: var(--s-text-sub);
}

.row-input {
  flex: 1;
  display: flex;
  align-items: center;
}

/* Action Group */
.action-group {
  display: flex;
  align-items: center;
  width: 100%;
  gap: 6px;
}
.agent-select {
  flex: 1.5;
  min-width: 100px;
}
.type-select {
  width: 100px;
  flex-shrink: 0;
}
.target-select {
  flex: 2;
  min-width: 120px;
}

.arrow-separator {
  color: var(--s-text-sub);
  opacity: 0.5;
  font-size: 12px;
}

/* Delay Group */
.delay-input-group {
  display: flex;
  align-items: center;
  gap: 10px;
  width: 100%;
}
.modern-number.compact {
  width: 120px !important;
}
.unit {
  font-size: 12px;
  color: var(--s-text-sub);
}

.delete-btn {
  padding: 4px;
  height: auto;
  color: var(--s-text-sub);
  opacity: 0.5;
  transition: all 0.2s;
  border-radius: 4px;
}
.step-card:hover .delete-btn {
  opacity: 1;
}
.delete-btn:hover {
  background-color: rgba(245, 108, 108, 0.1);
  color: var(--s-danger);
}

/* ============================================
   5. 添加栏 & 底部按钮 (Fix Colors)
   ============================================ */
.add-bar {
  margin-top: 10px;
  margin-left: 47px;
  display: flex;
  align-items: center;
  border: 1px dashed var(--s-card-border);
  border-radius: 8px;
  background-color: var(--s-card-bg);
  padding: 5px;
  transition: all 0.2s;
}
.add-bar:hover {
  border-color: var(--s-primary);
  background-color: rgba(64, 158, 255, 0.05);
}

.add-btn {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 8px;
  cursor: pointer;
  border-radius: 6px;
  font-size: 13px;
  color: var(--s-text-sub);
  transition: all 0.2s;
}
.add-btn:hover {
  background-color: rgba(128, 128, 128, 0.05);
  color: var(--s-text-main);
}
/* [修复] 正确使用变量 */
.add-btn.action:hover {
  color: var(--s-success);
  background-color: rgba(103, 194, 58, 0.1);
}
.add-btn.delay:hover {
  color: var(--s-warning);
  background-color: rgba(230, 162, 60, 0.1);
}

.add-divider {
  width: 1px;
  height: 16px;
  background-color: var(--s-card-border);
}

/* Footer */
.dialog-footer {
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}
.action-btn {
  border-radius: 6px;
  padding: 8px 20px;
  font-weight: 500;
  border: none;
}

.action-btn.cancel {
  background-color: var(--s-card-bg);
  color: var(--s-text-main);
  border: 1px solid var(--s-card-border);
}
.action-btn.cancel:hover {
  background-color: var(--s-line);
}

.action-btn.confirm {
  /* [修复] 使用变量定义渐变，或者直接使用变量色 */
  background: linear-gradient(
    135deg,
    #409eff,
    #337ecc
  ); /* 可以保持硬编码，或者用 var(--s-primary) */
  color: white;
  box-shadow: 0 3px 10px rgba(64, 158, 255, 0.3);
}
.action-btn.confirm:hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 14px rgba(64, 158, 255, 0.4);
}

/* Animations */
.list-enter-active,
.list-leave-active {
  transition: all 0.3s ease;
}
.list-enter-from,
.list-leave-to {
  opacity: 0;
  transform: translateX(-20px);
}
</style>
