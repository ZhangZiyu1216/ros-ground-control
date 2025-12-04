<template>
  <el-dialog
    v-model="visible"
    title="序列编辑器"
    width="520px"
    :close-on-click-modal="false"
    class="seq-editor-dialog"
    destroy-on-close
    append-to-body
    align-center
  >
    <div class="editor-container">
      <!-- 1. 顶部：序列名称 -->
      <div class="name-section">
        <span class="section-label">序列名称</span>
        <el-input
          v-model="localForm.name"
          placeholder="给这个启动序列起个名字..."
          class="modern-input"
        >
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
          <span class="step-count">{{ localForm.steps.length }} Steps</span>
        </div>

        <div class="timeline-wrapper">
          <el-scrollbar max-height="350px" view-style="padding: 10px 20px;">
            <!-- 步骤列表 -->
            <transition-group name="list">
              <div v-for="(step, index) in localForm.steps" :key="index" class="step-item">
                <!-- 连接线 (除了最后一个) -->
                <div v-if="index < localForm.steps.length" class="step-line"></div>

                <!-- 步骤序号/图标 -->
                <div class="step-marker" :class="step.type">
                  <el-icon v-if="step.type === 'node'"><Operation /></el-icon>
                  <el-icon v-else><Timer /></el-icon>
                </div>

                <!-- 步骤内容卡片 -->
                <div class="step-card compact-row">
                  <!-- 1. 左侧标签 -->
                  <div class="row-label">
                    <span class="step-type">{{
                      step.type === 'node' ? '启动节点' : '等待延时'
                    }}</span>
                  </div>

                  <!-- 2. 中间控件 -->
                  <div class="row-input">
                    <!-- 类型 A: 节点选择 -->
                    <el-select
                      v-if="step.type === 'node'"
                      v-model="step.content"
                      placeholder="选择节点"
                      class="modern-select compact"
                      filterable
                    >
                      <el-option v-for="n in nodes" :key="n.id" :label="n.name" :value="n.id">
                        <span style="float: left">{{ n.name }}</span>
                        <span
                          style="
                            float: right;
                            color: var(--s-text-sub);
                            font-size: 12px;
                            margin-left: 10px;
                          "
                        >
                          {{ n.cmd }}
                        </span>
                      </el-option>
                    </el-select>

                    <!-- 类型 B: 延时输入 -->
                    <div v-else class="delay-input-group">
                      <el-input-number
                        v-model="step.content"
                        :min="100"
                        :step="500"
                        controls-position="right"
                        class="modern-number compact"
                      />
                      <span class="unit">ms</span>
                    </div>
                  </div>

                  <!-- 3. 右侧删除 -->
                  <el-button link type="danger" class="delete-btn" @click="removeStep(index)">
                    <el-icon><Close /></el-icon>
                  </el-button>
                </div>
              </div>
            </transition-group>

            <!-- 底部添加栏 -->
            <div class="add-bar">
              <div class="add-btn node" @click="addStep('node')">
                <el-icon><Plus /></el-icon> 节点
              </div>
              <div class="add-divider"></div>
              <div class="add-btn delay" @click="addStep('delay')">
                <el-icon><Timer /></el-icon> 延时
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
import { EditPen, Operation, Timer, Close, Plus } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

const props = defineProps({
  modelValue: Boolean,
  nodes: { type: Array, default: () => [] }, // 可选节点列表
  initialData: { type: Object, default: null } // 编辑时的初始数据
})

const emit = defineEmits(['update:modelValue', 'save'])

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

const localForm = reactive({
  id: '',
  name: '',
  steps: []
})

// 监听打开，初始化数据
watch(
  () => props.modelValue,
  (val) => {
    if (val) {
      if (props.initialData) {
        localForm.id = props.initialData.id
        localForm.name = props.initialData.name
        // 深拷贝步骤，防止直接修改原数据
        localForm.steps = JSON.parse(JSON.stringify(props.initialData.steps))
      } else {
        resetForm()
      }
    }
  }
)

const resetForm = () => {
  localForm.id = ''
  localForm.name = ''
  localForm.steps = []
}

const addStep = (type) => {
  if (type === 'node') {
    // 默认选中第一个节点
    const firstNodeId = props.nodes.length > 0 ? props.nodes[0].id : ''
    localForm.steps.push({ type: 'node', content: firstNodeId })
  } else {
    localForm.steps.push({ type: 'delay', content: 1000 })
  }
}

const removeStep = (index) => {
  localForm.steps.splice(index, 1)
}

const handleSave = () => {
  if (!localForm.name.trim()) return ElMessage.warning('请输入序列名称')

  // 简单校验
  for (const step of localForm.steps) {
    if (step.type === 'node' && !step.content) {
      return ElMessage.warning('请为所有节点步骤选择目标节点')
    }
  }

  emit('save', { ...localForm })
  // visible.value = false // 由父组件控制关闭，或者在这里关闭
}
</script>

<style scoped>
/* ============================================
   1. 变量映射 (Mapping to Global Theme)
   ============================================ */
.editor-container {
  /* [核心修复] 引用 App.vue 定义的全局变量 */
  --s-bg: var(--bg-color); /* 整体背景 / 输入框悬浮背景 */
  --s-card-bg: var(--panel-bg-color); /* 卡片背景 / 输入框默认背景 */
  --s-card-border: var(--glass-border); /* 边框 */
  --s-text-main: var(--text-primary); /* 主文字 */
  --s-text-sub: var(--text-secondary); /* 次文字 */
  --s-line: var(--divider-color); /* 线条 */

  /* 功能色 (保持独立定义) */
  --s-primary: #409eff;
  --s-warning: #e6a23c;
}

/* 确保文字颜色继承正确 */
.editor-container {
  color: var(--s-text-main);
  padding: 0 5px;
}

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

/* ============================================
   2. 输入框样式 (Modern Input)
   ============================================ */
/* 
   注意：这里使用了 !important 来强制覆盖 Element Plus 默认样式
   配合上面的变量映射，深色模式下会自动变为深色背景
*/
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

/* Input Number 特殊处理 */
.modern-number {
  width: 120px !important;
}
.modern-number :deep(.el-input-number__decrease),
.modern-number :deep(.el-input-number__increase) {
  background: transparent;
  border-left: 1px solid var(--s-card-border);
  color: var(--s-text-sub);
}
.modern-number :deep(.el-input-number__increase) {
  border-bottom: 1px solid var(--s-card-border);
}
.modern-number :deep(.el-input-number__decrease:hover),
.modern-number :deep(.el-input-number__increase:hover) {
  color: var(--s-primary);
  background-color: rgba(64, 158, 255, 0.1);
}

/* ============================================
   3. 时间轴样式 (Custom Timeline)
   ============================================ */
.section-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
}
.step-count {
  font-size: 12px;
  color: var(--s-text-sub);
  background: var(--s-card-bg);
  padding: 2px 6px;
  border-radius: 10px;
  border: 1px solid var(--s-card-border);
}

.timeline-wrapper {
  position: relative;
  /* 左侧贯穿线 */
  &::before {
    content: '';
    position: absolute;
    top: 20px;
    bottom: 60px;
    left: 18px;
    width: 2px;
    background-color: var(--s-line);
    z-index: 0;
  }
}

.step-item {
  display: flex;
  align-items: flex-start;
  gap: 15px;
  margin-left: -33px;
  margin-bottom: 15px;
  position: relative;
  z-index: 1;
}

/* 序号/图标标记 */
.step-marker {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  /* 这里的 border 需要跟随背景色，形成"切断线条"的视觉效果 */
  border: 2px solid var(--s-bg);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.1);
  font-size: 16px;
}
.step-marker.node {
  background-color: var(--s-primary);
  color: white;
}
.step-marker.delay {
  background-color: var(--s-warning);
  color: white;
}

/* 内容卡片容器 */
.step-card {
  flex: 1;
  background-color: var(--s-card-bg);
  border: 1px solid var(--s-card-border);
  border-radius: 8px;
  overflow: hidden;
  transition: all 0.2s;

  /* [修改]改为 Flex 行布局 */
  display: flex;
  align-items: center;
  padding: 8px 12px; /* 统一内边距 */
  gap: 12px;
  min-height: 46px; /* 保证高度统一 */
}

.step-card:hover {
  border-color: var(--s-primary);
  background-color: var(--s-bg);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05);
}

/* 左侧标签 */
.row-label {
  flex-shrink: 0;
  width: 60px; /* 固定宽度，对齐 */
}
.step-type {
  font-size: 12px;
  font-weight: 700;
  color: var(--s-text-sub);
}

/* 中间输入区 */
.row-input {
  flex: 1; /* 占据剩余空间 */
  display: flex;
  align-items: center;
}
:deep(.el-input-number.is-center .el-input__inner) {
  text-align: left;
}

/* 右侧删除按钮 */
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
  color: #f56c6c;
}

/* --- 输入控件微调 (Compact Mode) --- */
.modern-select.compact,
.modern-number.compact {
  width: 100% !important; /* 填满 row-input */
}

/* 覆盖之前定义的 padding，使其在行内更紧凑 */
.modern-select.compact :deep(.el-select__wrapper),
.modern-number.compact :deep(.el-input__wrapper) {
  padding: 4px 8px !important;
  min-height: 30px !important;
  font-size: 13px;
  background-color: transparent !important; /* 融入卡片背景 */
  border-color: transparent !important; /* 默认无边框，像纯文本 */
}

/* 悬浮/聚焦时显示边框，增强交互感 */
.step-card:hover .modern-select.compact :deep(.el-select__wrapper),
.step-card:hover .modern-number.compact :deep(.el-input__wrapper) {
  background-color: var(--s-bg) !important;
  border-color: var(--s-card-border) !important;
}

.modern-select.compact :deep(.el-select__wrapper.is-focused),
.modern-number.compact :deep(.el-input__wrapper.is-focus) {
  background-color: var(--s-bg) !important;
  border-color: var(--s-primary) !important;
  box-shadow: 0 0 0 1px var(--s-primary) !important;
}

/* 延时单位 */
.delay-input-group {
  width: 100%;
  display: flex;
  align-items: center;
  gap: 8px;
}
.unit {
  font-size: 12px;
  color: var(--s-text-sub);
  white-space: nowrap;
}

/* ============================================
   4. 添加栏 (Add Bar)
   ============================================ */
.add-bar {
  margin-top: 10px;
  margin-left: 47px; /* 对齐卡片左边 */
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
  transition: background 0.2s;
}
.add-btn:hover {
  background-color: rgba(128, 128, 128, 0.05);
  color: var(--s-text-main);
}
.add-btn.node:hover {
  color: var(--s-primary);
}
.add-btn.delay:hover {
  color: var(--s-warning);
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
