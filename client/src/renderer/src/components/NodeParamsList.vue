<template>
  <div class="params-list-container" @click.stop>
    <!-- 顶部标题栏 -->
    <div class="params-header">
      <span class="header-title">参数列表</span>
      <el-button
        link
        type="primary"
        size="small"
        :disabled="disabled"
        class="add-param-btn"
        @click.stop="addParam"
      >
        <el-icon><Plus /></el-icon> 添加
      </el-button>
    </div>

    <!-- 空状态 -->
    <div v-if="localParams.length === 0" class="empty-params">
      <span class="empty-dot"></span> 暂无参数文件
    </div>

    <!-- 参数列表 -->
    <div v-else class="params-items">
      <div v-for="(param, index) in localParams" :key="index" class="param-row">
        <!-- 1. 参数名称 (透明输入框) -->
        <el-input
          v-model="param.name"
          size="small"
          class="param-name-input"
          :placeholder="`Param ${index + 1}`"
          :disabled="disabled"
          @change="triggerSave"
        />

        <!-- 2. 路径按钮 (文件胶囊) -->
        <div
          class="file-capsule"
          :class="{ 'is-empty': !param.path, 'is-disabled': disabled }"
          :title="param.path"
          @click.stop="!disabled && handlePickFile(param)"
        >
          <span class="file-name">{{ getFileName(param.path) || '点击选择文件...' }}</span>
        </div>

        <!-- 3. 操作按钮 (悬浮显示) -->
        <div class="param-actions">
          <el-button
            link
            class="action-icon edit"
            size="small"
            :icon="EditPen"
            :disabled="disabled || !param.path"
            @click.stop="openEditor(param.path)"
          />
          <el-button
            link
            class="action-icon delete"
            size="small"
            :icon="Delete"
            :disabled="disabled"
            @click.stop="removeParam(index)"
          />
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, watch, toRaw } from 'vue'
import { Plus, EditPen, Delete } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

const props = defineProps({
  node: { type: Object, required: true },
  backendId: { type: String, required: true },
  disabled: { type: Boolean, default: false }
})

const emit = defineEmits(['update-node', 'pick-file'])

// [核心] 本地草稿数据
const localParams = ref([])

// 监听 Props 变化同步到本地 (单向数据流：Store -> UI)
// 只有当 backend 真正更新了数据时，才重置本地草稿
watch(
  () => props.node.params,
  (newVal) => {
    if (Array.isArray(newVal)) {
      // 深拷贝，断开引用，防止 v-model 直接修改 Store
      localParams.value = JSON.parse(JSON.stringify(newVal))
    } else {
      localParams.value = []
    }
  },
  { immediate: true, deep: true }
)

function getFileName(path) {
  if (!path) return ''
  return path.split('/').pop()
}

// --- 动作逻辑 ---

// 1. 添加
function addParam() {
  localParams.value.push({ name: '', path: '' })
}

// 2. 删除
function removeParam(index) {
  localParams.value.splice(index, 1)
  triggerSave()
}

// 3. 触发保存：将本地草稿发送给父组件
function triggerSave() {
  // 检查是否有“半成品”条目
  // 只要有一个条目的 name 或 path 为空，就视为“编辑中”，不进行保存
  const hasIncompleteParam = localParams.value.some((p) => !p.name || !p.path)

  if (hasIncompleteParam) {
    // 处于草稿状态，只更新本地 UI，不通知 Store，防止被后端清洗掉
    console.log('当前有未完成的参数填写，暂不保存')
    return
  }

  // 只有当所有条目都有效时，才发送给后端
  const updatedNode = {
    ...props.node,
    params: toRaw(localParams.value)
  }
  emit('update-node', updatedNode)
}

// 4. 选择文件：这是最复杂的交互
// 因为文件选择器在父组件，我们需要把“当前操作的对象”和“保存回调”一起传出去
function handlePickFile(param) {
  const referencePath = props.node.args && props.node.args.length > 0 ? props.node.args[0] : null
  emit('pick-file', {
    paramObject: param, // 传递引用，父组件修改这个对象的 path
    referencePath: referencePath,
    onSuccess: () => triggerSave() // 提供一个回调，父组件选完文件后调用它来触发保存
  })
}

// 打开编辑器
async function openEditor(path) {
  if (!path) return
  const fileName = getFileName(path)
  try {
    await window.api.openFileEditor({
      name: fileName,
      path: path,
      backendId: props.backendId,
      backendLabel: props.backendId
    })
  } catch (e) {
    ElMessage.error('打开编辑器失败: ' + e.message)
  }
}
</script>

<style scoped>
/* ============================================
   容器样式 (Drawer Tray Style)
   ============================================ */
.params-list-container {
  /* 视觉位置调整：稍微缩进，营造层次感 */
  margin: 0 18px 2px 10px;
  padding: 15px 15px 15px 15px;

  /* 背景：轻微的半透明底色，比卡片深一点 */
  background-color: var(--panel-bg-color);

  /* 边框 */
  border: 1px solid var(--divider-color);
  border-radius: 0 0 16px 16px; /* 底部圆角 */

  /* 动画过渡 */
  transition: all 0.3s ease;
}

/* ============================================
   Header & Controls
   ============================================ */
.params-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
  padding-bottom: 5px;
  border-bottom: 1px dashed var(--divider-color, #e4e7ed); /* 使用 CSS 变量 */
}

.header-title {
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 1px;
  color: var(--text-secondary, #909399);
  text-transform: uppercase; /* 英文大写，更像仪表盘 */
}

.add-param-btn {
  font-size: 12px;
  font-weight: 600;
}

/* ============================================
   列表内容
   ============================================ */
.param-row {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 6px;
  /* 增加高度稳定性 */
  height: 28px;
}

/* 1. 参数名输入框 - 极简风格 */
.param-name-input {
  width: 80px;
  flex-shrink: 0;
}
/* 穿透修改 Element 输入框样式：去背景、去边框 */
.param-name-input :deep(.el-input__wrapper) {
  background-color: transparent !important;
  box-shadow: none !important;
  padding: 0 5px !important;
  transition: all 0.2s;
}
/* 聚焦或悬浮时显示下划线 */
.param-name-input:hover :deep(.el-input__wrapper),
.param-name-input :deep(.el-input__wrapper.is-focus) {
  box-shadow: 0 1px 0 0 var(--el-color-primary) !important;
  border-radius: 0;
}
.param-name-input :deep(.el-input__inner) {
  font-size: 13px;
  color: var(--text-primary, #606266);
  font-weight: 500;
  text-align: left; /* 让文字靠右，靠近文件路径 */
}
.add-param-btn {
  display: flex;
  align-items: center;
}
.add-param-btn :deep(.el-icon) {
  margin-right: 4px;
}

/* 2. 文件路径胶囊 (File Capsule) */
.file-capsule {
  flex-grow: 1;
  background-color: rgba(255, 255, 255, 0.6);
  border: 1px solid transparent;
  border-radius: 12px;
  padding: 0 8px;
  height: 24px;
  display: flex;
  align-items: center;
  font-size: 12px;
  color: var(--text-primary, #606266);
  cursor: pointer;
  transition: all 0.2s;
  overflow: hidden;
}
/* 交互效果 */
.file-capsule:hover:not(.is-disabled) {
  background-color: #fff;
  border-color: var(--add-card-text, #409eff);
  color: var(--add-card-text, #409eff);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.05);
}
.file-capsule.is-empty {
  color: var(--text-secondary, #909399);
  background-color: rgba(0, 0, 0, 0.03);
}
.file-capsule.is-disabled {
  cursor: not-allowed;
  opacity: 0.7;
}
.file-name {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
/* 深色模式适配 */
:global(html.dark) .file-capsule {
  background-color: rgba(255, 255, 255, 0.05);
}
:global(html.dark) .file-capsule:hover:not(.is-disabled) {
  background-color: rgba(255, 255, 255, 0.1);
}

/* 3. 操作按钮 */
.param-actions {
  display: flex;
  opacity: 0.6; /* 默认半透明，减少干扰 */
  transition: opacity 0.2s;
}
.param-row:hover .param-actions {
  opacity: 1; /* 鼠标移入行时高亮 */
}

.action-icon {
  padding: 4px;
  margin-left: 2px;
  height: auto;
}
.action-icon.edit:hover {
  color: var(--el-color-primary);
}
.action-icon.delete:hover {
  color: var(--el-color-danger);
}

/* 空状态 */
.empty-params {
  text-align: center;
  color: var(--text-secondary, #c0c4cc);
  font-size: 12px;
  padding: 5px 0;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
}
.empty-dot {
  width: 4px;
  height: 4px;
  background-color: #dcdfe6;
  border-radius: 50%;
}
</style>
