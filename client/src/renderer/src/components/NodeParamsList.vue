<template>
  <div class="params-list-container">
    <div class="params-header">
      <span>关联参数文件</span>
      <el-button link type="primary" size="small" @click="addParam">
        <el-icon><Plus /></el-icon> 添加
      </el-button>
    </div>

    <div v-if="params.length === 0" class="empty-params">无关联文件</div>

    <div v-else class="params-items">
      <div v-for="(param, index) in params" :key="index" class="param-row">
        <!-- 1. 参数名称 (可编辑) -->
        <el-input
          v-model="param.name"
          size="small"
          class="param-name-input"
          :placeholder="`参数${index + 1}`"
          @change="triggerSave"
        />

        <!-- 2. 路径按钮 -->
        <el-button
          size="small"
          class="param-path-btn"
          :title="param.path"
          @click="$emit('pick-file', param)"
        >
          {{ getFileName(param.path) || '选择文件' }}
        </el-button>

        <!-- 3. 操作按钮组 -->
        <div class="param-actions">
          <!-- 编辑内容 -->
          <el-button
            link
            type="primary"
            size="small"
            :icon="EditPen"
            :disabled="!param.path"
            title="编辑文件内容"
            @click="openEditor(param.path)"
          />
          <!-- 删除条目 -->
          <el-button
            link
            type="danger"
            size="small"
            :icon="Delete"
            title="移除关联"
            @click="removeParam(index)"
          />
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { computed } from 'vue'
import { Plus, EditPen, Delete } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

const props = defineProps({
  node: { type: Object, required: true },
  backendId: { type: String, required: true }
})

const emit = defineEmits(['update-node', 'pick-file'])

// 确保 params 是数组
const params = computed(() => {
  if (!Array.isArray(props.node.params)) {
    return []
  }
  return props.node.params
})

function getFileName(path) {
  if (!path) return ''
  return path.split('/').pop()
}

// --- 动作逻辑 ---

function addParam() {
  const newParams = [...params.value]
  newParams.push({
    name: '', // 留空则显示 placeholder
    path: ''
  })
  updateParent(newParams)
}

function removeParam(index) {
  const newParams = [...params.value]
  newParams.splice(index, 1)
  updateParent(newParams)
}

function triggerSave() {
  // 这里的 triggerSave 主要是为了响应 input 的 change
  // 实际上 v-model 修改的是 props.node 内部对象的引用（Vue 响应式特性）
  // 但我们需要通知 Dashboard 显式调用 Store 的 save
  updateParent(params.value)
}

// 通知父组件更新并持久化
function updateParent(newParams) {
  // 构造更新后的节点对象
  const updatedNode = {
    ...props.node,
    params: newParams
  }
  emit('update-node', updatedNode)
}

// 打开编辑器 (直接调用 IPC)
async function openEditor(path) {
  if (!path) return
  const fileName = getFileName(path)
  try {
    await window.api.openFileEditor({
      name: fileName,
      path: path,
      backendId: props.backendId,
      backendLabel: props.backendId // 或者传 IP
    })
  } catch (e) {
    ElMessage.error('打开编辑器失败: ' + e.message)
  }
}
</script>

<style scoped>
.params-list-container {
  background-color: #f5f7fa; /* 稍微深一点的背景，区分于 Card */
  border: 1px solid #e4e7ed;
  border-top: none; /* 去掉上边框，使其看起来像从 Card 延伸出来的 */
  border-radius: 0 0 4px 4px; /* 只有底部圆角 */
  padding: 10px 15px;
  margin: 0 4px; /* 稍微缩进一点，营造层次感，或者设为 0 与卡片等宽 */
  box-shadow: 0 2px 4px 0 rgba(0, 0, 0, 0.05);
}

.params-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 12px;
  color: #909399;
  margin-bottom: 8px;
}

.empty-params {
  text-align: center;
  color: #c0c4cc;
  font-size: 12px;
  padding: 5px 0;
}

.param-row {
  display: flex;
  align-items: center;
  gap: 5px;
  margin-bottom: 5px;
}

.param-name-input {
  width: 80px;
  flex-shrink: 0;
}

.param-path-btn {
  flex-grow: 1;
  justify-content: flex-start;
  overflow: hidden;
  text-overflow: ellipsis;
  color: #606266;
}

.param-actions {
  display: flex;
  flex-shrink: 0;
}
</style>
