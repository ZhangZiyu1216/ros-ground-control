<template>
  <div class="params-list-container" @click.stop>
    <div class="params-header">
      <span>关联参数文件</span>
      <el-button link type="primary" size="small" :disabled="disabled" @click.stop="addParam">
        <el-icon><Plus /></el-icon> 添加
      </el-button>
    </div>

    <div v-if="localParams.length === 0" class="empty-params">无关联文件</div>

    <div v-else class="params-items">
      <div v-for="(param, index) in localParams" :key="index" class="param-row">
        <!-- 1. 参数名称 -->
        <!-- 注意：这里用 @change 而不是 @input，意味着只有失去焦点或回车时才触发保存 -->
        <el-input
          v-model="param.name"
          size="small"
          class="param-name-input"
          :placeholder="`参数${index + 1}`"
          :disabled="disabled"
          @change="triggerSave"
        />

        <!-- 2. 路径按钮 -->
        <el-button
          size="small"
          class="param-path-btn"
          :title="param.path"
          :disabled="disabled"
          @click.stop="handlePickFile(param)"
        >
          {{ getFileName(param.path) || '选择文件' }}
        </el-button>

        <!-- 3. 操作按钮 -->
        <div class="param-actions">
          <el-button
            link
            type="primary"
            size="small"
            :icon="EditPen"
            :disabled="disabled || !param.path"
            title="编辑内容"
            @click.stop="openEditor(param.path)"
          />
          <el-button
            link
            type="danger"
            size="small"
            :icon="Delete"
            :disabled="disabled"
            title="移除"
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
  emit('pick-file', {
    paramObject: param, // 传递引用，父组件修改这个对象的 path
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
