<!-- src/renderer/src/components/Dashboard.vue -->
<template>
  <el-container class="dashboard">
    <!-- 节点控制部分 -->
    <el-aside class="node-list-panel">
      <el-card
        v-for="node in nodes"
        :key="node.id || node.name"
        class="node-card"
        :class="{ 'is-editing': editingNodeName === node.id }"
        @mouseover="node.showEdit = true"
        @mouseleave="node.showEdit = false"
      >
        <template #header>
          <div class="card-header">
            <div class="header-row">
              <el-tag size="small" :type="statusType(node.status)" />
            </div>
            <div class="header-row">
              <!-- 编辑模式的标题输入框 -->
              <el-input
                v-if="editingNodeId === node.id"
                v-model="form.name"
                placeholder="Display Name"
                size="small"
                class="name-input"
              />
              <!-- 显示模式的标题 -->
              <span v-else class="node-name">{{ node.name }}</span>
            </div>
          </div>
        </template>

        <!-- 卡片内容部分 -->
        <div class="card-body-content">
          <!-- 编辑模式 -->
          <el-button
            v-if="editingNodeId === node.id"
            class="launch-path-button"
            @click="openFileBrowser"
          >
            {{ form.path || '选择启动文件' }}
          </el-button>
          <!-- 显示模式 -->
          <p v-else class="launch-path">{{ getDisplayPath(node.args) }}</p>
        </div>

        <template #footer>
          <div class="card-footer">
            <!-- 编辑模式: 显示 Save/Cancel 按钮 -->
            <div v-if="editingNodeId === node.id" class="footer-edit-actions">
              <el-button type="primary" size="small" round @click="saveNode">保存</el-button>
              <el-button size="small" round @click="cancelEditing">取消</el-button>
            </div>

            <!-- 显示模式: 显示 启动/停止 和 编辑按钮 -->
            <div v-else class="footer-display-actions">
              <!-- 删除图标（和编辑图标同样式，因此使用一个class） -->
              <el-button
                v-show="node.showEdit"
                class="delete-icon-btn"
                :icon="Delete"
                type="danger"
                circle
                link
                @click="deleteNode(node)"
              />
              <!-- 编辑图标 -->
              <el-button
                v-show="node.showEdit"
                class="edit-icon-btn"
                :icon="Edit"
                :disabled="!isCurrentBackendReady"
                type="primary"
                circle
                link
                @click="startEditing(node)"
              />
              <el-button
                class="start-stop-btn"
                :type="node.status === 'running' ? 'danger' : 'primary'"
                :loading="node.status === 'starting'"
                :disabled="node.status === 'starting' || !isCurrentBackendReady"
                round
                @click="toggleNode(node)"
              >
                {{ node.status === 'running' ? '停止' : '启动' }}
              </el-button>
            </div>
          </div>
        </template>
      </el-card>

      <!-- “新增节点”卡片 -->
      <el-card v-if="editingNodeId === '__new__'" class="node-card is-editing">
        <template #header>
          <div class="card-header">
            <div class="header-row">
              <el-tag size="small" type="info" />
            </div>
            <div class="header-row">
              <el-input
                v-model="form.name"
                placeholder="节点名称"
                size="small"
                class="name-input"
              />
            </div>
          </div>
        </template>
        <div class="card-body-content">
          <el-button class="launch-path-button" @click="openFileBrowser">
            {{ getDisplayPath(form.args) || '选择launch文件' }}
          </el-button>
        </div>
        <template #footer>
          <div class="card-footer">
            <div class="footer-edit-actions">
              <el-button type="primary" size="small" @click.stop="saveNode">添加</el-button>
              <el-button size="small" @click.stop="cancelEditing">取消</el-button>
            </div>
          </div>
        </template>
      </el-card>
      <!-- “添加新节点”的占位符按钮 -->
      <el-card v-else class="node-card add-new-card" @click="startAdding">
        <span
          :style="{
            fontSize: '2.5vh',
            color: '#409EFF',
            marginLeft: '0.5vw'
          }"
          >+ 添加新节点</span
        >
      </el-card>
    </el-aside>
    <!-- 右侧：功能区 -->
    <el-main class="function-panel">
      <div class="auto-start-panel">
        <h2>一键启动序列</h2>
        <!-- 占位符，未来放置序列控制 -->
        <p>此处将放置一键启动序列的控制组件。</p>
      </div>
      <div class="rosbag-panel">
        <h2>ROSbag 录制/播放</h2>
        <!-- 占位符，未来放置录制/播放控制 -->
        <p>此处将放置ROSbag录制和播放的控制组件。</p>
      </div>
    </el-main>
  </el-container>
  <el-dialog
    v-model="isFileBrowserVisible"
    class="file-browser-dialog"
    destroy-on-close
    :close-on-click-modal="false"
    draggable
    :style="{
      '--el-dialog-width': '75vw',
      '--el-dialog-margin-top': '10vh'
    }"
  >
    <div v-if="isFileBrowserVisible" style="height: 75vh">
      <FileBrowser
        :initial-path="browserInitialPath"
        :backend-id="props.currentBackendId"
        :allowed-extensions="['.launch', '.launch.xml']"
        @file-selected="handleFileSelected"
        @cancel="handleCancel"
      />
    </div>
    <el-empty v-else description="主机连接未建立。" />
  </el-dialog>
</template>

<script setup>
// #region 1. Imports
import FileBrowser from './FileBrowser.vue'
import { ref, reactive, computed } from 'vue'
import { useRobotStore } from '../store/robot'
import { ElMessage, ElMessageBox } from 'element-plus'
import { Edit, Delete } from '@element-plus/icons-vue'
// #endregion

// #region 2. State & Store Connection
const props = defineProps({
  currentBackendId: String,
  isCurrentBackendReady: Boolean
})

const robotStore = useRobotStore()
// 从 Store 中获取当前机器人的节点列表
const nodes = computed(() => {
  const client = robotStore.clients[props.currentBackendId]
  return client ? client.nodes : []
})

// UI 状态
const isFileBrowserVisible = ref(false)
const browserInitialPath = ref('')
const editingNodeId = ref(null) // 使用 ID 判断编辑状态，新增用 '__new__'

// 表单数据
const form = reactive({
  id: '',
  name: '',
  cmd: 'roslaunch',
  args: [] // Agent 格式: ["/path/to/file.launch"]
})

// 辅助显示：从 args 数组提取显示的路径字符串
const getDisplayPath = (nodeArgs) => {
  if (Array.isArray(nodeArgs) && nodeArgs.length > 0) {
    return nodeArgs[0] // 默认取第一个参数作为路径显示
  }
  return ''
}
// #endregion

// #region 3. Node Actions (Start/Stop)
async function toggleNode(node) {
  if (!props.isCurrentBackendReady) return

  try {
    if (node.status === 'running') {
      // 停止：传递 name (作为 process ID)
      await robotStore.stopNodeProcess(props.currentBackendId, node.name)
    } else {
      // 启动
      await robotStore.startNodeProcess(props.currentBackendId, node)
    }
  } catch (e) {
    ElMessage.error(`操作失败: ${e.message}`)
  }
}
// #endregion

// #region 4. CRUD Operations (Edit/Save/Delete)

// 开始添加
function startAdding() {
  editingNodeId.value = '__new__'
  form.id = ''
  form.name = ''
  form.cmd = 'roslaunch'
  form.args = []
}

// 开始编辑
function startEditing(node) {
  editingNodeId.value = node.id
  form.id = node.id
  form.name = node.name
  form.cmd = node.cmd
  // 确保 args 是数组
  form.args = Array.isArray(node.args) ? [...node.args] : [node.args]
}

// 取消
function cancelEditing() {
  editingNodeId.value = null
}

// 保存 (新增或更新)
async function saveNode() {
  const pathStr = getDisplayPath(form.args)
  if (!form.name || !pathStr) {
    ElMessage.error('节点名称和启动文件路径不能为空。')
    return
  }

  // 查重
  const isNameDuplicate = nodes.value.some((n) => n.name === form.name && n.id !== form.id)
  if (isNameDuplicate) {
    ElMessage.error('节点名称已存在，请使用唯一的名称。')
    return
  }
  const isDuplicatePath = nodes.value.some(
    (n) => getDisplayPath(n.args) === pathStr && n.id !== form.id
  )
  if (isDuplicatePath) {
    ElMessage.error('该 Launch 文件已被其他节点使用。')
    return
  }

  // 构造数据
  const nodeData = {
    id: form.id || `node-${Date.now()}`, // 如果是新节点，生成临时 ID
    name: form.name,
    cmd: form.cmd,
    args: [pathStr], // 简单处理：目前只支持单个路径参数
    description: ''
  }

  try {
    await robotStore.saveNodeConfig(props.currentBackendId, nodeData)
    // 保存成功后，显式重置表单并关闭编辑模式
    // 这样可以防止下一次点击 "Add" 时带入旧数据
    form.id = ''
    form.name = ''
    form.args = []
    cancelEditing()
    // eslint-disable-next-line no-unused-vars
  } catch (e) {
    // 错误已在 Store 中处理，这里不做额外操作
  }
}

// 删除
async function deleteNode(node) {
  try {
    await ElMessageBox.confirm(`确定要删除节点 "${node.name}" 吗？`, '删除确认', {
      confirmButtonText: '删除',
      cancelButtonText: '取消',
      type: 'warning'
    })

    await robotStore.deleteNodeConfig(props.currentBackendId, node.id)
    ElMessage.success('节点已删除')
  } catch (e) {
    if (e !== 'cancel') ElMessage.error('删除失败')
  }
}
// #endregion

// #region 5. File Browser Logic
function getPosixDirname(filePath) {
  if (!filePath) return null
  const lastSlashIndex = filePath.lastIndexOf('/')
  if (lastSlashIndex === -1) return null
  if (lastSlashIndex === 0) return '/'
  return filePath.substring(0, lastSlashIndex)
}

function openFileBrowser() {
  const currentPath = getDisplayPath(form.args)
  if (currentPath) {
    browserInitialPath.value = getPosixDirname(currentPath)
  } else {
    browserInitialPath.value = null
  }
  isFileBrowserVisible.value = true
}

function handleFileSelected(filePath) {
  if (filePath) {
    // 将选择的路径放入 args 数组的第一个位置
    form.args = [filePath]
  }
  isFileBrowserVisible.value = false
}

function handleCancel() {
  isFileBrowserVisible.value = false
}
// #endregion

// #region 6. UI Helpers
const statusType = (status) => {
  const map = {
    running: 'success',
    stopped: 'info',
    error: 'error',
    starting: 'warning'
  }
  return map[status] || 'info'
}
// #endregion
</script>

<style scoped>
.dashboard {
  height: 100%; /* 去掉padding高度*/
  padding: 0;
  display: flex;
  flex-grow: 1;
  flex-direction: row;
}
/* ------ 左侧卡片容器样式 ------ */
.node-list-panel {
  padding: 1% 1% 1% 1%;
  border: 2px dashed #e4e7ed;
  width: 50%;
  height: 100%;
}
.node-list-panel .el-card {
  display: flex;
  align-items: center;
  border: rgba(144, 147, 153, 1);
  border-radius: 0px 40px 40px 0px;
  height: 10%;
  background-image: 
    /* 上层：内容背景色 (通常是白色，或者用 Element 的变量 var(--el-bg-color)) */
    linear-gradient(var(--el-bg-color), var(--el-bg-color)),
    /* 下层：渐变边框色 (从左上到右下，各种蓝色) */
      linear-gradient(135deg, #409eff, #8cc5ff, #ecf5ff);
  /* 4. 背景裁剪方式 */
  background-origin: border-box;
  background-clip: padding-box, border-box;
  /* 5. (可选) 添加一点阴影让它更立体 */
  box-shadow: 0 2px 4px rgba(64, 158, 255, 0.15);
  transition: all 0.3s ease-out;
  margin-bottom: 10px;
}
.node-list-panel .el-card:hover {
  transform: scale(1.005); /* 保留之前的放大效果 */
}

/* 头部容器样式 */
.node-list-panel :deep(.el-card__header) {
  padding: 0px;
  display: flex;
  flex-direction: row;
  align-items: center;
  height: 80%;
  width: 25%; /* <-- 设定一个固定的宽度 */
  flex-shrink: 0; /* <-- 防止它在空间不足时被压缩 */
  border-right: 1px solid #e4e7ed; /* 在中间加一条分割线，更美观 */
  border-bottom: none; /* Element Plus 默认有下边框，去掉它 */
  transition: none;
}
/* 头部内容样式 */
.node-list-panel .el-card .card-header {
  display: flex;
  flex-direction: row;
  align-items: center;
  gap: 8px;
}
.node-list-panel .el-card .header-row {
  display: flex;
  justify-content: flex-start; /* 左右两端对齐 */
  align-items: center; /* 垂直居中对齐 */
  height: 100%;
  gap: 10px;
  position: relative;
}
.node-list-panel .el-card .header-row .node-name {
  font-size: 2.5vh;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.node-list-panel .el-card .header-row .el-input {
  height: 3vh;
  width: 90%;
  font-size: 2.5vh;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper) {
  box-shadow: none !important; /* 核心：去掉默认边框 */
  background-color: transparent; /* 去掉白色背景，使其更像纯文字 */
  padding: 0 !important; /* 核心：去掉左右内边距，让文字紧贴边缘 */
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper:hover),
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper.is-focus) {
  box-shadow: none !important; /* 确保交互时也不显示边框 */
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__inner) {
  border: none !important;
  padding: 0 !important; /* 确保文字真的紧贴左边 */
  height: auto; /* 可选：根据文字高度自适应，而不是默认的 32px */
  line-height: 1.5; /* 调整行高以匹配普通文本 */
}
.node-list-panel .el-card :deep(.card-header .el-tag) {
  border-radius: 0px;
  height: 100vh;
  width: 5px;
  padding: 0 0px;
  border: transparent;
  margin-right: 4px;
  transition: all 0.2s ease;
}
.node-list-panel .el-card .card-header:deep(.el-tag--success) {
  --el-tag-bg-color: rgb(41, 194, 66);
}
.node-list-panel .el-card .card-header:deep(.el-tag--danger) {
  --el-tag-bg-color: rgb(234, 104, 104);
}
.node-list-panel .el-card .card-header:deep(.el-tag--info) {
  --el-tag-bg-color: rgb(173, 177, 182);
}

/* 尾部容器样式 */
.node-list-panel .el-card :deep(.el-card__footer) {
  width: auto;
  padding-right: 15px;
  height: auto; /* 确保填满 footer 区域 */
  display: flex;
  flex-direction: row;
  justify-content: flex-end;
  align-items: center; /* 垂直居中 */
  border: none;
  transition: none;
}
.node-list-panel .el-card .card-footer .edit-icon-btn,
.node-list-panel .el-card .card-footer .delete-icon-btn {
  padding: 0px;
  margin-right: 0px;
  height: auto;
  width: auto;
  border: none;
}
.node-list-panel .el-card .card-footer .edit-icon-btn :deep(.el-icon) {
  font-size: 1.2em;
}
.node-list-panel .el-card .card-footer .delete-icon-btn :deep(.el-icon) {
  font-size: 1.2em;
  margin-bottom: 0.15em;
}
.node-list-panel .el-card .footer-display-actions {
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}
.node-list-panel .el-card .footer-edit-actions {
  display: flex;
  justify-content: space-between; /* 让两个按钮均匀分开 */
  align-items: center;
  width: 100%;
}
.node-list-panel .el-card .card-footer .start-stop-btn .start-stop-btn,
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--primary,
.node-list-panel .el-card .card-footer .start-stop-btn .footer-edit-actions .el-button {
  border-radius: 16px;
  transition: all 0.2s ease; /* 添加过渡效果 */
  height: 32px;
  width: 64px;
  font-size: 100%;
  margin-left: 10px;
}
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--primary,
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--primary:focus {
  border-color: #409eff;
  background-color: #409eff;
  color: #ffffff;
}
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--primary:hover {
  transform: scale(1.02);
  box-shadow: 1px 1px 1px rgba(0, 0, 0, 0.3);
}
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--danger,
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--danger:focus {
  border-color: #f56c6c;
  background-color: #f56c6c;
  color: #ffffff;
}
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--danger:hover {
  transform: scale(1.02);
  box-shadow: 1px 1px 1px rgba(0, 0, 0, 0.3);
}

/* 内容部容器样式 */
.node-list-panel .el-card :deep(.el-card__body) {
  font-size: 2vh;
  flex-grow: 1;
  width: 10%;
  padding: 10px 5px; /* 调整内边距 */
  transition: none;
}
.card-body-content {
  color: rgb(173, 177, 182);
  width: 100%;
  height: 100%;
  display: flex;
  justify-content: space-between; /* 让 launch path 和 edit icon 两端对齐 */
  align-items: center;
  word-break: break-all;
  margin-left: 0.5vw;
}
/* 4. launch 文件路径按钮的样式 */
.launch-path-button {
  font-size: 2vh;
  width: 100%;
  padding: 8px 8px 8px 8px;
  border-radius: 8px;
  justify-content: flex-start; /* 让按钮内文字左对齐 */
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis; /* 超长时显示省略号 */
  word-break: break-all;
}

/* ------ 右侧功能容器样式 ------ */
.el-main.function-panel {
  padding: 0;
  width: auto;
  height: 100%;
  margin-left: 0.5%;
}
.el-main.function-panel .auto-start-panel {
  width: auto;
  height: 60%;
  border: 2px dashed #e4e7ed;
  padding: 1% 1% 1% 1%;
  margin-bottom: 0.5%;
}
.el-main.function-panel .rosbag-panel {
  width: auto;
  height: auto;
  flex-grow: 1;
  border: 2px dashed #e4e7ed;
  padding: 1% 1% 1% 1%;
}
</style>
