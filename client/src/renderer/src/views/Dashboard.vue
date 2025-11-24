<!-- src/renderer/src/components/Dashboard.vue -->
<template>
  <el-container class="dashboard">
    <!-- 节点控制部分 -->
    <el-aside class="node-list-panel">
      <el-card
        v-for="node in nodes"
        :key="node.name"
        class="node-card"
        :class="{ 'is-editing': editingNodeName === node.name }"
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
                v-if="editingNodeName === node.name"
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
            v-if="editingNodeName === node.name"
            class="launch-path-button"
            @click="openFileBrowser"
          >
            {{ form.path || '选择启动文件' }}
          </el-button>
          <!-- 显示模式 -->
          <p v-else class="launch-path">{{ node.path }}</p>
        </div>

        <template #footer>
          <div class="card-footer">
            <!-- 编辑模式: 显示 Save/Cancel 按钮 -->
            <div v-if="editingNodeName === node.name" class="footer-edit-actions">
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
      <el-card v-if="editingNodeName === '__new__'" class="node-card is-editing">
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
            {{ form.path || '选择launch文件' }}
          </el-button>
        </div>
        <template #footer>
          <div class="card-footer">
            <div class="footer-edit-actions">
              <el-button type="primary" size="small" @click="saveNode">添加</el-button>
              <el-button size="small" @click="cancelEditing">取消</el-button>
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
import FileBrowser from './FileBrowser.vue'
import { ref, onMounted, reactive, watch, computed, onUnmounted } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { Edit, Delete } from '@element-plus/icons-vue'
import { useRobotStore } from '../store/robot'
import { useStorage } from '@vueuse/core'
import { v4 as uuidv4 } from 'uuid' // 如果没有装uuid，可用简易随机数替代

const props = defineProps({
  currentBackendId: { type: String, default: null }, // 即 IP
  isCurrentBackendReady: { type: Boolean, default: false }
})

// 状态管理
const robotStore = useRobotStore()
const nodes = ref([]) // UI 绑定的节点列表
// 本地缓存：Key 为 IP，Value 为节点列表。用于断线时回显。
const nodesCache = useStorage('dashboard-nodes-cache', {})

// 轮询定时器
let statusPoller = null

// UI 状态
const isFileBrowserVisible = ref(false)
const browserInitialPath = ref('')
const editingNodeName = ref(null)

// 表单数据
const form = reactive({
  id: '', // Agent 需要唯一 ID
  name: '',
  path: ''
})

// ----------------------------------------------------------------
// 1. 核心数据加载逻辑 (Load & Sync)
// ----------------------------------------------------------------

// 将 Agent 的数据格式转换为 UI 格式
// Agent: { id, name, args: ["/path/to.launch"], cmd: "roslaunch" }
// UI:    { id, name, path: "/path/to.launch", status: "stopped" }
const mapAgentToUi = (agentNodes) => {
  return agentNodes.map((an) => ({
    id: an.id || uuidv4(),
    name: an.name,
    path: an.args && an.args.length > 0 ? an.args[0] : '',
    status: 'stopped' // 默认状态，后续由 pollStatus 更新
  }))
}

const loadNodes = async () => {
  const ip = props.currentBackendId
  if (!ip) return

  // 步骤 A: 先从本地缓存加载 (防止白屏)
  if (nodesCache.value[ip]) {
    nodes.value = nodesCache.value[ip].map((n) => ({ ...n, status: 'stopped' }))
  } else {
    nodes.value = []
  }

  // 步骤 B: 如果已连接，从 Agent 拉取最新配置并更新缓存
  if (props.isCurrentBackendReady && robotStore.activeClient?.api) {
    try {
      const res = await robotStore.activeClient.api.get('/api/nodes')
      if (Array.isArray(res)) {
        // 转换数据
        const remoteNodes = mapAgentToUi(res)

        // 这里的策略是：以机器人端的配置为准 (Overwrite Local)
        // 你也可以实现复杂的 Merge 逻辑，但 Overwrite 最稳健
        nodes.value = remoteNodes

        // 更新本地缓存 (剥离 status 字段，只存配置)
        nodesCache.value[ip] = remoteNodes.map(({ status, ...rest }) => rest)

        // 立即刷新一次运行状态
        await pollStatus()
      }
    } catch (e) {
      console.warn('Failed to sync nodes from agent:', e)
      ElMessage.warning('无法同步节点配置，显示本地缓存')
    }
  }
}

// ----------------------------------------------------------------
// 2. 运行状态同步 (Process Status)
// ----------------------------------------------------------------

// 轮询当前运行的进程，更新 nodes 的 status
const pollStatus = async () => {
  if (!props.isCurrentBackendReady || !robotStore.activeClient?.api) return

  try {
    const res = await robotStore.activeClient.api.get('/proc/list')
    const processes = res.processes || []

    // 遍历 UI 上的节点，检查是否有对应的进程在运行
    // 匹配规则：进程的 ID 等于节点的 Name (我们启动时会这样设置)
    nodes.value.forEach((node) => {
      // 正在启动中的节点(starting)不要立刻被覆盖为 stopped
      if (node.status === 'starting') return

      const isRunning = processes.some((p) => p.id === node.name)
      node.status = isRunning ? 'running' : 'stopped'
    })
  } catch (e) {
    // 静默失败，不要在轮询中弹窗
    console.debug('Status poll failed', e)
  }
}

// 启动轮询
const startPoller = () => {
  stopPoller()
  // 1.5秒刷新一次状态
  statusPoller = setInterval(pollStatus, 1500)
}

const stopPoller = () => {
  if (statusPoller) {
    clearInterval(statusPoller)
    statusPoller = null
  }
}

// ----------------------------------------------------------------
// 3. 交互动作 (Action)
// ----------------------------------------------------------------

// 启动/停止节点
async function toggleNode(node) {
  if (!props.isCurrentBackendReady) return

  if (node.status === 'running') {
    // 停止: POST /proc/stop { id: node.name }
    try {
      node.status = 'stopping' // UI 乐观更新
      await robotStore.activeClient.api.post('/proc/stop', { id: node.name })
      // 等待下一次轮询更新状态，或者手动设为 stopped
      node.status = 'stopped'
    } catch (e) {
      ElMessage.error(`停止失败: ${e.message}`)
      node.status = 'running' // 回滚
    }
  } else {
    // 启动: POST /proc/start
    try {
      node.status = 'starting' // UI loading
      await robotStore.activeClient.api.post('/proc/start', {
        id: node.name, // 使用 Name 作为进程 ID，方便通过 Name 停止
        cmd: 'roslaunch',
        args: [node.path] // 假设 path 就是 launch 文件全路径
      })
      // 注意：启动需要时间，保持 starting 状态，交给轮询器去变更为 running
    } catch (e) {
      ElMessage.error(`启动失败: ${e.message}`)
      node.status = 'error'
    }
  }
}

// 保存节点 (新增/编辑) -> 同步给 Agent
async function saveNode() {
  if (!form.name || !form.path) {
    ElMessageBox.error('名称和路径不能为空')
    return
  }

  // 构造 Agent 需要的 payload
  // 如果是新增模式 (__new__)，ID 设为空，由后端或这里生成均可，这里生成保险
  const payload = {
    id: editingNodeName.value === '__new__' || !form.id ? `node-${Date.now()}` : form.id,
    name: form.name,
    cmd: 'roslaunch',
    args: [form.path],
    description: 'Created by ROS Desktop'
  }

  try {
    // 发送给 Agent 保存
    await robotStore.activeClient.api.post('/api/nodes', payload)
    ElMessage.success('配置已保存')

    // 重新加载列表 (最简单的同步方式)
    await loadNodes()
    cancelEditing()
  } catch (e) {
    ElMessage.error(`保存失败: ${e.message}`)
  }
}

// 删除节点 -> 同步给 Agent
async function deleteNode(node) {
  if (!node.id) return // 理论上应该都有 ID

  try {
    await ElMessageBox.confirm(`确定删除 "${node.name}"?`, '提示', { type: 'warning' })

    // 调用 DELETE 接口
    await robotStore.activeClient.api.delete('/api/nodes', { params: { id: node.id } })
    ElMessage.success('已删除')

    // 刷新
    await loadNodes()
  } catch (e) {
    if (e !== 'cancel') ElMessage.error(`删除失败: ${e.message}`)
  }
}

// ----------------------------------------------------------------
// 4. UI 辅助逻辑 (Keep mostly same)
// ----------------------------------------------------------------

function getPosixDirname(filePath) {
  if (!filePath) return null
  const lastSlashIndex = filePath.lastIndexOf('/')
  if (lastSlashIndex === -1) return null
  if (lastSlashIndex === 0) return '/'
  return filePath.substring(0, lastSlashIndex)
}

function startEditing(node) {
  editingNodeName.value = node.name
  form.id = node.id
  form.name = node.name
  form.path = node.path
}

function startAdding() {
  if (!props.isCurrentBackendReady) {
    ElMessage.warning('请先连接机器人')
    return
  }
  editingNodeName.value = '__new__'
  form.id = ''
  form.name = ''
  form.path = ''
}

function cancelEditing() {
  editingNodeName.value = null
}

function openFileBrowser() {
  if (form.path) {
    browserInitialPath.value = getPosixDirname(form.path)
  } else {
    browserInitialPath.value = null
  }
  isFileBrowserVisible.value = true
}

function handleFileSelected(filePath) {
  if (filePath) form.path = filePath
  isFileBrowserVisible.value = false
}

function handleCancel() {
  isFileBrowserVisible.value = false
}

// ----------------------------------------------------------------
// 5. 生命周期与监听
// ----------------------------------------------------------------

// 监听 IP 变化
watch(
  () => props.currentBackendId,
  async (newVal) => {
    stopPoller()
    if (newVal) {
      await loadNodes() // 加载新机器的数据
      if (props.isCurrentBackendReady) startPoller()
    } else {
      nodes.value = []
    }
  },
  { immediate: true }
)

// 监听连接状态变化 (例如断线重连)
watch(
  () => props.isCurrentBackendReady,
  (isReady) => {
    if (isReady) {
      loadNodes() // 连上后立即同步一次
      startPoller()
    } else {
      stopPoller()
      // 断线后，将所有节点状态置为 stopped (灰色)，表明无法控制
      nodes.value.forEach((n) => (n.status = 'stopped'))
    }
  }
)

onUnmounted(() => {
  stopPoller()
})

// 状态样式映射
const statusType = (status) => {
  const map = {
    running: 'success',
    stopped: 'info',
    starting: 'warning',
    stopping: 'warning',
    error: 'error'
  }
  return map[status] || 'info'
}
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
