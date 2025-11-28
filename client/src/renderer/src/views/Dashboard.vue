<!-- src/renderer/src/components/Dashboard.vue -->
<template>
  <el-container class="dashboard">
    <!-- 节点控制部分 -->
    <el-aside class="node-list-panel">
      <div
        v-for="node in nodes"
        :key="node.id || node.name"
        class="node-wrapper"
        @mouseenter="node.showEdit = true"
        @mouseleave="node.showEdit = false"
      >
        <el-card
          class="node-card"
          :class="{ 'is-editing': editingNodeId === node.id }"
          @click="toggleExpand(node)"
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
                  @click.stop
                />
                <!-- 显示模式的标题 -->
                <span v-else class="node-name">{{ node.name }}</span>
              </div>
              <el-icon class="expand-arrow" :class="{ 'is-active': node.isExpanded }">
                <ArrowRight />
              </el-icon>
            </div>
          </template>

          <!-- 卡片内容部分 -->
          <div class="card-body-content">
            <!-- 编辑模式 -->
            <div v-if="editingNodeId === node.id" class="launch-edit-row">
              <el-button class="launch-path-button" @click.stop="openFileBrowser('launch')">
                {{ getDisplayPath(form.args) || '选择启动文件' }}
              </el-button>
              <!-- 编辑 Launch 内容按钮 -->
              <el-tooltip content="编辑 Launch 文件" placement="top">
                <el-button
                  :icon="EditPen"
                  type="primary"
                  plain
                  class="launch-content-edit-btn"
                  :disabled="!getDisplayPath(form.args)"
                  @click.stop="openLaunchEditor"
                />
              </el-tooltip>
            </div>
            <!-- 显示模式 -->
            <p v-else class="launch-path">{{ getDisplayPath(node.args) }}</p>
          </div>

          <template #footer>
            <div class="card-footer">
              <!-- 编辑模式: 显示 Save/Cancel 按钮 -->
              <div v-if="editingNodeId === node.id" class="footer-edit-actions">
                <el-button type="primary" size="small" round @click.stop="saveNode">保存</el-button>
                <el-button size="small" round @click.stop="cancelEditing">取消</el-button>
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
                  :disabled="!isCurrentBackendReady || node.status !== 'stopped'"
                  @click.stop="deleteNode(node)"
                />
                <!-- 编辑图标 -->
                <el-button
                  v-show="node.showEdit"
                  class="edit-icon-btn"
                  :icon="Edit"
                  :disabled="!isCurrentBackendReady || node.status !== 'stopped'"
                  type="primary"
                  circle
                  link
                  @click.stop="startEditing(node)"
                />
                <el-button
                  class="start-stop-btn"
                  :type="
                    node.status === 'starting'
                      ? 'primary'
                      : node.status === 'stopping'
                        ? 'danger'
                        : node.status === 'running'
                          ? 'danger'
                          : 'primary'
                  "
                  :loading="node.status === 'starting' || node.status === 'stopping'"
                  :disabled="
                    node.status === 'starting' ||
                    node.status === 'stopping' ||
                    !isCurrentBackendReady
                  "
                  round
                  @click.stop="toggleNode(node)"
                >
                  {{
                    node.status === 'starting'
                      ? '启动'
                      : node.status === 'stopping'
                        ? '停止'
                        : node.status === 'running'
                          ? '停止'
                          : '启动'
                  }}
                </el-button>
              </div>
            </div>
          </template>
        </el-card>

        <!-- 2. 参数列表面板 (独立于 Card) -->
        <el-collapse-transition>
          <div v-show="node.isExpanded && editingNodeId !== node.id" class="params-drawer">
            <NodeParamsList
              :node="node"
              :backend-id="props.currentBackendId"
              @update-node="handleNodeUpdate"
              @pick-file="(param) => openFileBrowser('param', param)"
              @click.stop
            />
          </div>
        </el-collapse-transition>
      </div>

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
          <el-button class="launch-path-button" @click="openFileBrowser('launch')">
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
      <el-card
        v-else
        class="node-card add-new-card"
        :class="{ 'is-disabled': !isCurrentBackendReady }"
        @click="startAdding"
      >
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
        <div class="panel-header">
          <h2>一键启动序列</h2>
          <el-button
            type="primary"
            link
            :icon="CirclePlus"
            :disabled="!isCurrentBackendReady"
            @click="openSeqDialog(null)"
          >
            新建序列
          </el-button>
        </div>

        <el-scrollbar height="calc(100% - 40px)">
          <div v-if="sequences.length === 0" class="empty-hint">暂无序列</div>

          <div v-for="seq in sequences" :key="seq.id" class="seq-item">
            <div class="seq-info">
              <div class="seq-name">{{ seq.name }}</div>
              <div class="seq-desc">{{ seq.steps.length }} 个步骤</div>
            </div>
            <div class="seq-actions">
              <el-button
                circle
                size="small"
                :type="seq._status === 'running' ? 'danger' : 'success'"
                :icon="seq._status === 'running' ? VideoPause : VideoPlay"
                :loading="seq._status === 'running'"
                :disabled="!isCurrentBackendReady"
                @click="toggleSequence(seq)"
              />
              <el-button circle size="small" :icon="Edit" @click="openSeqDialog(seq)" />
              <el-button
                circle
                size="small"
                :icon="Delete"
                type="danger"
                plain
                @click="deleteSequence(seq)"
              />
            </div>
          </div>
        </el-scrollbar>
      </div>

      <div class="rosbag-panel">
        <div class="panel-header">
          <h2>ROSbag 录制</h2>
          <el-tag v-if="isRecording" type="danger" effect="dark" class="recording-tag">
            <span class="dot-blink">●</span> REC
          </el-tag>
        </div>

        <!-- [修复 Bug 2] 移除 el-form 上的 disabled 属性 -->
        <el-form label-position="top" size="small">
          <el-row :gutter="10">
            <el-col :span="16">
              <el-form-item label="保存路径">
                <!-- [修改] 只有在录制时禁用输入框 -->
                <el-input
                  v-model="bagForm.path"
                  readonly
                  :disabled="isRecording"
                  @click="!isRecording && openFileBrowser('bag-path')"
                >
                  <template #append>
                    <el-button :disabled="isRecording" @click="openFileBrowser('bag-path')"
                      >选择</el-button
                    >
                  </template>
                </el-input>
              </el-form-item>
            </el-col>
            <el-col :span="8">
              <el-form-item label="文件名前缀">
                <el-input v-model="bagForm.name" placeholder="record" :disabled="isRecording" />
              </el-form-item>
            </el-col>
          </el-row>

          <el-form-item label="话题选择 (留空则录制所有)">
            <el-select
              v-model="bagForm.selectedTopics"
              multiple
              collapse-tags
              placeholder="All Topics (-a)"
              style="width: 100%"
              filterable
              :disabled="isRecording"
            >
              <el-option
                v-for="t in availableTopics"
                :key="t.topic"
                :label="t.topic"
                :value="t.topic"
              >
                <span style="float: left">{{ t.topic }}</span>
                <span style="float: right; color: #8492a6; font-size: 12px">{{
                  t.schemaName
                }}</span>
              </el-option>
            </el-select>
          </el-form-item>

          <el-row :gutter="10">
            <el-col :span="8">
              <el-form-item label="自动分割">
                <el-switch v-model="bagForm.split" :disabled="isRecording" />
              </el-form-item>
            </el-col>
            <el-col v-if="bagForm.split" :span="16">
              <el-form-item label="分割大小 (MB)">
                <el-input-number
                  v-model="bagForm.size"
                  :min="100"
                  :step="100"
                  :disabled="isRecording"
                />
              </el-form-item>
            </el-col>
          </el-row>

          <!-- [修复 Bug 1 & 2] 按钮状态逻辑 -->
          <el-button
            class="record-btn"
            :type="isRecording ? 'danger' : 'primary'"
            :icon="isRecording ? VideoPause : VideoPlay"
            :loading="false"
            :disabled="!isCurrentBackendReady || (!isRecording && !isRosServiceReady)"
            @click="toggleRecording"
          >
            <!-- 
              disabled 逻辑解释:
              1. 后端未连接 -> 禁用
              2. 或者 (没在录制 且 ROS服务没好) -> 禁用 (防止未启动服务就开始)
              3. 如果正在录制 (isRecording=true)，则允许点击 (因为要停止)
            -->
            {{ isRecording ? '停止录制' : isRosServiceReady ? '开始录制' : 'ROS 服务未就绪' }}
          </el-button>
        </el-form>
      </div>
    </el-main>
  </el-container>
  <el-dialog
    v-model="fileBrowserContext.visible"
    class="file-browser-dialog"
    destroy-on-close
    :close-on-click-modal="false"
    :show-close="false"
    draggable
  >
    <div style="height: 100%; overflow: hidden">
      <FileBrowser
        :initial-path="fileBrowserContext.initialPath"
        :backend-id="props.currentBackendId"
        :allowed-extensions="fileBrowserContext.allowedExtensions"
        :show-close="true"
        :hide-footer="false"
        :target-type="fileBrowserContext.targetType"
        @file-selected="handleFileSelected"
        @cancel="handleCancel"
        @close="fileBrowserContext.visible = false"
      />
    </div>
  </el-dialog>
  <el-dialog
    v-model="isSeqDialogVisible"
    title="编辑启动序列"
    width="500px"
    :close-on-click-modal="false"
    append-to-body
  >
    <el-form label-position="top">
      <el-form-item label="序列名称">
        <el-input v-model="seqForm.name" placeholder="例如: 全系统启动" />
      </el-form-item>

      <el-form-item label="执行流程">
        <div class="timeline-editor">
          <el-timeline>
            <el-timeline-item
              v-for="(step, index) in seqForm.steps"
              :key="index"
              :type="step.type === 'node' ? 'primary' : 'warning'"
              :icon="step.type === 'delay' ? Timer : undefined"
              hide-timestamp
            >
              <div class="step-content">
                <!-- 节点选择 -->
                <el-select
                  v-if="step.type === 'node'"
                  v-model="step.content"
                  placeholder="选择节点"
                  size="small"
                  style="width: 200px"
                >
                  <el-option v-for="n in nodes" :key="n.id" :label="n.name" :value="n.id" />
                </el-select>

                <!-- 延时输入 -->
                <div v-else class="delay-input">
                  <span>等待</span>
                  <el-input-number
                    v-model="step.content"
                    size="small"
                    :min="100"
                    :step="500"
                    controls-position="right"
                    style="width: 100px"
                  />
                  <span>ms</span>
                </div>

                <!-- 删除步骤 -->
                <el-button link type="danger" :icon="Remove" @click="removeSeqStep(index)" />
              </div>
            </el-timeline-item>

            <!-- 底部添加按钮 -->
            <el-timeline-item class="add-step-item">
              <el-button-group size="small">
                <el-button :icon="Plus" @click="addSeqStep('node')">节点</el-button>
                <el-button :icon="Timer" @click="addSeqStep('delay')">延时</el-button>
              </el-button-group>
            </el-timeline-item>
          </el-timeline>
        </div>
      </el-form-item>
    </el-form>

    <template #footer>
      <el-button @click="isSeqDialogVisible = false">取消</el-button>
      <el-button type="primary" @click="saveSequence">保存</el-button>
    </template>
  </el-dialog>
</template>

<script setup>
// #region 1. Imports
import FileBrowser from '../components/FileBrowser.vue'
import NodeParamsList from '../components/NodeParamsList.vue' // [新增] 引入组件
import { ref, reactive, computed, watch, onMounted } from 'vue'
import { useRobotStore } from '../store/robot.js'
import { useFoxglove } from '../composables/useFoxglove.js'
import { ElMessage, ElMessageBox } from 'element-plus'
import {
  Edit,
  Delete,
  EditPen,
  VideoPlay,
  VideoPause,
  Timer,
  CirclePlus,
  Remove,
  Plus,
  ArrowRight
} from '@element-plus/icons-vue'
// #endregion

// #region 2. State & Store Connection
const props = defineProps({
  currentBackendId: String,
  isCurrentBackendReady: Boolean
})

const robotStore = useRobotStore()
const { getTopicsRef } = useFoxglove()
// 从 Store 中获取当前机器人的节点列表
const nodes = computed(() => {
  const client = robotStore.clients[props.currentBackendId]
  return client ? client.nodes : []
})

// UI 状态
const fileBrowserContext = reactive({
  visible: false,
  initialPath: '',
  allowedExtensions: [],
  mode: 'launch', // 业务逻辑标识: 'launch' | 'bag-path'
  targetType: 'file', // 组件行为标识: 'file' | 'path'
  targetData: null // 存储临时的引用对象，例如 param 对象
})
const editingNodeId = ref(null) // 使用 ID 判断编辑状态，新增用 '__new__'

// --- 序列管理状态 ---
const sequences = computed(() => {
  const client = robotStore.clients[props.currentBackendId]
  return client ? client.sequences || [] : []
})

const isSeqDialogVisible = ref(false)
const seqForm = reactive({
  id: '',
  name: '',
  steps: [] // { type: 'node'|'delay', content: 'node-id'|'1000' }
})

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
  if (node.status === 'running') {
    // Store 内部会设为 stopping
    await robotStore.stopNodeProcess(props.currentBackendId, node.name)
  } else {
    // Store 内部会设为 starting 并检查 ROS 服务
    await robotStore.startNodeProcess(props.currentBackendId, node)
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

// 切换展开状态
function toggleExpand(node) {
  // 如果正在编辑节点本身(改名/换launch)，不响应展开，避免逻辑冲突
  if (editingNodeId.value === node.id) return
  node.isExpanded = !node.isExpanded
}

// --- [新增] 序列操作 ---
function openSeqDialog(seq = null) {
  if (seq) {
    // 编辑模式
    seqForm.id = seq.id
    seqForm.name = seq.name
    // 深拷贝步骤，防止修改时直接影响 UI
    seqForm.steps = JSON.parse(JSON.stringify(seq.steps))
  } else {
    // 新增模式
    seqForm.id = ''
    seqForm.name = ''
    seqForm.steps = []
  }
  isSeqDialogVisible.value = true
}

function addSeqStep(type) {
  if (type === 'node') {
    // 默认选中第一个节点(如果有)
    const firstNodeId = nodes.value.length > 0 ? nodes.value[0].id : ''
    seqForm.steps.push({ type: 'node', content: firstNodeId })
  } else {
    seqForm.steps.push({ type: 'delay', content: '1000' })
  }
}

function removeSeqStep(index) {
  seqForm.steps.splice(index, 1)
}

async function saveSequence() {
  if (!seqForm.name) {
    ElMessage.warning('请输入序列名称')
    return
  }
  // 校验步骤完整性
  for (const step of seqForm.steps) {
    if (step.type === 'node' && !step.content) {
      ElMessage.warning('请为所有节点步骤选择具体的节点')
      return
    }
  }

  const data = {
    id: seqForm.id || `seq-${Date.now()}`,
    name: seqForm.name,
    steps: seqForm.steps,
    _status: 'stopped' // 运行时状态，不持久化也没关系，但在 Store 里初始化一下
  }

  await robotStore.saveSequenceConfig(props.currentBackendId, data)
  isSeqDialogVisible.value = false
}

async function deleteSequence(seq) {
  try {
    await ElMessageBox.confirm(`确定删除序列 "${seq.name}" 吗？`, '提示', { type: 'warning' })
    await robotStore.deleteSequenceConfig(props.currentBackendId, seq.id)
  } catch {
    /* */
  }
}

async function toggleSequence(seq) {
  if (seq._status === 'running') {
    // 停止序列中的所有节点
    await robotStore.stopSequenceNodes(props.currentBackendId, seq)
  } else {
    // 开始执行
    await robotStore.runSequence(props.currentBackendId, seq)
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

// 打开文件浏览器 (通用入口)
// type: 'launch' | 'param'
// data: 如果是 param，传入 param 对象引用
function openFileBrowser(mode, data = null) {
  // 1. 设置基础上下文
  fileBrowserContext.mode = mode
  fileBrowserContext.targetData = data

  let currentPathStr = ''

  // 2. 根据模式配置行为
  switch (mode) {
    case 'launch':
      fileBrowserContext.targetType = 'file'
      fileBrowserContext.allowedExtensions = ['.launch', '.launch.xml']
      // 从表单获取当前路径
      currentPathStr = getDisplayPath(form.args)
      break

    case 'param':
      fileBrowserContext.targetType = 'file'
      fileBrowserContext.allowedExtensions = [] // 不限制，或者是 .yaml
      currentPathStr = data ? data.path : ''
      break

    case 'bag-path': // [新增] 录制保存路径
      fileBrowserContext.targetType = 'path' // 关键：告诉 FileBrowser 只能选文件夹
      fileBrowserContext.allowedExtensions = []
      currentPathStr = bagForm.path
      break

    default:
      console.warn('Unknown file browser mode:', mode)
      fileBrowserContext.targetType = 'file'
      fileBrowserContext.allowedExtensions = []
  }

  // 3. 计算初始路径
  // 如果是选文件模式，我们通常希望打开文件所在的【父目录】
  // 如果是选目录模式，我们通常希望直接进入【该目录】
  if (currentPathStr) {
    if (fileBrowserContext.targetType === 'file') {
      fileBrowserContext.initialPath = getPosixDirname(currentPathStr)
    } else {
      fileBrowserContext.initialPath = currentPathStr
    }
  } else {
    fileBrowserContext.initialPath = null // 让组件自动加载 Home
  }

  // 4. 显示弹窗
  fileBrowserContext.visible = true
}

// 处理文件选择 (根据上下文分发)
function handleFileSelected(filePath) {
  if (!filePath) return

  switch (fileBrowserContext.mode) {
    case 'launch':
      // 修改节点 Launch 文件
      form.args = [filePath]
      break

    case 'param': {
      // 修改参数文件
      const param = fileBrowserContext.targetData
      if (param) {
        param.path = filePath
        // 尝试找到父节点并触发自动保存 (根据你的业务逻辑)
        const parentNode = nodes.value.find((n) => n.params && n.params.includes(param))
        if (parentNode && typeof handleNodeUpdate === 'function') {
          handleNodeUpdate(parentNode)
        }
      }
      break
    }

    case 'bag-path':
      // [新增] 修改 Rosbag 保存路径
      bagForm.path = filePath
      break
  }

  fileBrowserContext.visible = false
}

// 处理节点更新 (来自 NodeParamsList)
async function handleNodeUpdate(updatedNode) {
  // 调用 Store 保存
  await robotStore.saveNodeConfig(props.currentBackendId, updatedNode)
}

// 直接打开编辑器 (Feature 4)
async function openLaunchEditor() {
  const path = getDisplayPath(form.args)
  if (!path) return

  try {
    const fileName = path.split('/').pop()
    await window.api.openFileEditor({
      name: fileName,
      path: path,
      backendId: props.currentBackendId,
      backendLabel: props.currentBackendId
    })
  } catch (e) {
    ElMessage.error(e.message)
  }
}

function handleCancel() {
  fileBrowserContext.visible = false
}
// #endregion

// [新增] 录制状态与表单
const bagForm = reactive({
  name: 'record',
  path: '',
  selectedTopics: [], // 空数组代表所有
  split: false,
  size: 1024
})

// [新增] 计算属性：检查 ROS 服务是否就绪
const isRosServiceReady = computed(() => {
  const client = robotStore.clients[props.currentBackendId]
  // 至少 roscore 要在运行，bridge 其实不影响录制（那是 agent 本地的事），但为了数据流一致性，通常建议全栈就绪
  return client?.serviceStatus?.roscore === 'active'
})

// [新增] 获取当前机器的话题列表 (响应式)
const availableTopics = computed(() => {
  if (!props.currentBackendId) return []
  // getTopicsRef 返回的是 ref，需要 .value
  // 每个 topic 对象包含 { topic: '/scan', schemaName: '...', isAlive: true }
  return getTopicsRef(props.currentBackendId).value
})

// [新增] 获取当前录制状态 (从 Store)
const isRecording = computed(() => {
  const client = robotStore.clients[props.currentBackendId]
  return client && !!client.recordingId
})

// [新增] 开始/停止录制
async function toggleRecording() {
  if (!props.isCurrentBackendReady) return
  const client = robotStore.clients[props.currentBackendId]

  if (isRecording.value) {
    // 停止
    try {
      await robotStore.bagStop(props.currentBackendId, client.recordingId)
      ElMessage.success('录制停止指令已发送')
    } catch (e) {
      ElMessage.error('停止失败: ' + e.message)
    }
  } else {
    // [修复 Bug 1] 校验 ROS 服务状态
    if (!isRosServiceReady.value) {
      ElMessage.warning('ROS 服务未启动，无法录制数据。请先启动 ROS 服务。')
      return
    }

    // [修复 Bug 1] 校验话题列表 (可选：如果没有话题，提示一下，但不一定强制阻断，因为话题可能随时出现)
    if (availableTopics.value.length === 0) {
      try {
        await ElMessageBox.confirm(
          '当前没有检测到任何活跃话题，录制可能会生成空文件。确定要继续吗？',
          '无话题警告',
          {
            confirmButtonText: '强制录制',
            cancelButtonText: '取消',
            type: 'warning'
          }
        )
      } catch {
        return // 用户取消
      }
    }
    // 开始
    if (!bagForm.path) {
      // 如果没选路径，尝试获取 Home
      try {
        const sidebar = await robotStore.fsGetSidebar(props.currentBackendId)
        const home = sidebar.places.find((p) => p.icon === 'House')?.path || '/'
        bagForm.path = home
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        bagForm.path = '/'
      }
    }

    try {
      await robotStore.bagStart(props.currentBackendId, {
        path: bagForm.path,
        name: bagForm.name || 'record',
        topics: bagForm.selectedTopics, // API: 空数组 = 所有
        split: bagForm.split,
        size: bagForm.size
      })
      ElMessage.success('开始录制')
    } catch (e) {
      ElMessage.error('启动失败: ' + e.message)
    }
  }
}

// #region 6. UI Helpers
const statusType = (status) => {
  const map = {
    running: 'success',
    stopped: 'info',
    crashed: 'error',
    error: 'error',
    starting: 'warning',
    stopping: 'warning' // [新增] 停止中也显示黄色
  }
  return map[status] || 'info'
}
// #endregion

onMounted(async () => {
  if (props.currentBackendId) {
    console.log('Dashboard mounted, loading nodes from cache...')
    await robotStore.loadNodesFromCache(props.currentBackendId)
  }
})

watch(
  () => props.currentBackendId,
  async (newId) => {
    if (newId) {
      await robotStore.loadNodesFromCache(newId)
    }
  }
)
</script>

<style scoped>
/* ============================================
   1. 全局布局 (Global Layout)
   ============================================ */
.dashboard {
  height: 100%;
  padding: 0;
  display: flex;
  flex-grow: 1;
  flex-direction: row;
}

/* ============================================
   2. 左侧：节点控制面板 (Left Aside: Node List)
   ============================================ */

/* --- 2.1 容器与列表项 --- */
.node-list-panel {
  padding: 1% 1% 1% 1%;
  border: 2px dashed #e4e7ed;
  width: 50%;
  height: 100%;
}

.node-wrapper {
  margin-bottom: 15px;
  display: flex;
  flex-direction: column;
}

.params-drawer {
  z-index: 1;
  margin-top: -5px;
}

/* --- 2.2 节点卡片基础样式 (Card Basic) --- */
.node-card {
  display: flex;
  align-items: center;
  border: rgba(144, 147, 153, 1);
  border-radius: 0px 40px 40px 0px;
  height: 60px;
  background-image:
    linear-gradient(var(--el-bg-color), var(--el-bg-color)),
    linear-gradient(135deg, #409eff, #8cc5ff, #ecf5ff);
  background-origin: border-box;
  background-clip: padding-box, border-box;
  box-shadow: 0 2px 4px rgba(64, 158, 255, 0.15);
  transition: all 0.3s ease-out;
  z-index: 2;
}

.node-card:hover {
  transform: scale(1.005);
  margin-bottom: 0;
}

.node-card.has-params-open {
  border-bottom-left-radius: 0;
  border-bottom-right-radius: 0;
  border-bottom-color: transparent;
  box-shadow: 0 4px 12px 0 rgba(0, 0, 0, 0.1);
}

.add-new-card.is-disabled {
  cursor: not-allowed;
  background-color: #f5f7fa;
  border-color: #e4e7ed;
}

.clickable-card {
  cursor: pointer;
}

.clickable-card.is-editing {
  cursor: default;
}

.expand-arrow {
  margin-right: 8px;
  color: #909399;
  transition: transform 0.3s;
  font-size: 12px;
}

.expand-arrow.is-active {
  transform: rotate(90deg);
}

/* --- 2.3 卡片头部 (Card Header) --- */
/* 头部容器 */
.node-list-panel :deep(.el-card__header) {
  padding: 0px;
  display: flex;
  flex-direction: row;
  align-items: center;
  height: 80%;
  width: 25%;
  flex-shrink: 0;
  border-right: 1px solid #e4e7ed;
  border-bottom: none;
  transition: none;
}

/* 头部内容布局 */
.node-list-panel .el-card .card-header {
  display: flex;
  flex-direction: row;
  align-items: center;
  gap: 8px;
}

.node-list-panel .el-card .header-row {
  display: flex;
  justify-content: flex-start;
  align-items: center;
  height: 100%;
  gap: 10px;
  position: relative;
}

/* 节点名称文本 */
.node-list-panel .el-card .header-row .node-name {
  font-size: 20px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

/* 节点名称输入框 (编辑模式) */
.node-list-panel .el-card .header-row .el-input {
  width: 90%;
  font-size: 20px;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper) {
  box-shadow: none !important;
  background-color: transparent;
  padding: 0 !important;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper:hover),
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper.is-focus) {
  box-shadow: none !important;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__inner) {
  border: none !important;
  padding: 0 !important;
  height: auto;
  line-height: 1.5;
}

/* 状态标签 (Tags) */
.node-list-panel .el-card :deep(.card-header .el-tag) {
  border-radius: 0px;
  height: 100vh;
  width: 5px;
  padding: 0 0px;
  border: transparent;
  margin-right: 4px;
  transition: all 0.2s ease;
}
/* Tag 颜色定义 */
.node-list-panel .el-card .card-header:deep(.el-tag--success) {
  --el-tag-bg-color: rgb(41, 194, 66);
}
.node-list-panel .el-card .card-header:deep(.el-tag--danger) {
  --el-tag-bg-color: rgb(234, 104, 104);
}
.node-list-panel .el-card .card-header:deep(.el-tag--warning) {
  --el-tag-bg-color: rgb(230, 193, 45);
}
.node-list-panel .el-card .card-header:deep(.el-tag--info) {
  --el-tag-bg-color: rgb(173, 177, 182);
}

/* --- 2.4 卡片内容主体 (Card Body) --- */
.node-list-panel .el-card :deep(.el-card__body) {
  font-size: 2vh;
  flex-grow: 1;
  width: 10%;
  padding: 10px 5px;
  transition: none;
}

.card-body-content {
  color: rgb(173, 177, 182);
  width: 100%;
  height: 100%;
  display: flex;
  justify-content: space-between;
  align-items: center;
  word-break: break-all;
  margin-left: 0.5vw;
}

/* Launch文件路径及按钮 */
.launch-path-button {
  font-size: 2vh;
  width: 100%;
  padding: 8px 8px 8px 8px;
  border-radius: 8px;
  justify-content: flex-start;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  word-break: break-all;
}
.launch-edit-row {
  display: flex;
  gap: 5px;
  width: 100%;
}
.launch-path-button {
  flex-grow: 1;
}
.launch-content-edit-btn {
  flex-shrink: 0;
  width: 32px;
  padding: 0;
}

/* --- 2.5 卡片底部操作区 (Card Footer) --- */
.node-list-panel .el-card :deep(.el-card__footer) {
  width: auto;
  padding-right: 15px;
  height: auto;
  display: flex;
  flex-direction: row;
  justify-content: flex-end;
  align-items: center;
  border: none;
  transition: none;
}

/* 图标按钮 (编辑/删除) */
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

/* 按钮组布局 */
.node-list-panel .el-card .footer-display-actions,
.node-list-panel .el-card .footer-edit-actions {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}

/* 启动/停止/保存按钮样式 */
.node-list-panel .el-card .card-footer .start-stop-btn .start-stop-btn,
.node-list-panel .el-card .card-footer .start-stop-btn .el-button--primary,
.node-list-panel .el-card .card-footer .start-stop-btn .footer-edit-actions .el-button,
.node-list-panel .el-card .footer-edit-actions .el-button {
  border-radius: 16px;
  transition: all 0.2s ease;
  height: 32px;
  width: 64px;
  font-size: 100%;
  margin-left: 10px;
}

/* Primary Button 交互 */
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

/* Danger Button 交互 */
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

/* ============================================
   3. 右侧：功能面板 (Right Main: Function Panel)
   ============================================ */

/* --- 3.1 容器 --- */
.el-main.function-panel {
  padding: 0;
  width: auto;
  height: 100%;
  margin-left: 0.5%;
}

/* --- 3.2 顶部通用标题栏 --- */
.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 15px;
}
.panel-header h2 {
  margin: 0;
  font-size: 16px;
  color: #303133;
}

/* --- 3.3 一键启动序列 (Auto Start) [Scoped 部分] --- */
.el-main.function-panel .auto-start-panel {
  width: auto;
  height: 60%;
  border: 2px dashed #e4e7ed;
  padding: 1% 1% 1% 1%;
  margin-bottom: 0.5%;
}

/* --- 3.4 Rosbag 录制 (Rosbag Panel) --- */
.el-main.function-panel .rosbag-panel {
  width: auto;
  height: auto;
  flex-grow: 1;
  border: 2px dashed #e4e7ed;
  padding: 1% 1% 1% 1%;
}
.rosbag-panel {
  background: white;
  padding: 15px;
  border-radius: 8px;
  box-shadow: 0 2px 12px 0 rgba(0, 0, 0, 0.05);
  margin-top: 15px;
}
.record-btn {
  width: 100%;
  margin-top: 10px;
  height: 40px;
  font-size: 16px;
}
.dot-blink {
  animation: blink 1s infinite;
  margin-right: 4px;
}
@keyframes blink {
  0% {
    opacity: 1;
  }
  50% {
    opacity: 0;
  }
  100% {
    opacity: 1;
  }
}
</style>

<style>
/* ============================================
   4. 全局弹窗与覆盖层 (Dialogs & Overlays)
   ============================================ */

/* --- 4.1 文件浏览器弹窗 (File Browser) --- */
.el-overlay-dialog:has(.file-browser-dialog) {
  overflow: hidden !important;
  display: flex;
  align-items: center;
  justify-content: center;
}

.file-browser-dialog {
  margin: 0 !important;
  padding: 12px;
  display: flex;
  flex-direction: column;
  height: 75vh;
  width: 75vw;
  border-radius: 8px;
  overflow: hidden;
}

.file-browser-dialog .el-dialog__header {
  display: none;
}

.file-browser-dialog .el-dialog__body {
  flex: 1;
  overflow: hidden;
  padding: 0 !important;
  border: 1px solid var(--el-border-color-light) !important;
  height: 100%;
}

/* ============================================
   5. 全局功能样式 (可能在 scoped 中有冲突)
   ============================================ */

/* --- 5.1 一键启动序列 (Auto Start) [Global 部分] --- */
/* 注意：这里与 scoped 中的 .auto-start-panel 定义共存，目前保持原样 */
.auto-start-panel {
  flex: 1;
  display: flex;
  flex-direction: column;
  background: #fff;
  border-radius: 8px;
  padding: 15px;
  margin-bottom: 20px;
  box-shadow: 0 2px 12px 0 rgba(0, 0, 0, 0.05);
}

/* 注意：Global panel-header 可能会影响全局 */
.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
}
.panel-header h2 {
  margin: 0;
  font-size: 16px;
  color: #303133;
}

/* 序列列表项 */
.seq-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
  border-bottom: 1px solid #f0f2f5;
  transition: background 0.2s;
}
.seq-item:last-child {
  border-bottom: none;
}
.seq-item:hover {
  background-color: #fafafa;
}

.seq-name {
  font-size: 14px;
  font-weight: 500;
  color: #606266;
}
.seq-desc {
  font-size: 12px;
  color: #909399;
}

.empty-hint {
  text-align: center;
  color: #909399;
  padding: 20px;
  font-size: 13px;
}

/* --- 5.2 时间轴编辑器 (Timeline Editor) --- */
.timeline-editor {
  max-height: 300px;
  overflow-y: auto;
  padding-top: 10px;
}
.step-content {
  display: flex;
  align-items: center;
  gap: 10px;
}
.delay-input {
  display: flex;
  align-items: center;
  gap: 5px;
  font-size: 13px;
  color: #606266;
}
/* 去掉 timeline 最后一个节点的 tail */
:deep(.el-timeline-item:last-child .el-timeline-item__tail) {
  display: none;
}
</style>
