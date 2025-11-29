<!-- src/renderer/src/components/Dashboard.vue -->
<template>
  <div class="dashboard">
    <!-- [新增] 左侧列容器 -->
    <div class="layout-col left-col">
      <!-- 节点控制部分 -->
      <div class="glass-panel node-list-panel">
        <div class="panel-watermark">
          <el-icon><Operation /></el-icon>
          <span>节点列表</span>
        </div>
        <div style="position: relative; z-index: 2">
          <div
            v-for="node in nodes"
            :key="node.id || node.name"
            class="node-wrapper"
            @mouseenter="node.showEdit = true"
            @mouseleave="node.showEdit = false"
          >
            <el-card
              class="node-card"
              :class="{
                'is-editing': editingNodeId === node.id,
                ['status-' + node.status]: true
              }"
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
                  <el-icon
                    class="expand-arrow"
                    :class="{ 'is-active': expandedNodeIds.has(node.id) }"
                  >
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
                    <el-button type="primary" size="small" round @click.stop="saveNode"
                      >保存</el-button
                    >
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
              <div
                v-show="expandedNodeIds.has(node.id) && editingNodeId !== node.id"
                class="params-drawer"
              >
                <NodeParamsList
                  :node="node"
                  :backend-id="props.currentBackendId"
                  :disabled="!isCurrentBackendReady"
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
                fontSize: '18px',
                marginLeft: '8px'
              }"
              >+ 添加新节点</span
            >
          </el-card>
        </div>
      </div>
    </div>
    <!-- 右侧列容器 -->
    <div class="layout-col right-col">
      <!-- 启动序列面板 -->
      <div class="glass-panel auto-start-panel">
        <!-- [新增] 背景水印标签 -->
        <div class="panel-watermark">
          <el-icon><VideoPlay /></el-icon>
          <span>启动序列</span>
        </div>

        <!-- [修改] 滚动区域 -->
        <el-scrollbar height="calc(100% - 40px)" view-style="padding: 10px 5px 5px 5px;">
          <!-- 空状态提示 -->
          <div v-if="sequences.length === 0" class="empty-hint">暂无序列</div>

          <!-- [修改] 序列列表 - 卡片化 -->
          <div
            v-for="seq in sequences"
            :key="seq.id"
            class="seq-card"
            :class="{ 'is-running': seq._status === 'running' }"
          >
            <div class="seq-card-main">
              <div class="seq-info">
                <div class="seq-name">{{ seq.name }}</div>
                <div class="seq-desc">
                  <el-icon style="vertical-align: middle; margin-right: 2px"
                    ><d-arrow-right
                  /></el-icon>
                  {{ seq.steps.length }} 个步骤
                </div>
              </div>

              <!-- 扁平化操作按钮 -->
              <div class="seq-actions">
                <!-- 更多操作组 -->
                <div class="sub-actions">
                  <el-button
                    link
                    class="sub-btn"
                    :icon="Edit"
                    :disabled="!isCurrentBackendReady"
                    @click="openSeqDialog(seq)"
                  />
                  <el-button
                    link
                    class="sub-btn delete-btn"
                    :icon="Delete"
                    :disabled="!isCurrentBackendReady"
                    @click="deleteSequence(seq)"
                  />
                </div>
                <!-- 运行/停止按钮 -->
                <el-button
                  class="action-btn"
                  :type="seq._status === 'running' ? 'danger' : 'success'"
                  :class="seq._status === 'running' ? 'is-stop' : 'is-play'"
                  circle
                  :icon="seq._status === 'running' ? VideoPause : VideoPlay"
                  :loading="seq._status === 'running' && false"
                  :disabled="!isCurrentBackendReady"
                  @click="toggleSequence(seq)"
                />
              </div>
            </div>

            <!-- 进度条装饰 (可选，仅运行时显示) -->
            <div v-if="seq._status === 'running'" class="running-indicator"></div>
          </div>

          <div
            class="seq-card add-new-seq-card"
            :class="{ 'is-disabled': !isCurrentBackendReady }"
            @click="isCurrentBackendReady && openSeqDialog(null)"
          >
            <el-icon><CirclePlus /></el-icon>
            <span>新建序列</span>
          </div>
        </el-scrollbar>
      </div>

      <div class="glass-panel rosbag-panel">
        <!-- [修改] 水印：将位置留给 CSS 控制，移至右上角 -->
        <div class="panel-watermark top-right-watermark">
          <el-icon><VideoCamera /></el-icon>
          <span>Bag录制</span>
        </div>

        <!-- 内容区域：允许内部滚动，防止顶出按钮 -->
        <div class="rosbag-content">
          <el-form label-position="left" label-width="70px" size="default">
            <!-- [修改] 第一行：缩短宽度的路径选择 -->
            <el-form-item label="存储路径" class="compact-form-item short-input-item">
              <el-input
                v-model="bagForm.path"
                readonly
                :disabled="isRecording"
                placeholder="点击选择保存路径"
                class="clickable-input"
                @click="!isRecording && openFileBrowser('bag-path')"
              >
                <!-- 只有前置图标，没有 append 按钮了 -->
                <template #prefix
                  ><el-icon><Folder /></el-icon
                ></template>
              </el-input>
            </el-form-item>

            <!-- 第二行：前缀 & 分割 -->
            <div class="form-row">
              <el-form-item label="文件前缀" class="compact-form-item item-half">
                <el-input v-model="bagForm.name" placeholder="record" :disabled="isRecording" />
              </el-form-item>

              <el-form-item label="自动分割" class="compact-form-item item-half">
                <div class="split-controls">
                  <el-switch
                    v-model="bagForm.split"
                    inline-prompt
                    active-text="ON"
                    inactive-text="OFF"
                    :disabled="isRecording"
                  />
                  <el-input-number
                    v-if="bagForm.split"
                    v-model="bagForm.size"
                    :min="100"
                    :step="100"
                    :disabled="isRecording"
                    controls-position="right"
                    style="width: 80px; margin-left: 10px"
                    placeholder="MB"
                  />
                </div>
              </el-form-item>
            </div>

            <!-- 第三行：话题选择 -->
            <el-form-item label="录制话题" class="compact-form-item">
              <el-select
                v-model="bagForm.selectedTopics"
                multiple
                collapse-tags
                collapse-tags-tooltip
                placeholder="默认录制所有 (-a)"
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
          </el-form>
        </div>

        <!-- 底部按钮组：固定在底部，flex-shrink 0 -->
        <div class="rosbag-actions">
          <!-- [修改] 回放按钮：支持状态切换 -->
          <el-button
            class="play-bag-btn"
            :class="{ 'is-playing': isPlaying }"
            :type="isPlaying ? 'warning' : 'success'"
            :disabled="isRecording || !isCurrentBackendReady"
            @click="playRosbag"
          >
            <el-icon style="margin-right: 6px; font-size: 1.2em">
              <VideoPause v-if="isPlaying" />
              <VideoPlay v-else />
            </el-icon>
            {{ isPlaying ? '停止回放' : '回放记录' }}
          </el-button>

          <!-- [修改] 录制按钮 -->
          <el-button
            class="record-btn"
            :class="{ 'is-recording': isRecording }"
            :type="isRecording ? 'danger' : 'primary'"
            :loading="false"
            :disabled="!isCurrentBackendReady || (!isRecording && !isRosServiceReady)"
            @click="toggleRecording"
          >
            <el-icon style="margin-right: 6px; font-size: 1.2em">
              <VideoPause v-if="isRecording" />
              <Headset v-else />
            </el-icon>
            {{ isRecording ? '停止录制' : '开始录制' }}
          </el-button>
        </div>
      </div>
    </div>
  </div>
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
  VideoCamera,
  Timer,
  CirclePlus,
  Remove,
  Plus,
  ArrowRight,
  DArrowRight,
  Folder,
  Headset,
  Operation
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
const expandedNodeIds = ref(new Set())

// UI 状态
const fileBrowserContext = reactive({
  visible: false,
  initialPath: '',
  allowedExtensions: [],
  mode: 'launch', // 业务逻辑标识: 'launch' | 'bag-path'
  targetType: 'file', // 组件行为标识: 'file' | 'path'
  targetData: null, // 存储临时的引用对象，例如 param 对象
  callback: null
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
  if (!props.isCurrentBackendReady) return

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

  let currentParams = []
  if (form.id) {
    const existingNode = nodes.value.find((n) => n.id === form.id)
    if (existingNode && existingNode.params) {
      // 深拷贝，防止引用问题
      currentParams = JSON.parse(JSON.stringify(existingNode.params))
    }
  }

  // 构造数据
  const nodeData = {
    id: form.id || `node-${Date.now()}`, // 如果是新节点，生成临时 ID
    name: form.name,
    cmd: form.cmd,
    args: [pathStr], // 简单处理：目前只支持单个路径参数
    description: '',
    params: currentParams
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
  if (editingNodeId.value === node.id) return
  const id = node.id
  if (expandedNodeIds.value.has(id)) {
    expandedNodeIds.value.delete(id)
  } else {
    expandedNodeIds.value.add(id)
  }
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
  fileBrowserContext.callback = null

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
      // data 结构: { paramObject: {...}, onSuccess: fn }
      fileBrowserContext.targetData = data.paramObject // 存入真实的参数对象引用
      fileBrowserContext.callback = data.onSuccess // 存入回调
      currentPathStr = data.paramObject.path || '' // 获取当前路径
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
        if (fileBrowserContext.callback) {
          fileBrowserContext.callback()
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

// 回放状态 (用于控制按钮样式)
const isPlaying = ref(false)

// 回放函数：模拟开关逻辑
const playRosbag = () => {
  if (isPlaying.value) {
    isPlaying.value = false
    // ElMessage.info('停止回放')
  } else {
    isPlaying.value = true
    ElMessage.success('开始回放 (模拟)')
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
watch(
  () => props.currentBackendId,
  (newId, oldId) => {
    if (newId !== oldId) {
      expandedNodeIds.value.clear()
    }
  }
)
</script>

<style scoped>
/* ============================================
   0. 主题变量定义 (Updated for Full Dark Mode Support)
   ============================================ */
.dashboard {
  --bg-color: #f6f8fa;
  --text-primary: #303133;
  --text-secondary: #909399;
  --divider-color: #e4e7ed;

  /* 玻璃卡片统一风格 */
  --panel-bg-color: rgba(255, 255, 255, 0.75); /* 稍微降低不透明度，增强通透感 */
  --panel-border-color: rgba(255, 255, 255, 0.8);
  --panel-radius: 20px;
  --panel-shadow: 0 8px 32px rgba(0, 0, 0, 0.05); /* 柔和的弥散阴影 */

  /* 左侧内部卡片颜色 */
  --card-inner-bg: #ffffff;
  --card-border-gradient: linear-gradient(135deg, #409eff, #8cc5ff, #ecf5ff);
  --card-shadow: 0 4px 12px rgba(64, 158, 255, 0.12);
  --card-hover-scale: scale(1.005);
  --add-card-text: #409eff;
  --add-card-disabled-bg: #f5f7fa;

  /* 布局 */
  height: 100%;
  padding: 12px; /* 全局外边距 */
  gap: 12px; /* 左右列间距 */
  display: flex;
  flex-direction: row;
  background-color: var(--bg-color);
  box-sizing: border-box;
  transition: background-color 0.3s;
}

/* --- 深色模式适配 (当父级 html 或 body 有 .dark 类时生效) --- */
:global(html.dark) .dashboard {
  --bg-color: #141414;
  --text-primary: #e5eaf3;
  --text-secondary: #a3a6ad;
  --divider-color: #4c4d4f;
  --panel-bg-color: rgba(30, 30, 30, 0.75);
  --panel-border-color: rgba(255, 255, 255, 0.1);
  --panel-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
  --card-inner-bg: #1d1e1f;
  --card-border-gradient: linear-gradient(135deg, #337ecc, #1d3d5e, #1d1e1f);
  --card-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
  --add-card-text: #337ecc;
  --add-card-disabled-bg: #2b2b2b;
}

/* ============================================
   0.5 水印标签样式 (New)
   ============================================ */
.panel-watermark {
  position: absolute;
  top: auto;
  bottom: 10px;
  right: 15px;
  font-family: 'Segoe UI', Impact, sans-serif;
  font-weight: 900;
  font-size: 20px;
  letter-spacing: 2px;
  color: var(--text-secondary);
  opacity: 0.25;
  display: flex;
  align-items: center;
  gap: 6px;
  z-index: 0; /* 在最底层 */
  pointer-events: none; /* 不挡鼠标点击 */
  user-select: none;
}
/* 图标稍微大一点 */
.panel-watermark .el-icon {
  font-size: 18px;
}
/* 专门针对 Rosbag 面板的右上角水印 */
.top-right-watermark {
  top: 10px; /* 顶部距离 */
  bottom: auto; /* 取消底部定位 */
  right: 15px; /* 右侧距离 */
}

/* ============================================
   1. 布局列 (Layout Columns)
   ============================================ */
.layout-col {
  display: flex;
  flex-direction: column;
  gap: 12px; /* 垂直方向间距 (针对右侧两块) */
  min-width: 0; /* 防止Flex子项溢出 */
}

.left-col {
  width: 55%; /* 或者 flex: 1 */
}

.right-col {
  flex: 1;
}

/* ============================================
   2. 统一玻璃面板 (The Glass Panel)
   ============================================ */
.glass-panel {
  background: var(--panel-bg-color);
  backdrop-filter: blur(16px); /* 增强磨砂感 */
  -webkit-backdrop-filter: blur(16px);
  border: 1px solid var(--panel-border-color);
  border-radius: var(--panel-radius);
  box-shadow: var(--panel-shadow);
  padding: 15px;
  box-sizing: border-box;
  transition: all 0.3s ease;
  position: relative;
  overflow: hidden; /* 默认隐藏，有滚动需求单独覆盖 */
  will-change: transform, backdrop-filter;
}

/* 悬浮微交互：光影流转 */
.glass-panel:hover {
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.08);
  border-color: rgba(255, 255, 255, 1);
}
:global(html.dark) .glass-panel:hover {
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.5);
  border-color: rgba(255, 255, 255, 0.25);
}

/* ============================================
   3. 面板特定样式
   ============================================ */

/* --- 3.1 左侧：节点控制面板 --- */
.node-list-panel {
  height: 100%;
  /* 滚动条 */
  overflow: hidden !important;
  padding: 0;
  display: flex;
  flex-direction: column;
  position: relative;
}

/* 悬浮动效：增加阴影深度，提亮边框 */
.node-list-panel:hover {
  box-shadow: 0 12px 28px rgba(0, 0, 0, 0.08);
  border-color: rgba(255, 255, 255, 0.9);
}

/* 深色模式下的悬浮微调 */
:global(html.dark) .node-list-panel:hover {
  border-color: rgba(255, 255, 255, 0.2);
  box-shadow: 0 12px 28px rgba(0, 0, 0, 0.5);
}

.node-list-panel > div[style*='position: relative'] {
  padding: 15px;
  height: 100%;
  overflow-y: auto; /* 滚动条移到这里 */
  box-sizing: border-box;

  /* 隐藏默认滚动条，使用自定义样式 */
  scrollbar-width: thin;
}

/* 自定义滚动条样式 (适配 Chrome/Safari) */
.node-list-panel > div[style*='position: relative']::-webkit-scrollbar {
  width: 6px;
}
.node-list-panel > div[style*='position: relative']::-webkit-scrollbar-thumb {
  background-color: rgba(144, 147, 153, 0.3);
  border-radius: 4px;
}
.node-list-panel > div[style*='position: relative']::-webkit-scrollbar-track {
  background: transparent;
}

.node-wrapper {
  margin-bottom: 12px;
  display: flex;
  flex-direction: column;
}

.params-drawer {
  z-index: 1;
  margin-top: -5px;
}

/* --- 节点卡片基础样式 (Card Basic) --- */
.node-card {
  display: flex;
  align-items: center;
  border: rgba(144, 147, 153, 1); /* 原代码保留，虽然通常被 background 覆盖 */
  border-radius: 0px 40px 40px 0px;
  height: 60px;
  /* [修改] 核心背景逻辑：利用双层背景实现渐变边框 */
  background-image:
    linear-gradient(var(--card-inner-bg), var(--card-inner-bg)),
    /* 上层：内容背景 */ var(--card-border-gradient); /* 下层：边框渐变色 */
  background-origin: border-box;
  background-clip: padding-box, border-box;
  /* [修改] 阴影变量化 */
  box-shadow: var(--card-shadow);
  transition: all 0.3s ease-out;
  padding-bottom: 0px;
  z-index: 2;
  cursor: pointer;
  will-change: transform, backdrop-filter;
}
.node-card.is-editing {
  cursor: default;
}

.node-card:hover {
  transform: var(--card-hover-scale);
}

.node-card.has-params-open {
  border-bottom-left-radius: 0;
  border-bottom-right-radius: 0;
  border-bottom-color: transparent;
  box-shadow: 0 4px 12px 0 rgba(0, 0, 0, 0.1);
}

.add-new-card.is-disabled {
  cursor: not-allowed;
  /* [修改] 禁用状态背景 */
  background-color: var(--add-card-disabled-bg);
  border-color: var(--node-list-border);
}

/* --- 卡片头部 (Card Header) --- */
/* 头部容器 */
.node-list-panel :deep(.el-card__header) {
  padding: 0px;
  display: flex;
  flex-direction: row;
  align-items: center;
  height: 80%;
  width: 25%;
  flex-shrink: 0;
  /* [修改] 分割线颜色 */
  border-right: 1px solid var(--divider-color);
  border-bottom: none;
  transition: border-color 0.3s;
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
  font-size: 2.5vh;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  /* [修改] 文字颜色 */
  color: var(--text-primary);
}

/* 节点名称输入框 (编辑模式) */
.node-list-panel .el-card .header-row .el-input {
  height: 3vh;
  width: 90%;
  font-size: 2.5vh;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__wrapper) {
  box-shadow: none !important;
  background-color: transparent;
  padding: 0 !important;
}
.node-list-panel .el-card .header-row .name-input :deep(.el-input__inner) {
  border: none !important;
  padding: 0 !important;
  height: auto;
  line-height: 1.5;
  /* [修改] 输入框文字颜色 */
  color: var(--text-primary);
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
/* Tag 颜色定义 (保持原样，或改为变量，此处保持原样因为颜色比较鲜艳，深浅模式通用) */
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
.expand-arrow {
  margin-right: 8px;
  color: #909399;
  transition: transform 0.3s;
  font-size: 12px;
}
/* 旋转动画 */
.expand-arrow.is-active {
  transform: rotate(90deg);
}

/* --- 卡片内容主体 (Card Body) --- */
.node-list-panel .el-card :deep(.el-card__body) {
  font-size: 2vh;
  flex-grow: 1;
  width: 10%;
  padding: 10px 5px;
  transition: none;
}

.card-body-content {
  /* [修改] 内容文字颜色 */
  color: var(--text-secondary);
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
  /* [新增] 按钮文字颜色继承 */
  color: inherit;
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

/* --- 卡片底部操作区 (Card Footer) --- */
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
.node-list-panel .edit-icon-btn,
.node-list-panel .delete-icon-btn,
.sub-btn {
  transition: all 0.2s;
  border-radius: 8px; /* 稍微方一点的圆角 */
  padding: 5px; /* 增加点击热区 */
  margin: 0 2px;
}

/* 正常状态下：透明背景，颜色保持原样 */
.node-list-panel .edit-icon-btn,
.sub-btn {
  color: var(--text-secondary);
}
.node-list-panel .delete-icon-btn,
.sub-btn.delete-btn {
  color: #f56c6c;
  opacity: 0.8;
}

/* Hover 状态：加背景，变色 */
.node-list-panel .edit-icon-btn:not(:disabled):hover,
.sub-btn:not(.delete-btn):not(:disabled):hover {
  background-color: rgba(64, 158, 255, 0.1); /* 浅蓝底 */
  color: #409eff;
}

.node-list-panel .delete-icon-btn:not(:disabled):hover,
.sub-btn.delete-btn:not(:disabled):hover {
  background-color: rgba(245, 108, 108, 0.1); /* 浅红底 */
  color: #f56c6c;
  opacity: 1;
}

/* 禁用状态 */
.node-list-panel .edit-icon-btn:disabled,
.node-list-panel .delete-icon-btn:disabled {
  opacity: 0.3;
  cursor: not-allowed;
  background: transparent;
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
.node-list-panel .start-stop-btn {
  border: none !important; /* 移除默认边框 */
  color: white !important; /* 强制白字 */
  font-weight: 600;
  letter-spacing: 0.5px;
  box-shadow: 0 3px 8px rgba(0, 0, 0, 0.15); /* 基础阴影 */
  transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
  overflow: hidden;
  position: relative;
  will-change: background-color, box-shadow, transform;
}

/* 状态 A: 准备启动 (Primary - 蓝色渐变) */
.node-list-panel .start-stop-btn.el-button--primary {
  background: linear-gradient(135deg, #409eff, #337ecc) !important;
}
.node-list-panel .start-stop-btn.el-button--primary:not(:disabled):hover {
  transform: scale(1.02);
  box-shadow: 0 5px 12px rgba(64, 158, 255, 0.4);
  filter: brightness(1.1);
}

/* 状态 B: 运行中/停止 (Danger - 红色渐变) */
.node-list-panel .start-stop-btn.el-button--danger {
  background: linear-gradient(135deg, #f56c6c, #c03636) !important;
  /* [修改] 节点是持久运行的，停止按钮需要呼吸，提示正在运行中 */
  animation: btn-breathe-red 2s infinite ease-in-out;
}
.node-list-panel .start-stop-btn.el-button--danger:not(:disabled):hover {
  transform: translateY(-1px);
  box-shadow: 0 5px 12px rgba(245, 108, 108, 0.4);
  filter: brightness(1.1);
}

/* 禁用状态优化 (避免太丑) */
.node-list-panel .start-stop-btn:disabled {
  background: #e4e7ed !important;
  color: #a8abb2 !important;
  box-shadow: none;
  transform: none;
}

/* 添加新节点卡片文字颜色 */
.add-new-card span {
  color: var(--add-card-text) !important;
}

/* --- 3.2 右侧：自动启动面板 (一键启动序列) --- */
.auto-start-panel {
  flex: 1;
  height: auto;
  display: flex;
  flex-direction: column;
  padding: 0 !important; /* 去掉 padding，交给 scrollbar view 处理 */
}

/* 序列卡片通用样式 */
.seq-card {
  position: relative;
  display: flex;
  flex-direction: column;
  background-color: var(--card-inner-bg);
  /* 使用稍小的圆角，显得更精致 */
  border-radius: 12px;
  box-shadow: var(--card-shadow);
  margin-bottom: 12px;
  border: 1px solid transparent; /* 预留边框位置 */
  overflow: hidden;
  transition: all 0.2s ease-out;
  cursor: default;
  transform: scale(1);
}

/* 悬浮效果：轻微放大 + 边框高亮 */
.seq-card:hover {
  transform: scale(1.005);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.08);
  border-color: rgba(64, 158, 255, 0.3);
}

/* 卡片主要内容布局 */
.seq-card-main {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 15px;
}

.seq-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.seq-name {
  font-size: 15px;
  font-weight: 600;
  color: var(--text-primary);
}

.seq-desc {
  font-size: 12px;
  color: var(--text-secondary);
  display: flex;
  align-items: center;
}

/* 按钮区域 */
.seq-actions {
  display: flex;
  align-items: center;
  gap: 15px;
  margin-left: auto; /* 核心：把这一组推到最右边 */
}

/* 主操作按钮 (播放/暂停) */
/* --- 启动序列：主操作按钮升级 --- */
.action-btn.is-stop {
  background: linear-gradient(135deg, #f56c6c, #c03636) !important;
  animation: none !important;
}
.action-btn.is-stop:hover {
  transform: scale(1.05);
  filter: brightness(1.1);
}

/* 次要按钮组 */
.sub-actions {
  display: flex;
  gap: 2px;
  background: rgba(0, 0, 0, 0.03); /* 浅灰底座 */
  border-radius: 16px;
  padding: 2px 6px;
}

/* 运行时的底部动画条 */
.running-indicator {
  height: 3px;
  width: 100%;
  background: linear-gradient(90deg, #67c23a, #95d475);
  animation: progress-indeterminate 2s infinite linear;
  background-size: 200% 100%;
}
@keyframes progress-indeterminate {
  0% {
    background-position: 100% 0;
  }
  100% {
    background-position: -100% 0;
  }
}

/* --- 新建序列卡片 (模仿左侧 Add New) --- */
.add-new-seq-card {
  justify-content: center; /* 内容居中 */
  align-items: center; /* 垂直居中 */
  border: 1px dashed var(--divider-color); /* 虚线框 */
  background-color: transparent; /* 透明背景 */
  box-shadow: none; /* 无阴影 */
  cursor: pointer;
  padding: 15px;
  flex-direction: row; /* 横向排列图标和文字 */
  gap: 8px;
  color: var(--add-card-text);
  font-weight: 500;
  font-size: 15px;
}
.add-new-seq-card:hover {
  border-color: var(--add-card-text);
  background-color: var(--add-card-disabled-bg);
  transform: scale(1.01);
}
.add-new-seq-card.is-disabled {
  filter: grayscale(1);
  opacity: 0.6;
  cursor: not-allowed;
}

/* 暗色模式适配 */
:global(html.dark) .sub-actions {
  background: rgba(255, 255, 255, 0.05);
}
:global(html.dark) .action-btn.is-play {
  background-color: rgba(103, 194, 58, 0.2);
}
:global(html.dark) .action-btn.is-stop {
  background-color: rgba(245, 108, 108, 0.2);
}

/* --- 3.3 右侧：Rosbag 面板 --- */
.rosbag-panel {
  display: flex;
  flex-direction: column;
  padding: 15px 15px 15px 15px !important;
  height: 220px;
}

/* 内容区域：表单 */
.rosbag-content {
  flex: 1; /* 占据所有可用空间 */
  min-height: 0; /* [核心] 允许 Flex 子项收缩小于内容高度 */
  overflow-y: auto; /* [核心] 当空间不足时，只有表单部分出现滚动条 */
  overflow-x: hidden;
  /* 布局微调 */
  display: flex;
  flex-direction: column;
  justify-content: center; /* 垂直居中 */
  margin-bottom: 2px; /* 与底部按钮的间距 */
  /* 隐藏滚动条 (Chrome/Safari) */
  scrollbar-width: none;
}
.rosbag-content::-webkit-scrollbar {
  display: none;
}

/* 紧凑表单样式 */
.compact-form-item {
  margin-bottom: 18px; /* 减少行间距 */
}
.compact-form-item :deep(.el-form-item__label) {
  font-weight: 500;
  color: var(--text-secondary);
  padding-right: 12px;
}

/* 两列布局行 */
.form-row {
  display: flex;
  gap: 20px;
  align-items: center;
}
.item-half {
  flex: 1;
  min-width: 0;
}

/* 分割控制组 */
.split-controls {
  display: flex;
  align-items: center;
  width: 100%;
}
.unit-text {
  font-size: 12px;
  color: var(--text-secondary);
  margin-left: 6px;
}

/* 输入框样式调整 */
.short-input-item {
  width: 65%; /* [核心] 限制宽度，给右上角水印留出 35% 的空间 */
  transition: width 0.3s;
}

/* 让只读输入框看起来像按钮 */
.clickable-input :deep(.el-input__wrapper) {
  cursor: pointer;
  background-color: rgba(255, 255, 255, 0.5); /* 默认半透明 */
  box-shadow: 0 0 0 1px #dcdfe6 inset;
}
.clickable-input :deep(.el-input__inner) {
  cursor: pointer;
  font-weight: 500;
  color: var(--text-primary);
}
.clickable-input:hover :deep(.el-input__wrapper) {
  background-color: #fff;
  box-shadow: 0 0 0 1px var(--add-card-text) inset;
}

/* 紧凑表单项 */
.compact-form-item {
  margin-bottom: 12px; /* 进一步减小间距，防止太容易触发滚动 */
}
.compact-form-item :deep(.el-form-item__label) {
  font-weight: 500;
  color: var(--text-secondary);
  padding-right: 12px;
  line-height: 32px; /* 垂直对齐 */
}

/* 底部按钮区域 - 永远固定在底部 */
.rosbag-actions {
  flex-shrink: 0; /* [核心] 禁止被压缩，确保永远可见 */
  display: flex;
  gap: 2px;
  padding-top: 2px; /* 稍微留点呼吸空间 */
}

/* 按钮通用风格 (Base Style) */
.play-bag-btn,
.record-btn {
  height: 44px;
  border-radius: 22px;
  font-weight: bold;
  font-size: 15px;
  letter-spacing: 1px;
  border: none;
  transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
  position: relative;
  overflow: hidden;
  color: white; /* 统一白字 */
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1); /* 基础阴影 */
}

/* 基础 Hover (通用) */
.play-bag-btn:not(.is-playing):hover,
.record-btn:not(.is-recording):hover {
  transform: scale(1.02);
  filter: brightness(1.1); /* 简单提亮，不破坏渐变 */
}
.play-bag-btn:not(.is-playing):active,
.record-btn:not(.is-recording):active {
  transform: scale(1.02);
}

/* 1. 回放按钮 (Play Button) */
.play-bag-btn {
  flex: 1;
}

/* 状态 A: 待机 (绿色渐变) */
.play-bag-btn:not(.is-playing) {
  background: linear-gradient(135deg, #67c23a, #429e18); /* Element Success Green */
  box-shadow: 0 4px 10px rgba(103, 194, 58, 0.3);
}

/* 状态 B: 播放中 (橙色呼吸) */
.play-bag-btn.is-playing {
  background-color: #e6a23c; /* Element Warning Orange */
  animation: breathe-orange 2s infinite ease-in-out;
}
/* Hover 时保持呼吸，仅微调颜色暗示可交互 */
.play-bag-btn.is-playing:hover {
  background-color: #cf9236;
}

/* 2. 录制按钮 (Record Button) */
.record-btn {
  flex: 1.5; /* 稍微宽一点，作为主操作 */
}

/* 状态 A: 待机 (蓝色渐变) */
.record-btn:not(.is-recording) {
  background: linear-gradient(135deg, #409eff, #337ecc); /* Element Primary Blue */
  box-shadow: 0 4px 10px rgba(64, 158, 255, 0.3);
}

/* 状态 B: 录制中 (红色呼吸) */
.record-btn.is-recording {
  background-color: #f56c6c; /* Element Danger Red */
  animation: breathe-red 2s infinite ease-in-out;
}
/* Hover 时保持呼吸，仅微调颜色暗示可交互 */
.record-btn.is-recording:hover {
  background-color: #c95050;
}

/* 动画定义 (Animations) */
/* 红色呼吸 (录制) */
@keyframes breathe-red {
  0%,
  100% {
    box-shadow: 0 0 0 0 rgba(245, 108, 108, 0.7);
  }
  50% {
    box-shadow: 0 0 0 8px rgba(245, 108, 108, 0.25); /* 扩散更柔和 */
  }
}

/* 橙色呼吸 (回放) */
@keyframes breathe-orange {
  0%,
  100% {
    box-shadow: 0 0 0 0 rgba(230, 162, 60, 0.7);
  }
  50% {
    box-shadow: 0 0 0 8px rgba(230, 162, 60, 0.25);
  }
}

/* --- 3.4 右侧通用元素 --- */
.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 10px;
  padding-bottom: 8px;
  border-bottom: 1px solid rgba(0, 0, 0, 0.03);
}
.panel-header h2 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  color: var(--text-primary);
}

/* 序列列表卡片 */
.seq-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 12px;
  margin-bottom: 8px;
  border-radius: 12px;
  background-color: rgba(255, 255, 255, 0.4);
  border: 1px solid transparent;
  transition: all 0.2s;
}
.seq-item:hover {
  background-color: #fff;
  transform: translateX(2px);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
}
.seq-name {
  font-weight: 600;
  font-size: 14px;
  color: var(--text-primary);
}
.seq-desc {
  font-size: 12px;
  color: var(--text-secondary);
}

/* 表单控件透明化 */
.rosbag-panel .el-input__wrapper,
.rosbag-panel .el-select__wrapper {
  background-color: rgba(255, 255, 255, 0.5) !important;
  box-shadow: none !important;
  border: 1px solid #dcdfe6;
}
.rosbag-panel .el-input__wrapper:hover,
.rosbag-panel .el-select__wrapper:hover {
  background-color: #fff !important;
}
.rosbag-panel .el-input__wrapper.is-focus {
  background-color: #fff !important;
  border-color: #409eff;
  box-shadow: 0 0 0 1px #409eff !important;
}

/* 1. 运行中 (Running) -> Success Green */
/* 对应 el-tag success: rgb(103, 194, 58) */
.node-card.status-running {
  border-color: transparent; /* 保持无边框，靠阴影 */
  animation: shadow-flow-green 3s infinite ease-in-out;
}

/* 2. 启动/停止中 (Starting/Stopping) -> Warning Orange */
/* 对应 el-tag warning: rgb(230, 193, 45) */
.node-card.status-starting,
.node-card.status-stopping {
  animation: shadow-flow-yellow 2s infinite ease-in-out; /* 稍微快一点，表示忙碌 */
}

/* 3. 错误/崩溃 (Error/Crashed) -> Danger Red */
/* 对应 el-tag error: rgb(245, 108, 108) */
.node-card.status-crashed,
.node-card.status-error {
  animation: shadow-flow-red 1.5s infinite ease-in-out; /* 急促的呼吸 */
}

/* 4. 已停止 (Stopped) -> 保持默认蓝色渐变阴影 */
.node-card.status-stopped {
  /* 保持默认的 var(--card-shadow) 即可，或者稍微淡一点 */
  box-shadow: var(--card-shadow);
}

/* --- 定义流光动画 (Shadow Animations) --- */

/* 1. 绿色流光 (Running): 尺寸基本不变，通过透明度变化体现能量感 */
@keyframes shadow-flow-green {
  0%,
  100% {
    /* 基准状态：与静止态大小一致 (12px blur)，颜色为绿 */
    box-shadow: 0 4px 12px rgba(103, 194, 58, 0.2);
  }
  50% {
    /* 活跃状态：稍微扩散一点点 (16px blur)，亮度增加 */
    box-shadow: 0 6px 16px rgba(103, 194, 58, 0.4);
  }
}

/* 2. 黄色流光 (Busy) */
@keyframes shadow-flow-yellow {
  0%,
  100% {
    box-shadow: 0 4px 12px rgba(230, 193, 45, 0.2);
  }
  50% {
    box-shadow: 0 6px 16px rgba(230, 193, 45, 0.4);
  }
}

/* 3. 红色流光 (Error) */
@keyframes shadow-flow-red {
  0%,
  100% {
    box-shadow: 0 4px 12px rgba(245, 108, 108, 0.2);
  }
  50% {
    box-shadow: 0 6px 16px rgba(245, 108, 108, 0.4);
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
   5. 全局功能样式 (右侧面板内部组件)
   ============================================ */

/* --- 5.1 序列列表 (Sequence List) --- */
/* 注意：移除原有的 .auto-start-panel 全局样式，避免冲突 */

/* 序列项 - 改为卡片式 */
.seq-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 15px;
  margin-bottom: 8px; /* 项之间的间距 */
  border-radius: 12px;
  background-color: rgba(255, 255, 255, 0.5); /* 列表项微白背景 */
  border: 1px solid transparent;
  transition: all 0.2s ease;
}

/* 最后一个元素不再特殊处理边框，因为现在是卡片分离的 */
.seq-item:last-child {
  border-bottom: 1px solid transparent;
  margin-bottom: 0;
}

/* 悬停效果 */
.seq-item:hover {
  background-color: white;
  transform: translateX(4px); /* 悬停时微微右移 */
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05);
  border-color: rgba(64, 158, 255, 0.2);
}

/* 选中或运行中的效果 (可选，如果你在 JS 里加了 class) */
.seq-item.is-active {
  background-color: #ecf5ff;
  border-color: #d9ecff;
}

.seq-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.seq-name {
  font-size: 15px;
  font-weight: 600;
  color: #303133;
}

.seq-desc {
  font-size: 12px;
  color: #909399;
}

/* 操作按钮组微调 */
.seq-actions {
  opacity: 0.8;
  transition: opacity 0.2s;
}
.seq-item:hover .seq-actions {
  opacity: 1;
}

.empty-hint {
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #c0c4cc;
  font-size: 14px;
  border: 2px dashed #e4e7ed;
  border-radius: 12px;
  margin: 10px 0;
}

/* --- 5.2 Form 控件美化 (针对右侧面板内的输入框) --- */
/* 让输入框背景半透明，融入玻璃背景 */
.function-panel .el-input__wrapper,
.function-panel .el-select__wrapper {
  background-color: rgba(255, 255, 255, 0.6) !important;
  box-shadow: none !important;
  border: 1px solid #dcdfe6;
  transition: all 0.2s;
}

.function-panel .el-input__wrapper:hover,
.function-panel .el-select__wrapper:hover {
  background-color: #fff !important;
  border-color: #c0c4cc;
}

.function-panel .el-input__wrapper.is-focus,
.function-panel .el-select__wrapper.is-focused {
  background-color: #fff !important;
  border-color: #409eff;
  box-shadow: 0 0 0 1px #409eff !important;
}

/* 标签样式微调 */
.function-panel .el-form-item__label {
  font-weight: 500;
  color: #606266;
}
</style>
