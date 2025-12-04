<template>
  <div v-loading="isInitializing" class="file-browser" element-loading-background="var(--fb-bg)">
    <!-- 1. 左侧导航栏 -->
    <div class="sidebar">
      <el-menu :default-active="currentPath" @select="handleMenuSelect">
        <el-menu-item-group title="常用位置">
          <el-menu-item v-for="place in sidebarItems.places" :key="place.path" :index="place.path">
            <el-icon><component :is="iconMap[place.icon] || Folder" /></el-icon>
            <span>{{ place.name }}</span>
          </el-menu-item>
        </el-menu-item-group>
        <el-menu-item-group title="书签">
          <el-menu-item
            v-for="bookmark in sidebarItems.bookmarks"
            :key="bookmark.path"
            :index="bookmark.path"
          >
            <el-icon><component :is="iconMap[bookmark.icon] || Star" /></el-icon>
            <span>{{ bookmark.name }}</span>
          </el-menu-item>
        </el-menu-item-group>
      </el-menu>
    </div>

    <!-- 2. 右侧主内容区 -->
    <div class="main-content" @dragenter.prevent.stop="onDragEnter" @dragover.prevent.stop>
      <!-- 2.1 顶部工具栏 -->
      <div class="header-toolbar">
        <div class="left-controls">
          <el-button-group class="history-nav">
            <el-button :icon="ArrowLeft" :disabled="!canGoBack" @click="goBack" />
            <el-button :icon="ArrowRight" :disabled="!canGoForward" @click="goForward" />
            <el-button :icon="Top" :disabled="isRoot" @click="goUp" />
          </el-button-group>
          <div
            ref="breadcrumbScrollRef"
            class="breadcrumb-wrapper"
            @wheel.prevent="onBreadcrumbWheel"
          >
            <el-breadcrumb :separator-icon="ArrowRight" class="breadcrumb-nav">
              <el-breadcrumb-item
                v-for="item in breadcrumbItems"
                :key="item.path"
                class="breadcrumb-item"
                @click="loadDirectory(item.path)"
              >
                {{ item.name }}
              </el-breadcrumb-item>
            </el-breadcrumb>
          </div>
        </div>
        <div class="right-controls">
          <!-- 嵌入式上传进度条 -->
          <transition name="el-fade-in">
            <div v-if="uploadStatus.uploading" class="inline-upload-panel">
              <el-icon class="is-loading"><Loading /></el-icon>
              <span class="upload-text">
                上传中 {{ uploadStatus.count }}/{{ uploadStatus.total }}:
                {{ uploadStatus.percent }}%
              </span>
              <div class="mini-progress-bar">
                <div class="bar-inner" :style="{ width: uploadStatus.percent + '%' }"></div>
              </div>
            </div>
          </transition>
          <el-tooltip content="显示隐藏文件" placement="bottom">
            <el-button
              circle
              :type="showHiddenFiles ? 'primary' : ''"
              style="margin-right: 12px"
              @click="showHiddenFiles = !showHiddenFiles"
            >
              <el-icon v-if="showHiddenFiles"><View /></el-icon>
              <el-icon v-else><Hide /></el-icon>
            </el-button>
          </el-tooltip>
          <el-button-group>
            <el-button
              :icon="Tickets"
              :type="viewMode === 'list' ? 'primary' : 'default'"
              @click="viewMode = 'list'"
            />
            <el-button
              :icon="Grid"
              :type="viewMode === 'icon' ? 'primary' : 'default'"
              @click="viewMode = 'icon'"
            />
          </el-button-group>
          <template v-if="showClose">
            <el-divider direction="vertical" style="margin: 0 8px" />
            <el-button
              class="close-button"
              circle
              type="danger"
              plain
              :icon="Close"
              title="关闭窗口"
              @click="$emit('close')"
            />
          </template>
        </div>
      </div>

      <!-- 2.2 文件列表 -->
      <div
        v-loading="isLoading"
        class="file-list-container"
        @contextmenu.prevent="onContainerContextMenu"
      >
        <!-- 表格视图 -->
        <el-table
          v-if="viewMode === 'list'"
          ref="fileTableRef"
          :data="filteredFiles"
          height="100%"
          style="width: 100%"
          highlight-current-row
          :row-class-name="tableRowClassName"
          @row-click="onRowClick"
          @row-dblclick="onRowDoubleClick"
          @row-contextmenu="onRowContextMenu"
        >
          <el-table-column width="50" align="center">
            <template #default="scope">
              <el-icon :size="20"><component :is="getFileIcon(scope.row)" /></el-icon>
            </template>
          </el-table-column>
          <!-- 名称列 -->
          <el-table-column prop="name" label="名称" sortable min-width="200">
            <template #default="scope">
              <!-- 拖动名称可以下载 -->
              <div
                draggable="true"
                style="cursor: grab"
                @dragstart="onDragStart($event, scope.row)"
              >
                <!-- 编辑模式 -->
                <div
                  v-if="renamingFileName === scope.row.name"
                  class="rename-container"
                  @click.stop
                >
                  <el-input
                    v-model="renameInputValue"
                    size="small"
                    class="rename-input"
                    @blur="finishInlineRename(scope.row)"
                    @keyup.enter="finishInlineRename(scope.row)"
                    @keyup.esc="cancelInlineRename"
                  />
                </div>
                <!-- 显示模式 -->
                <span v-else>{{ getDisplayName(scope.row) }}</span>
              </div>
            </template>
          </el-table-column>
          <!-- 大小列 -->
          <el-table-column
            prop="size"
            label="大小"
            width="120"
            align="right"
            sortable
            :sort-by="['isDir', 'size']"
          >
            <template #default="scope">
              <!-- 目录不显示大小 -->
              {{ scope.row.isDir ? '-' : formatSize(scope.row.size) }}
            </template>
          </el-table-column>
          <!-- 修改日期列 -->
          <el-table-column
            prop="modified"
            label="修改日期"
            width="180"
            align="right"
            sortable
            :sort-by="['isDir', 'modified']"
          >
            <template #default="scope">
              {{ formatDate(scope.row.modified) }}
            </template>
          </el-table-column>
        </el-table>

        <!-- 图标视图 -->
        <div v-if="viewMode === 'icon'" class="icon-view-container">
          <div
            v-for="file in filteredFiles"
            :key="file.name"
            class="icon-item"
            :class="getItemClass(file)"
            :title="`${getDisplayName(file)}\n大小: ${file.isDir ? '-' : formatSize(file.size)}\n修改于: ${formatDate(file.modified)}`"
            draggable="true"
            @click="onRowClick(file)"
            @dblclick="onRowDoubleClick(file)"
            @contextmenu.prevent.stop="onIconContextMenu($event, file)"
            @dragstart="onDragStart($event, file)"
          >
            <el-icon class="item-icon">
              <component :is="getFileIcon(file)" />
            </el-icon>
            <div
              v-if="renamingFileName === file.name"
              class="rename-container icon-rename"
              @click.stop
              @dblclick.stop
            >
              <el-input
                v-model="renameInputValue"
                size="small"
                class="rename-input"
                @blur="finishInlineRename(file)"
                @keyup.enter="finishInlineRename(file)"
                @keyup.esc="cancelInlineRename"
              />
            </div>
            <span v-else class="item-name">{{ getDisplayName(file) }}</span>
          </div>
        </div>

        <el-empty
          v-if="!isLoading && filteredFiles.length === 0 && viewMode === 'list'"
          description="文件夹为空"
        />
      </div>

      <!-- 2.3 底部操作栏 -->
      <div v-if="!hideFooter" class="footer-toolbar">
        <div class="footer-left-area">
          <!-- 选中信息 (仅显示文件信息，命令信息由气泡承担) -->
          <div class="selected-info">
            <template v-if="targetType === 'path'">
              <span>当前路径: {{ currentPath }}</span>
            </template>
            <template v-else>
              <span v-if="selectedFile">已选择: {{ selectedFile.name }}</span>
              <span v-else>未选择文件</span>
            </template>
          </div>
        </div>

        <div class="footer-actions">
          <!-- [修改] 启动命令按钮区域 (包含悬浮气泡) -->
          <div v-if="enableRosCommand" class="cmd-trigger-wrapper">
            <!-- 气泡提示 (Chat Bubble) -->
            <transition name="bubble-pop">
              <div v-if="selectedCommandData" class="chat-bubble">
                <div class="bubble-header">
                  <el-icon><Operation /></el-icon>
                  <span>Ready to Run</span>
                  <el-icon class="bubble-close" @click="clearCommand"><Close /></el-icon>
                </div>
                <div class="bubble-body">
                  <code class="cmd-code"
                    >{{ selectedCommandData.cmd }} {{ selectedCommandData.argsStr }}</code
                  >
                </div>
                <!-- 底部小三角 -->
                <div class="bubble-arrow"></div>
              </div>
            </transition>

            <!-- 按钮 (选中命令后变为 Warning 色，提示用户注意) -->
            <el-button
              :type="selectedCommandData ? 'warning' : 'primary'"
              link
              @click="isCmdDialogVisible = true"
            >
              <el-icon style="margin-right: 4px"><Operation /></el-icon>
              {{ selectedCommandData ? '修改命令...' : '在此目录启动命令...' }}
            </el-button>
          </div>

          <el-divider v-if="enableRosCommand" direction="vertical" />

          <el-button @click="$emit('close')">取消</el-button>

          <!-- 确认按钮 -->
          <el-button type="primary" :disabled="!isSelectionConfirmed" @click="onConfirm">
            {{ selectedCommandData ? '保存命令' : '选择' }}
          </el-button>
        </div>
      </div>
      <RosCommandDialog v-model="isCmdDialogVisible" @confirm="handleRosCommandConfirm" />

      <!-- 通用名称输入弹窗 (新建/重命名) -->
      <el-dialog v-model="nameDialogVisible" :title="nameDialogTitle" width="400px" append-to-body>
        <el-input
          ref="nameInputRef"
          v-model="nameDialogValue"
          :placeholder="nameDialogPlaceholder"
          @keyup.enter="handleNameDialogConfirm"
        />
        <template #footer>
          <span class="dialog-footer">
            <el-button @click="nameDialogVisible = false">取消</el-button>
            <el-button type="primary" :loading="isProcessing" @click="handleNameDialogConfirm">
              确定
            </el-button>
          </span>
        </template>
      </el-dialog>
      <!-- 拖拽上传遮罩层 -->
      <div
        v-if="isDragOver"
        class="drag-overlay"
        @dragleave.prevent.stop="onDragLeave"
        @drop.prevent.stop="onDrop"
        @dragover.prevent.stop
      >
        <div class="drag-content">
          <el-icon :size="60"><UploadFilled /></el-icon>
          <h3>释放文件以上传到当前目录</h3>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
// #region 1. Imports
import { ref, onMounted, computed, watch, nextTick, h } from 'vue'
import { useRobotStore } from '../store/robot'
import { ElMessage, ElMessageBox } from 'element-plus'
import ContextMenu from '@imengyu/vue3-context-menu'
import '@imengyu/vue3-context-menu/lib/vue3-context-menu.css'
import RosCommandDialog from './RosCommandDialog.vue'

// Icons
import {
  Folder,
  Document,
  ArrowLeft,
  ArrowRight,
  Top,
  Tickets,
  DataAnalysis,
  House,
  Monitor,
  Download,
  Headset,
  Picture,
  Film,
  Star,
  Files,
  DocumentCopy,
  Reading,
  Grid,
  FolderAdd,
  DocumentAdd,
  Edit,
  Delete,
  CopyDocument,
  Scissor,
  Check,
  Refresh,
  RefreshLeft,
  UploadFilled,
  View,
  Hide,
  Loading,
  Close,
  Operation
} from '@element-plus/icons-vue'
// #endregion

// #region 2. Props & Emits
const props = defineProps({
  initialPath: String,
  initialCommandData: { type: Object, default: null },
  allowedExtensions: { type: Array, default: () => [] },
  backendId: { type: String, required: true }, // 核心：UUID
  showClose: { type: Boolean, default: false },
  hideFooter: { type: Boolean, default: true },
  targetType: { type: String, default: 'file' },
  enableRosCommand: { type: Boolean, default: false }
})

const emit = defineEmits(['file-selected', 'cancel', 'close'])
// #endregion

// #region 3. State Definitions
const robotStore = useRobotStore()

// 3.1 核心数据
const sidebarItems = ref({ places: [], bookmarks: [] })
const currentPath = ref('/')
const files = ref([])
const history = ref([])
const historyIndex = ref(-1)
const trashPath = ref(null) // 回收站路径

// 3.2 UI 状态
const isInitializing = ref(true)
const isLoading = ref(false)
const isProcessing = ref(false) // 用于全局耗时操作遮罩
const showHiddenFiles = ref(false)
const viewMode = ref('icon') // 'list' | 'icon'
const selectedFile = ref(null)
const isCmdDialogVisible = ref(false)
const selectedCommandData = ref(null) // { cmd, argsStr, path }

// 3.3 交互引用
const fileTableRef = ref(null)
const breadcrumbScrollRef = ref(null)
const nameInputRef = ref(null)

// 3.4 临时状态 (重命名/新建)
const nameDialogVisible = ref(false)
const nameDialogTitle = ref('')
const nameDialogValue = ref('')
const nameDialogPlaceholder = ref('')
const nameDialogType = ref('') // 'new-folder' | 'new-file' | 'rename'

const renamingFileName = ref(null) // 内联重命名 ID
const renameInputValue = ref('') // 内联重命名输入值

// 3.5 拖拽与上传
const isDragOver = ref(false)
const uploadStatus = ref({
  uploading: false,
  currentFile: '',
  percent: 0,
  count: 0,
  total: 0
})

// 图标映射
const iconMap = {
  House,
  Monitor,
  Document,
  Download,
  Headset,
  Picture,
  Film,
  Star,
  Folder,
  Grid,
  Delete
}
// #endregion

// #region 4. Computed Logic
const globalClipboard = computed(() => robotStore.clipboard)
const hasCommand = computed(() => !!selectedCommandData.value)

const isRoot = computed(() => currentPath.value === '/')
const canGoBack = computed(() => historyIndex.value > 0)
const canGoForward = computed(() => historyIndex.value < history.value.length - 1)

// 过滤隐藏文件
const filteredFiles = computed(() => {
  const list = files.value || []
  if (showHiddenFiles.value) return list
  return list.filter((file) => !file.name.startsWith('.'))
})

// 判断当前是否在回收站内
const isInTrash = computed(() => {
  if (!trashPath.value || !currentPath.value) return false
  return (
    currentPath.value === trashPath.value || currentPath.value.startsWith(trashPath.value + '/')
  )
})

// 确认按钮状态：如果设定了命令，则允许确认；否则看文件选择
const isSelectionConfirmed = computed(() => {
  if (hasCommand.value) return true
  if (props.targetType === 'path') return true
  return isFileSelectable(selectedFile.value)
})

// 面包屑导航数据
const breadcrumbItems = computed(() => {
  const parts = currentPath.value.split('/').filter(Boolean)
  const items = [{ name: '根目录', path: '/' }]
  let cumulativePath = ''
  for (const part of parts) {
    cumulativePath += `/${part}`
    items.push({ name: part, path: cumulativePath })
  }
  return items
})
// #endregion

// #region 5. Helpers (Path & Display)
const pathHelper = {
  join: (...args) => args.filter(Boolean).join('/').replace(/\/+/g, '/'),
  dirname: (p) => {
    if (p === '/') return '/'
    const idx = p.lastIndexOf('/')
    return idx <= 0 ? '/' : p.substring(0, idx)
  }
}

function formatSize(size) {
  if (size === null || size === undefined) return '-'
  if (size === 0) return '0 B'
  const units = ['B', 'KB', 'MB', 'GB', 'TB', 'PB']
  const i = Math.floor(Math.log(size) / Math.log(1024))
  return `${parseFloat((size / Math.pow(1024, i)).toFixed(2))} ${units[i]}`
}

function formatDate(timestamp) {
  if (!timestamp) return '-'
  return new Date(timestamp).toLocaleString()
}

function isFileSelectable(file) {
  if (hasCommand.value) return false // [核心修改] 互斥逻辑
  if (!file) return false
  if (props.targetType === 'path') return file.isDir
  if (file.isDir) return false
  if (props.allowedExtensions.length === 0) return true
  const name = file.name.toLowerCase()
  return props.allowedExtensions.some((ext) => name.endsWith(ext.toLowerCase()))
}

function getFileIcon(file) {
  if (file.isDir) return Folder
  const n = file.name.toLowerCase()
  if (/\.(zip|rar|7z|tar|gz)$/i.test(n)) return Files
  if (/\.(png|jpg|jpeg|gif|svg|webp)$/i.test(n)) return Picture
  if (/\.(pdf)$/i.test(n)) return Reading
  if (/\.(launch|xml)$/i.test(n)) return Tickets
  if (/\.(yaml|yml)$/i.test(n)) return DataAnalysis
  if (/\.(py|js|ts|json|md|txt|sh)$/i.test(n)) return DocumentCopy
  return Document
}

function getItemClass(file) {
  const classes = []
  if (selectedFile.value && file.name === selectedFile.value.name) classes.push('selected')
  // 如果有命令，或者是本来就不可选的文件 -> 加上置灰样式
  if (hasCommand.value || (!file.isDir && !isFileSelectable(file))) {
    classes.push('unselectable-file')
  }
  // 额外给一个全局禁用的标记，用于 CSS 区分透明度
  if (hasCommand.value) classes.push('global-disabled')
  return classes
}

function tableRowClassName({ row }) {
  if (hasCommand.value || (!row.isDir && !isFileSelectable(row))) {
    return 'unselectable-file global-disabled'
  }
  return ''
}

// 供模板使用的名称显示（去除回收站冲突后缀的视觉干扰）
function getDisplayName(file) {
  if (!isInTrash.value) return file.name
  return file.name.replace(/\.\d+$/, '')
}
// #endregion

// #region 6. API Actions (Business Logic)

// 加载侧边栏
async function loadSidebarData() {
  try {
    const res = await robotStore.fsGetSidebar(props.backendId)
    sidebarItems.value = res
    const trashItem = res.places.find((p) => p.icon === 'Delete')
    if (trashItem) trashPath.value = trashItem.path
  } catch (error) {
    console.error(error) // 侧边栏失败不阻断主流程
  }
}

// 加载目录
async function loadDirectory(dirPath, isHistoryNav = false) {
  if (!props.backendId) return
  isLoading.value = true
  selectedFile.value = null // 清除选中

  try {
    const fileList = await robotStore.fsListDir(props.backendId, dirPath)

    // 排序: 文件夹在前，然后按名称
    files.value = fileList
      .map((f) => ({ ...f, modified: f.modTime }))
      .sort((a, b) => {
        if (a.isDir !== b.isDir) return a.isDir ? -1 : 1
        return a.name.localeCompare(b.name, undefined, { numeric: true, sensitivity: 'base' })
      })

    currentPath.value = dirPath

    // 历史记录管理
    if (!isHistoryNav) {
      if (historyIndex.value < history.value.length - 1) {
        history.value.splice(historyIndex.value + 1)
      }
      history.value.push(dirPath)
      historyIndex.value++
    }

    // 滚动面包屑
    nextTick(() => {
      if (breadcrumbScrollRef.value) {
        breadcrumbScrollRef.value.scrollLeft = breadcrumbScrollRef.value.scrollWidth
      }
    })
  } catch (error) {
    ElMessage.error(`无法加载目录: ${error.message}`)
  } finally {
    isLoading.value = false
  }
}

// 打开编辑器
async function openEditorWindow(file) {
  const fullPath = pathHelper.join(currentPath.value, file.name)
  // 使用 Store 的集中式方法，自动带上 Token
  await robotStore.openFileInEditor(props.backendId, {
    name: file.name,
    path: fullPath
  })
}

// 文件操作
async function performFileAction(actionName, actionFn) {
  isProcessing.value = true
  try {
    await actionFn()
    ElMessage.success(actionName + '成功')
    loadDirectory(currentPath.value)
  } catch (error) {
    if (error !== 'cancel') ElMessage.error(`${actionName}失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}

const copyFile = (file) => {
  robotStore.setClipboard({
    sourceId: props.backendId,
    path: pathHelper.join(currentPath.value, file.name),
    mode: 'copy',
    name: file.name,
    isDir: file.isDir
  })
  ElMessage.info('已复制')
}

const cutFile = (file) => {
  robotStore.setClipboard({
    sourceId: props.backendId,
    path: pathHelper.join(currentPath.value, file.name),
    mode: 'cut',
    name: file.name,
    isDir: file.isDir
  })
  ElMessage.info('已剪切')
}

const pasteFile = async () => {
  if (!globalClipboard.value) return
  if (globalClipboard.value.sourceId !== props.backendId) {
    ElMessage.info('正在进行跨设备传输，请稍候...')
  }
  await performFileAction('粘贴', () => robotStore.fsPaste(props.backendId, currentPath.value))
}

const deleteFile = async (file, permanent = false) => {
  const msg = permanent ? `彻底删除 "${file.name}"？不可恢复！` : `将 "${file.name}" 移入回收站？`
  try {
    await ElMessageBox.confirm(msg, '确认删除', { type: permanent ? 'warning' : 'info' })
    const fullPath = pathHelper.join(currentPath.value, file.name)
    const action = permanent
      ? () => robotStore.fsDeletePermanent(props.backendId, fullPath)
      : () => robotStore.fsDelete(props.backendId, fullPath)
    await performFileAction('删除', action)
    // eslint-disable-next-line no-unused-vars
  } catch (e) {
    /* Cancelled */
  }
}

const restoreFile = async (file) => {
  const fullPath = pathHelper.join(currentPath.value, file.name)
  await performFileAction('还原', () => robotStore.fsRestore(props.backendId, fullPath))
}

const handleRosCommandConfirm = (data) => {
  // 记录命令和当前路径
  selectedCommandData.value = {
    cmd: data.cmd,
    argsStr: data.argsStr,
    path: currentPath.value
  }
}

const clearCommand = () => {
  selectedCommandData.value = null
}

// #endregion

// #region 7. UI Interaction Handlers

// 7.1 Navigation
const handleMenuSelect = (path) => loadDirectory(path)
const goUp = () => !isRoot.value && loadDirectory(pathHelper.dirname(currentPath.value))
const goBack = () =>
  canGoBack.value && (historyIndex.value--, loadDirectory(history.value[historyIndex.value], true))
const goForward = () =>
  canGoForward.value &&
  (historyIndex.value++, loadDirectory(history.value[historyIndex.value], true))
const onBreadcrumbWheel = (e) => {
  if (breadcrumbScrollRef.value) breadcrumbScrollRef.value.scrollLeft += e.deltaY
}

// 7.2 Selection
const onRowClick = (row) => {
  selectedFile.value = row
}
const onRowDoubleClick = (row) => {
  if (row.isDir) loadDirectory(pathHelper.join(currentPath.value, row.name))
  else if (isFileSelectable(row)) {
    selectedFile.value = row
    onConfirm()
  }
}
const onConfirm = () => {
  // Case 1: 选择了 ROS 命令
  if (selectedCommandData.value) {
    emit('file-selected', {
      isCommand: true,
      // 确保这里的 cmd 是刚才 Dialog 传出来的 (e.g. 'rostopic pub')
      cmd: selectedCommandData.value.cmd,
      // 构造 args 数组: [参数字符串, 当前路径]
      args: [selectedCommandData.value.argsStr, selectedCommandData.value.path]
    })
    return
  }
  // Case 2: 普通文件选择 (保持原逻辑)
  if (props.targetType === 'path') {
    const final = selectedFile.value?.isDir
      ? pathHelper.join(currentPath.value, selectedFile.value.name)
      : currentPath.value
    emit('file-selected', final)
  } else if (isSelectionConfirmed.value) {
    emit('file-selected', pathHelper.join(currentPath.value, selectedFile.value.name))
  } else {
    ElMessage.warning('请选择文件')
  }
}

// 7.3 Inline Rename
const startInlineRename = (file) => {
  renamingFileName.value = file.name
  renameInputValue.value = file.name
  nextTick(() => {
    // 聚焦输入框
    const inputs = document.querySelectorAll('.rename-input input')
    if (inputs.length > 0) {
      inputs[0].focus()
      const dot = file.name.lastIndexOf('.')
      dot > 0 ? inputs[0].setSelectionRange(0, dot) : inputs[0].select()
    }
  })
}
const finishInlineRename = async (file) => {
  const newName = renameInputValue.value.trim()
  if (!newName || newName === file.name) return cancelInlineRename()
  if (files.value.some((f) => f.name === newName)) return ElMessage.warning('文件名已存在')

  const oldP = pathHelper.join(currentPath.value, file.name)
  const newP = pathHelper.join(currentPath.value, newName)

  await performFileAction('重命名', () => robotStore.fsRename(props.backendId, oldP, newP))
  cancelInlineRename()
}
const cancelInlineRename = () => {
  renamingFileName.value = null
  renameInputValue.value = ''
}

// 7.4 Dialog Operations (New Folder/File)
const showNameDialog = (type) => {
  nameDialogType.value = type
  nameDialogVisible.value = true
  nameDialogValue.value = ''
  if (type === 'new-folder') {
    nameDialogTitle.value = '新建文件夹'
    nameDialogPlaceholder.value = '文件夹名称'
  } else if (type === 'new-file') {
    nameDialogTitle.value = '新建文件'
    nameDialogPlaceholder.value = '文件名'
  }
  nextTick(() => nameInputRef.value?.focus())
}
const handleNameDialogConfirm = async () => {
  const name = nameDialogValue.value.trim()
  if (!name) return
  const path = pathHelper.join(currentPath.value, name)

  let action = null
  if (nameDialogType.value === 'new-folder')
    action = () => robotStore.fsMkdir(props.backendId, path)
  else if (nameDialogType.value === 'new-file')
    action = () => robotStore.fsWriteFile(props.backendId, path, '')

  if (action) {
    nameDialogVisible.value = false
    await performFileAction('创建', action)
  }
}

// 7.5 Drag & Drop Upload
const onDragEnter = (e) => {
  if (e.dataTransfer.types.includes('Files')) isDragOver.value = true
}
const onDragLeave = () => {
  isDragOver.value = false
}
const onDrop = async (e) => {
  isDragOver.value = false
  const droppedFiles = e.dataTransfer.files
  if (!droppedFiles.length) return

  uploadStatus.value = {
    uploading: true,
    total: droppedFiles.length,
    count: 0,
    percent: 0,
    currentFile: ''
  }

  for (let i = 0; i < droppedFiles.length; i++) {
    const file = droppedFiles[i]
    uploadStatus.value.count = i + 1
    uploadStatus.value.currentFile = file.name
    uploadStatus.value.percent = 0
    try {
      await robotStore.fsUpload(props.backendId, {
        file,
        targetPath: pathHelper.join(currentPath.value, file.name),
        onProgress: (p) => (uploadStatus.value.percent = p)
      })
      ElMessage.success(`${file.name} 上传成功`)
      // eslint-disable-next-line no-unused-vars
    } catch (e) {
      ElMessage.error(`${file.name} 上传失败`)
    }
  }
  setTimeout(() => {
    uploadStatus.value.uploading = false
  }, 1000)
  loadDirectory(currentPath.value)
}

// 7.6 Drag Download (修复鉴权问题)
const onDragStart = (e, file) => {
  if (file.isDir) {
    e.preventDefault()
    return
  }
  const client = robotStore.clients[props.backendId]
  if (!client || !client.api) return

  // 1. 获取完整路径
  const fullPath = pathHelper.join(currentPath.value, file.name)
  // 2. 获取 API Base URL (去除末尾斜杠)
  const baseUrl = client.api.defaults.baseURL.replace(/\/$/, '')
  // 3. 【关键修复】将 Token 附加到 URL Query 参数中
  // 注意：后端需要在 GET /fs/download 接口支持 ?token=... 参数鉴权
  // 如果后端不支持，浏览器原生下载将因为缺少 Header 报 401/403
  const downloadUrl = `${baseUrl}/fs/download?path=${encodeURIComponent(fullPath)}&token=${client.token}`

  // 4. Electron/Chrome 专属格式 "MIME:filename:URL"
  const payload = `application/octet-stream:${file.name}:${downloadUrl}`
  e.dataTransfer.setData('DownloadURL', payload)
  e.dataTransfer.effectAllowed = 'copy'
}
// #endregion

// #region 8. Context Menu Items
const getContextMenuItems = (file) => {
  const items = []
  const isTrash = isInTrash.value

  if (isTrash) {
    items.push(
      {
        label: '还原',
        icon: h(RefreshLeft, { style: { color: '#67C23A' } }),
        onClick: () => restoreFile(file)
      },
      {
        label: '彻底删除',
        icon: h(Delete, { style: { color: '#F56C6C' } }),
        onClick: () => deleteFile(file, true)
      }
    )
  } else {
    if (!file.isDir)
      items.push({
        label: '编辑',
        icon: h(Edit, { style: { color: '#409EFF' } }),
        onClick: () => openEditorWindow(file)
      })
    items.push(
      { label: '复制', icon: h(CopyDocument), onClick: () => copyFile(file) },
      { label: '剪切', icon: h(Scissor), onClick: () => cutFile(file) },
      { label: '重命名', icon: h(Edit), onClick: () => startInlineRename(file) },
      { label: '移入回收站', icon: h(Delete), onClick: () => deleteFile(file, false) },
      {
        label: '彻底删除',
        icon: h(Delete, { style: { color: '#F56C6C' } }),
        onClick: () => deleteFile(file, true)
      }
    )
  }
  return items
}

const getContainerMenuItems = () => {
  const items = []
  if (!isInTrash.value) {
    items.push(
      { label: '新建文件夹', icon: h(FolderAdd), onClick: () => showNameDialog('new-folder') },
      { label: '新建文件', icon: h(DocumentAdd), onClick: () => showNameDialog('new-file') },
      { divided: true }
    )
    if (globalClipboard.value)
      items.push({ label: '粘贴', icon: h(Check), onClick: () => pasteFile() })
  }
  items.push({ label: '刷新', icon: h(Refresh), onClick: () => loadDirectory(currentPath.value) })
  return items
}

const onRowContextMenu = (row, col, e) => {
  e.preventDefault()
  e.stopPropagation()
  selectedFile.value = row
  fileTableRef.value?.setCurrentRow(row)
  ContextMenu.showContextMenu({ x: e.x, y: e.y, zIndex: 9999, items: getContextMenuItems(row) })
}
const onIconContextMenu = (e, file) => {
  e.stopPropagation()
  selectedFile.value = file
  ContextMenu.showContextMenu({ x: e.x, y: e.y, zIndex: 9999, items: getContextMenuItems(file) })
}
const onContainerContextMenu = (e) => {
  ContextMenu.showContextMenu({ x: e.x, y: e.y, zIndex: 9999, items: getContainerMenuItems() })
}
// #endregion

// #region 9. Lifecycle
onMounted(async () => {
  isInitializing.value = true
  await loadSidebarData()
  // 1. 确定初始路径
  let start = props.initialPath
  if (!start) {
    const home = sidebarItems.value.places.find((p) => p.icon === 'House')?.path
    start = home || '/'
  }
  // 2. [新增] 恢复命令回显状态
  if (props.enableRosCommand && props.initialCommandData) {
    selectedCommandData.value = {
      cmd: props.initialCommandData.cmd,
      argsStr: props.initialCommandData.argsStr,
      path: start // 命令绑定的路径就是当前打开的初始路径
    }
  }
  // 3. 加载目录
  await loadDirectory(start)
  isInitializing.value = false
})

watch(
  () => props.backendId,
  async (val, old) => {
    if (val && val !== old) {
      isInitializing.value = true
      await loadSidebarData()
      const home = sidebarItems.value.places.find((p) => p.icon === 'House')?.path || '/'
      await loadDirectory(home)
      isInitializing.value = false
    }
  }
)
// #endregion
</script>

<style>
/* ============================================
   全局主题变量定义 (非 Scoped)
   解决深色模式下 Element Plus 内部组件无法继承变量的问题
   ============================================ */

/* 1. 浅色模式 (默认) */
.file-browser {
  --fb-bg: #ffffff;
  --fb-sidebar-bg: #f5f7fa;
  --fb-border: #e4e7ed;
  --fb-text-main: #303133;
  --fb-text-sub: #909399;
  --fb-hover-bg: rgba(0, 0, 0, 0.04);
  --fb-active-bg: #ecf5ff;
  --fb-active-text: #409eff;
  --fb-bubble-bg: #303133;
  --fb-bubble-text: #fff;

  /* 强制覆盖 Element Plus 内部变量 (关键) */
  --el-color-primary: var(--fb-active-text);
  --el-text-color-primary: var(--fb-text-main);
  --el-text-color-regular: var(--fb-text-sub);
  --el-bg-color: var(--fb-bg);
  --el-fill-color-light: var(--fb-hover-bg);
  --el-border-color-lighter: var(--fb-border);
  /* 修复 Table 背景 */
  --el-table-bg-color: transparent;
  --el-table-tr-bg-color: transparent;
  --el-table-header-bg-color: var(--fb-sidebar-bg);
}

/* 2. 深色模式 */
html.dark .file-browser {
  --fb-bg: #1e1e20;
  --fb-sidebar-bg: #141414;
  --fb-border: #414243;
  --fb-text-main: #e5eaf3;
  --fb-text-sub: #a3a6ad;
  --fb-hover-bg: rgba(255, 255, 255, 0.08);
  --fb-active-bg: rgba(64, 158, 255, 0.2);
  --fb-active-text: #409eff;
  --fb-bubble-bg: #409eff;
  --fb-bubble-text: #fff;

  /* 深色模式下的 Element Plus 覆盖 */
  --el-bg-color: var(--fb-bg);
  --el-fill-color-light: var(--fb-hover-bg);
  --el-border-color-lighter: var(--fb-border);
  --el-table-header-bg-color: var(--fb-sidebar-bg);
}
</style>

<style scoped>
/* ============================================
   1. CSS 变量定义 (Theme System)
   ============================================ */
/* --- 深色模式适配 --- */
:global(html.dark) .file-browser {
  --fb-bg: #1e1e20;
  --fb-sidebar-bg: #141414;
  --fb-border: #414243;
  --fb-text-main: #e5eaf3;
  --fb-text-sub: #a3a6ad;
  --fb-hover-bg: rgba(255, 255, 255, 0.08);
  --fb-active-bg: rgba(64, 158, 255, 0.2);
  --fb-active-text: #409eff;
  --fb-bubble-bg: #409eff; /* 深色模式下气泡用蓝色 */
  --fb-bubble-text: #fff;
}
.file-browser {
  /* --- 浅色模式 --- */
  --fb-bg: #ffffff;
  --fb-sidebar-bg: #f5f7fa;
  --fb-border: #e4e7ed;
  --fb-text-main: #303133;
  --fb-text-sub: #909399;
  --fb-hover-bg: rgba(0, 0, 0, 0.04);
  --fb-active-bg: #ecf5ff;
  --fb-active-text: #409eff;
  --fb-bubble-bg: #303133; /* 气泡背景 */
  --fb-bubble-text: #fff;

  /* 布局与外观 */
  display: flex;
  flex-direction: row;
  height: 100%;
  width: 100%;
  border-radius: 8px;
  overflow: hidden;
  background-color: var(--fb-bg);
  color: var(--fb-text-main);
  border: 1px solid var(--fb-border);
  transition: all 0.3s ease;

  /* [核心修复] 强制覆盖 Element Plus 内部变量，使其跟随 FileBrowser 的主题 */
  --el-color-primary: var(--fb-active-text) !important;
  --el-text-color-primary: var(--fb-text-main) !important;
  --el-text-color-regular: var(--fb-text-sub) !important;
  --el-bg-color: var(--fb-bg) !important;
  --el-border-color-lighter: var(--fb-border) !important;
  --el-fill-color-light: var(--fb-hover-bg) !important;
}

/* ============================================
   2. 侧边栏 (Sidebar)
   ============================================ */
.sidebar {
  width: 200px;
  flex-shrink: 0;
  background-color: var(--fb-sidebar-bg);
  /* 移除右边框，改用背景色区分 */
  border-right: 1px solid transparent;
  overflow-y: auto;
  padding: 10px 0;
  display: flex;
  flex-direction: column;
}

/* 覆盖 Element Menu 样式 */
.sidebar :deep(.el-menu) {
  background: transparent;
  border-right: none;
}

.sidebar :deep(.el-menu-item-group__title) {
  padding: 12px 0 8px 12px !important; /* 上 右 下 左 */
  font-size: 12px;
  color: var(--fb-text-sub);
  font-weight: 600;
  letter-spacing: 0.5px;
}

.sidebar :deep(.el-menu-item) {
  height: 36px;
  line-height: 36px;
  margin: 2px 8px; /* 减小左右 margin，让它更宽 */
  border-radius: 6px;
  color: var(--fb-text-main);
  font-size: 13px;
  padding-left: 6px !important;
  display: flex;
  align-items: center;
}

.sidebar :deep(.el-menu-item .el-icon) {
  margin-right: 8px; /* 控制图标和文字的距离 */
  font-size: 16px;
}

.sidebar :deep(.el-menu-item:hover) {
  background-color: var(--fb-hover-bg);
}

.sidebar :deep(.el-menu-item.is-active) {
  background-color: var(--fb-active-bg);
  color: var(--fb-active-text);
  font-weight: 500;
}

/* ============================================
   3. 主内容区 (Main Content)
   ============================================ */
.main-content {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  position: relative;
  overflow: hidden;
  background-color: var(--fb-bg);
  min-width: 0;
}

/* 3.1 顶部工具栏 */
.header-toolbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 10px 15px;
  /* 使用阴影代替边框，更现代 */
  box-shadow: 0 1px 4px rgba(0, 0, 0, 0.05);
  border-bottom: 1px solid transparent;
  z-index: 10;
  flex-shrink: 0;
  gap: 16px;
}
:global(html.dark) .header-toolbar {
  box-shadow: none;
  border-bottom: 1px solid var(--fb-border);
}

.left-controls {
  display: flex;
  align-items: center;
  flex: 1;
  gap: 12px;
  min-width: 0;
}

/* 历史按钮组 */
.history-nav .el-button {
  padding: 8px;
  border: none;
  background: transparent;
  color: var(--fb-text-main);
}
.history-nav .el-button:hover:not(:disabled) {
  background: var(--fb-hover-bg);
  color: var(--fb-active-text);
}
.history-nav .el-button:disabled {
  background: transparent;
  color: var(--fb-text-sub);
  opacity: 0.5;
}

/* 面包屑区域 */
.breadcrumb-nav {
  display: inline-flex;
  align-items: center;
  flex-wrap: nowrap !important;
  margin-left: 0;
}
/* 修复 Element Plus 面包屑项样式 */
:deep(.el-breadcrumb__item) {
  float: none;
  display: inline-flex;
  align-items: center;
}
/* 普通路径项：悬浮变色 */
:deep(.el-breadcrumb__inner) {
  color: var(--fb-text-sub); /* 默认浅灰 */
  font-weight: normal;
  transition: color 0.2s;
  cursor: pointer !important; /* 确保显示手型 */
}
:deep(.el-breadcrumb__inner:hover) {
  color: var(--fb-text-main); /* 悬浮变深灰 */
}
/* [核心修改] 最后一项（当前目录）：高亮显示 */
:deep(.el-breadcrumb__item:last-child .el-breadcrumb__inner) {
  font-weight: 800; /* 加粗 */
  color: var(--fb-active-text) !important; /* 变为主题蓝 */
  cursor: default !important; /* 当前目录不需要手型，暗示不可点击 */
}
/* [可选] 最后一项悬浮时不改变颜色，保持高亮 */
:deep(.el-breadcrumb__item:last-child .el-breadcrumb__inner:hover) {
  color: var(--fb-active-text) !important;
}

.breadcrumb-wrapper {
  flex: 1;
  overflow-x: auto;
  overflow-y: hidden;
  white-space: nowrap;
  padding: 0 20px;
  /* 左右渐变遮罩 */
  /* 0-15px 是透明到黑色的过渡，15px之后是纯黑(可见) */
  mask-image: linear-gradient(
    to right,
    transparent,
    black 15px,
    black calc(100% - 15px),
    transparent
  );
  -webkit-mask-image: linear-gradient(
    to right,
    transparent,
    black 15px,
    black calc(100% - 15px),
    transparent
  );
}
.breadcrumb-wrapper::-webkit-scrollbar {
  display: none;
}

.breadcrumb-nav {
  display: inline-flex;
  align-items: center;
  flex-wrap: nowrap !important;
  margin-left: 0;
}

/* 右侧控制区 */
.right-controls {
  display: flex;
  align-items: center;
  gap: 8px;
  flex-shrink: 0;
}

/* 嵌入式上传进度条 */
.inline-upload-panel {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 0 12px;
  height: 28px;
  border-radius: 14px;
  background-color: var(--fb-active-bg);
  border: 1px solid var(--fb-active-text);
  color: var(--fb-active-text);
  font-size: 12px;
  margin-right: 10px;
}
.mini-progress-bar {
  width: 50px;
  height: 4px;
  border-radius: 2px;
  background: rgba(64, 158, 255, 0.2);
  overflow: hidden;
}
.bar-inner {
  height: 100%;
  background: #409eff;
  transition: width 0.3s;
}

/* 3.2 文件列表容器 */
.file-list-container {
  flex-grow: 1;
  position: relative;
  overflow: hidden; /* 内部 table/grid 自带滚动 */
}

/* --- 表格视图 (Table View) --- */
:deep(.el-table) {
  --el-table-bg-color: transparent;
  --el-table-tr-bg-color: transparent;
  --el-table-header-bg-color: transparent;
  --el-fill-color-light: var(--fb-hover-bg); /* Hover 颜色 */
  background: transparent !important;
}
:deep(.el-table tr) {
  background-color: transparent !important;
}
:deep(.el-table th.el-table__cell) {
  background-color: var(--fb-hover-bg); /* 表头稍微深一点 */
  color: var(--fb-text-sub);
  font-weight: 600;
  border-bottom: 1px solid var(--fb-border);
}
:deep(.el-table td.el-table__cell) {
  border-bottom: 1px solid var(--fb-border);
}
:deep(.unselectable-file) {
  color: var(--el-text-color-disabled);
  cursor: not-allowed !important;
}
:deep(.unselectable-file .el-icon) {
  opacity: 0.4;
  filter: grayscale(100%); /* 图标变灰 */
}
/* 全局禁用时（选择了命令），让文件列表变得更淡 */
:deep(.global-disabled) {
  opacity: 0.5;
}
:deep(.unselectable-file:hover > td),
:deep(.unselectable-file:hover) {
  background-color: transparent !important;
}

/* 文件名拖拽 & 重命名 */
.rename-container {
  display: flex;
  align-items: center;
  width: 100%;
}
.rename-input :deep(.el-input__wrapper) {
  box-shadow: none !important;
  border: 1px solid #409eff;
  background-color: var(--fb-bg);
  padding: 0 5px;
}

/* --- 图标视图 (Icon View) --- */
.icon-view-container {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(90px, 1fr));
  gap: 8px;
  padding: 15px;
  height: 100%;
  overflow-y: auto;
  align-content: flex-start;
}

.icon-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 10px 5px;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  border: 1px solid transparent;
  color: var(--fb-text-main);
}

.icon-item:hover {
  background-color: var(--fb-hover-bg);
}

.icon-item.selected {
  background-color: var(--fb-active-bg);
  border-color: rgba(64, 158, 255, 0.3);
}

.item-icon {
  font-size: 42px;
  margin-bottom: 6px;
  filter: drop-shadow(0 2px 4px rgba(0, 0, 0, 0.1));
}

.item-name {
  font-size: 12px;
  text-align: center;
  line-height: 1.3;
  width: 100%;
  word-break: break-all;
  display: -webkit-box;
  line-clamp: 2;
  -webkit-line-clamp: 2;
  -webkit-box-orient: vertical;
  overflow: hidden;
}

/* --- 底部操作栏 --- */
.footer-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 15px;
  border-top: 1px solid var(--fb-border);
  background-color: var(--fb-bg); /* 保持不透明以遮挡内容 */
  z-index: 10;
}
.selected-info {
  color: var(--fb-text-sub);
  font-size: 13px;
}
.footer-left-area {
  flex: 1;
  display: flex;
  align-items: center;
  overflow: hidden;
}

.footer-actions {
  display: flex;
  align-items: center;
  gap: 12px; /* 增加间距 */
  flex-shrink: 0;
  position: relative; /* 为气泡定位提供参考（如果需要） */
}
.cmd-trigger-wrapper {
  position: relative;
  display: flex;
  align-items: center;
}

/* [新增] 聊天气泡样式 */
.chat-bubble {
  position: absolute;
  bottom: 100%; /* 位于按钮上方 */
  left: 50%;
  transform: translateX(-50%); /* 水平居中 */
  margin-bottom: 15px; /* 留出箭头空间 */

  width: 260px;
  background-color: var(--fb-bubble-bg);
  color: var(--fb-bubble-text);
  border-radius: 8px;
  padding: 10px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
  z-index: 100;
  pointer-events: auto; /* 允许点击内部关闭按钮 */
}

/* 气泡箭头 */
.bubble-arrow {
  position: absolute;
  bottom: -6px;
  left: 50%;
  transform: translateX(-50%);
  width: 0;
  height: 0;
  border-left: 6px solid transparent;
  border-right: 6px solid transparent;
  border-top: 6px solid var(--fb-bubble-bg);
}

.bubble-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 12px;
  font-weight: bold;
  margin-bottom: 6px;
  opacity: 0.9;
}
.bubble-close {
  cursor: pointer;
  transition: transform 0.2s;
}
.bubble-close:hover {
  transform: scale(1.2);
}

.bubble-body {
  background: rgba(0, 0, 0, 0.2);
  border-radius: 4px;
  padding: 6px;
}
.cmd-code {
  font-family: monospace;
  font-size: 11px;
  word-break: break-all;
  display: -webkit-box;
  line-clamp: 3;
  -webkit-line-clamp: 3;
  -webkit-box-orient: vertical;
  overflow: hidden;
}

/* 气泡动画 */
.bubble-pop-enter-active,
.bubble-pop-leave-active {
  transition: all 0.3s cubic-bezier(0.175, 0.885, 0.32, 1.275);
}
.bubble-pop-enter-from,
.bubble-pop-leave-to {
  opacity: 0;
  transform: translate(-50%, 10px) scale(0.9);
}
/* --- 拖拽上传遮罩 --- */
.drag-overlay {
  position: absolute;
  top: 5px;
  left: 5px;
  right: 5px;
  bottom: 5px;
  background-color: rgba(var(--el-color-primary-rgb), 0.9);
  z-index: 2000;
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 8px;
  border: 2px dashed #fff;
  backdrop-filter: blur(4px);
}
.drag-content {
  text-align: center;
  color: #fff;
}

/* 实用工具类 */
.close-button {
  font-size: 16px;
  border: none;
}
</style>

<style>
/* 上下文菜单样式 (覆盖 vue-context-menu) */
.mx-context-menu {
  /* 适配深色模式 */
  --menu-bg: #fff;
  --menu-border: #e4e7ed;
  --menu-text: #303133;
  --menu-hover: #f5f7fa;
  --menu-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}

html.dark .mx-context-menu {
  --menu-bg: #1e1e20;
  --menu-border: #414243;
  --menu-text: #e5eaf3;
  --menu-hover: rgba(255, 255, 255, 0.08);
  --menu-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
}

.mx-context-menu {
  z-index: 9999 !important;
  background-color: var(--menu-bg) !important;
  border: 1px solid var(--menu-border) !important;
  box-shadow: var(--menu-shadow) !important;
  border-radius: 8px !important;
  padding: 4px 0 !important;
}

.mx-context-menu-item {
  color: var(--menu-text) !important;
  font-family: inherit !important;
  padding: 6px 12px !important;
  font-size: 13px !important;
  cursor: pointer !important;
  transition: background 0.2s;
}

.mx-context-menu-item:hover {
  background-color: var(--menu-hover) !important;
  color: #409eff !important; /* 悬浮高亮色 */
}

.mx-context-menu .mx-context-menu-item-separator {
  background-color: var(--menu-border) !important;
  margin: 4px 0 !important;
}
</style>
