<template>
  <div
    v-loading="isInitializing"
    class="file-browser"
    element-loading-background="rgba(255, 255, 255, 0.8)"
  >
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
        <div class="selected-info">
          <span v-if="selectedFile">已选择: {{ selectedFile.name }}</span>
          <span v-else>未选择文件</span>
        </div>
        <div>
          <el-button @click="onCancel">取消</el-button>
          <el-button type="primary" :disabled="!isSelectionConfirmed" @click="onConfirm"
            >选择</el-button
          >
        </div>
      </div>

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
import { useRobotStore } from '../store/robot' // 引入 Store
import { ElMessage, ElMessageBox } from 'element-plus'
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
  Close
} from '@element-plus/icons-vue'
import ContextMenu from '@imengyu/vue3-context-menu'
import '@imengyu/vue3-context-menu/lib/vue3-context-menu.css'
// #endregion

// #region 2. State & Props
const props = defineProps({
  initialPath: String,
  allowedExtensions: { type: Array, default: () => [] },
  backendId: { type: String, required: true }, // 这里传入的是 UUID
  showClose: { type: Boolean, default: false }, // 控制关闭按钮显示
  hideFooter: { type: Boolean, default: true }
})
const emit = defineEmits(['file-selected', 'cancel', 'close'])

const robotStore = useRobotStore() // Store 实例

const sidebarItems = ref({ places: [], bookmarks: [] })
const isInitializing = ref(true)
const isLoading = ref(false)
const currentPath = ref('/')
const files = ref([])
const selectedFile = ref(null)
const history = ref([])
const historyIndex = ref(-1)
const showHiddenFiles = ref(false)
const viewMode = ref('icon')
const fileTableRef = ref(null)
const globalClipboard = computed(() => robotStore.clipboard)
const isProcessing = ref(false)
const breadcrumbScrollRef = ref(null)

// 对话框状态
const nameDialogVisible = ref(false)
const nameDialogTitle = ref('')
const nameDialogValue = ref('')
const nameDialogPlaceholder = ref('')
const nameDialogType = ref('')
const nameInputRef = ref(null)
const targetFileForRename = ref(null)
const trashPath = ref(null)

// 内联重命名状态
const renamingFileName = ref(null) // 当前正在重命名的文件名 (作为 ID)
const renameInputValue = ref('') // 输入框的值

// 拖拽文件状态
const isDragOver = ref(false) // 是否正在拖拽文件在窗口上方
const uploadStatus = ref({
  uploading: false,
  currentFile: '',
  percent: 0,
  count: 0,
  total: 0
})

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

// #region 3. Helpers (Path & Format)
const pathHelper = {
  join: (...args) => args.filter(Boolean).join('/').replace(/\/+/g, '/'),
  dirname: (p) => (p === '/' ? '/' : p.substring(0, p.lastIndexOf('/')) || '/')
}

const isInTrash = computed(() => {
  if (!trashPath.value || !currentPath.value) return false
  // 判断当前路径是否以回收站路径开头
  return currentPath.value.startsWith(trashPath.value)
})

const filteredFiles = computed(() => {
  if (showHiddenFiles.value) return files.value
  return files.value.filter((file) => !file.name.startsWith('.'))
})

function formatSize(sizeInBytes) {
  if (sizeInBytes === null || sizeInBytes === undefined) return '-'
  if (sizeInBytes === 0) return '0 B'
  const units = ['B', 'KB', 'MB', 'GB', 'TB', 'PB']
  const i = Math.floor(Math.log(sizeInBytes) / Math.log(1024))
  return `${parseFloat((sizeInBytes / Math.pow(1024, i)).toFixed(2))} ${units[i]}`
}

function formatDate(timestamp) {
  if (!timestamp) return '-'
  return new Date(timestamp).toLocaleString()
}

function isFileSelectable(file) {
  if (!file || file.isDir) return false
  if (props.allowedExtensions.length === 0) return true
  const fileName = file.name.toLowerCase()
  return props.allowedExtensions.some((ext) => fileName.endsWith(ext.toLowerCase()))
}

// [新增] 处理面包屑区域的滚轮事件
function onBreadcrumbWheel(e) {
  if (breadcrumbScrollRef.value) {
    // 将垂直滚动转换为水平滚动
    // e.deltaY > 0 是向下滚(向右)，< 0 是向上滚(向左)
    // 加上一定的倍率 (例如 0.5 或 1) 让滚动更平滑
    breadcrumbScrollRef.value.scrollLeft += e.deltaY
  }
}

const isRoot = computed(() => currentPath.value === '/')
const canGoBack = computed(() => historyIndex.value > 0)
const canGoForward = computed(() => historyIndex.value < history.value.length - 1)
const isSelectionConfirmed = computed(() => isFileSelectable(selectedFile.value))
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

// #region 4. Core Logic (API Calls)
async function loadSidebarData() {
  try {
    const res = await robotStore.fsGetSidebar(props.backendId)
    sidebarItems.value = res
    // 找到并记录回收站的路径 (icon 为 Delete 的那个)
    const trashItem = res.places.find((p) => p.icon === 'Delete')
    if (trashItem) {
      trashPath.value = trashItem.path
    }
  } catch (error) {
    ElMessage.error('加载侧边栏失败: ' + error.message)
  }
}

async function loadDirectory(dirPath, isHistoryNav = false) {
  if (!props.backendId) return
  isLoading.value = true
  selectedFile.value = null
  try {
    // 调用 Store Action: /fs/list
    const fileList = await robotStore.fsListDir(props.backendId, dirPath)

    // Agent 返回的数据包含: name, size, isDir, modTime
    // 我们映射一下 modTime -> modified 以适配模板
    files.value = fileList
      .map((f) => ({
        ...f,
        modified: f.modTime
      }))
      .sort((a, b) => {
        if (a.isDir !== b.isDir) return a.isDir ? -1 : 1
        return a.name.localeCompare(b.name, undefined, { numeric: true, sensitivity: 'base' })
      })

    currentPath.value = dirPath

    if (!isHistoryNav) {
      if (historyIndex.value < history.value.length - 1) {
        history.value.splice(historyIndex.value + 1)
      }
      history.value.push(dirPath)
      historyIndex.value++
    }
    nextTick(() => {
      if (breadcrumbScrollRef.value) {
        // 滚动到最大宽度，即最右边
        breadcrumbScrollRef.value.scrollLeft = breadcrumbScrollRef.value.scrollWidth
      }
    })
  } catch (error) {
    ElMessage.error(`加载目录失败: ${error.message || '未知错误'}`)
    // 失败不回退 history，保持现状，避免死循环
  } finally {
    isLoading.value = false
  }
}

async function openEditorWindow(file) {
  const fullPath = pathHelper.join(currentPath.value, file.name)
  try {
    // 依然通过 IPC 打开新窗口，但传入 UUID
    // 新的编辑器窗口内部逻辑也需要重构为使用 Store (此处略)
    await window.api.openFileEditor({
      name: file.name,
      path: fullPath,
      backendId: props.backendId, // 传入 UUID
      backendLabel: props.backendId // 可选：传入 IP 或 Name 用于标题显示
    })
  } catch (error) {
    ElMessage.error('打开编辑器失败: ' + error.message)
  }
}

function getFileIcon(file) {
  if (file.isDir) return Folder
  const name = file.name.toLowerCase()
  if (/\.(zip|rar|7z|tar|gz)$/i.test(name)) return Files
  if (/\.(png|jpg|jpeg|gif|svg|webp)$/i.test(name)) return Picture
  if (/\.(pdf)$/i.test(name)) return Reading
  if (/\.(launch|xml)$/i.test(name)) return Tickets
  if (/\.(yaml|yml)$/i.test(name)) return DataAnalysis
  if (/\.(py|js|ts|json|md|txt|sh)$/i.test(name)) return DocumentCopy
  return Document
}
// #endregion

// #region 5. Event Handlers & Operations
function handleMenuSelect(path) {
  loadDirectory(path)
}
function goUp() {
  if (!isRoot.value) loadDirectory(pathHelper.dirname(currentPath.value))
}
function goBack() {
  if (canGoBack.value) {
    historyIndex.value--
    loadDirectory(history.value[historyIndex.value], true)
  }
}
function goForward() {
  if (canGoForward.value) {
    historyIndex.value++
    loadDirectory(history.value[historyIndex.value], true)
  }
}
function onRowClick(row) {
  selectedFile.value = row
}
function onRowDoubleClick(row) {
  if (row.isDir) loadDirectory(pathHelper.join(currentPath.value, row.name))
  else if (isFileSelectable(row)) {
    selectedFile.value = row
    onConfirm()
  }
}
function onConfirm() {
  if (isSelectionConfirmed.value)
    emit('file-selected', pathHelper.join(currentPath.value, selectedFile.value.name))
  else ElMessage.warning('请选择一个符合要求的文件类型。')
}
function onCancel() {
  emit('cancel')
}

function getItemClass(file) {
  const classes = []
  if (selectedFile.value && file.name === selectedFile.value.name) classes.push('selected')
  if (!file.isDir && !isFileSelectable(file)) classes.push('unselectable-file')
  return classes
}
function tableRowClassName({ row }) {
  return !row.isDir && !isFileSelectable(row) ? 'unselectable-file' : ''
}

// 剪贴板逻辑
function copyFile(file) {
  robotStore.setClipboard({
    sourceId: props.backendId, // 记录来源机器 UUID
    path: pathHelper.join(currentPath.value, file.name),
    mode: 'copy',
    name: file.name,
    isDir: file.isDir
  })
  ElMessage.info(`已复制: ${file.name}`)
}
function cutFile(file) {
  robotStore.setClipboard({
    sourceId: props.backendId,
    path: pathHelper.join(currentPath.value, file.name),
    mode: 'cut', // 跨机剪切 = 复制+删除
    name: file.name,
    isDir: file.isDir
  })
  ElMessage.info(`已剪切: ${file.name}`)
}

async function pasteFile() {
  if (!globalClipboard.value) return
  isProcessing.value = true

  // 提示用户如果是跨机操作可能较慢
  if (globalClipboard.value.sourceId !== props.backendId) {
    ElMessage.info('正在进行跨设备传输，请稍候...')
  }

  try {
    await robotStore.fsPaste(props.backendId, currentPath.value)
    ElMessage.success('粘贴成功')
    loadDirectory(currentPath.value)
  } catch (error) {
    ElMessage.error(`粘贴失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}

async function deleteFile(file, permanent = false) {
  const confirmMsg = permanent
    ? `此操作不可恢复！确定要彻底删除 "${file.name}" 吗？`
    : `确定要将 "${file.name}" 移入回收站吗？`

  try {
    await ElMessageBox.confirm(confirmMsg, '删除确认', {
      confirmButtonText: '删除',
      cancelButtonText: '取消',
      type: 'warning',
      icon: permanent ? 'Warning' : 'InfoFilled'
    })

    isProcessing.value = true
    const fullPath = pathHelper.join(currentPath.value, file.name)

    // [修改] 根据标志调用不同的 API
    if (permanent) {
      await robotStore.fsDeletePermanent(props.backendId, fullPath)
    } else {
      await robotStore.fsDelete(props.backendId, fullPath)
    }

    ElMessage.success(permanent ? '已彻底删除' : '已移入回收站')
    loadDirectory(currentPath.value)
  } catch (error) {
    if (error !== 'cancel') ElMessage.error(`删除失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}

// 还原文件
async function restoreFile(file) {
  isProcessing.value = true
  try {
    const fullPath = pathHelper.join(currentPath.value, file.name)
    await robotStore.fsRestore(props.backendId, fullPath)
    ElMessage.success(`已还原: ${file.name}`)
    loadDirectory(currentPath.value)
  } catch (error) {
    ElMessage.error(`还原失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}

// 拖拽事件处理
// 当文件拖入区域
function onDragEnter(e) {
  // 检查拖拽的是否是文件 (避免拖拽文字也触发)
  if (e.dataTransfer.types.includes('Files')) {
    isDragOver.value = true
  }
}

// 当鼠标离开遮罩层 (取消拖拽)
// eslint-disable-next-line no-unused-vars
function onDragLeave(e) {
  isDragOver.value = false
}

// 当文件被放下 (核心逻辑)
async function onDrop(e) {
  isDragOver.value = false
  const files = e.dataTransfer.files // FileList 对象
  if (files.length === 0) return

  // 启动上传队列
  await processUploadQueue(files)
}

// 4. 处理上传队列
async function processUploadQueue(fileList) {
  uploadStatus.value.total = fileList.length
  uploadStatus.value.count = 0
  uploadStatus.value.uploading = true
  const filesArray = Array.from(fileList)

  for (let i = 0; i < filesArray.length; i++) {
    const file = filesArray[i]
    uploadStatus.value.count = i + 1
    uploadStatus.value.currentFile = file.name
    uploadStatus.value.percent = 0

    try {
      const targetPath = pathHelper.join(currentPath.value, file.name)
      await robotStore.fsUpload(props.backendId, {
        file: file,
        targetPath: targetPath,
        onProgress: (percent) => {
          uploadStatus.value.percent = percent
        }
      })
      ElMessage.success(`${file.name} 上传成功`)
    } catch (e) {
      ElMessage.error(`${file.name} 上传失败: ${e.message}`)
    }
  }

  // 完成后延迟关闭进度条
  setTimeout(() => {
    uploadStatus.value.uploading = false
  }, 1000)
  loadDirectory(currentPath.value)
}

// 拖拽下载处理 (Robot -> Host)
function onDragStart(e, file) {
  if (file.isDir) {
    e.preventDefault() // 暂不支持拖拽文件夹下载
    return
  }
  const client = robotStore.clients[props.backendId]
  if (!client || !client.api) return
  // 构造下载链接
  // 注意：baseURL 通常包含 /api，我们需要确保拼接正确
  // 最终格式: http://192.168.1.10:8080/api/fs/download?path=/home/user/a.txt
  const downloadUrl = `${client.api.defaults.baseURL}/fs/download?path=${encodeURIComponent(pathHelper.join(currentPath.value, file.name))}`
  // 构造 DownloadURL 格式: "MIME:filename:URL"
  // 这是一个 Chrome/Electron 特有的 Hack，允许拖出文件
  const mimeType = 'application/octet-stream'
  const payload = `${mimeType}:${file.name}:${downloadUrl}`
  e.dataTransfer.setData('DownloadURL', payload)
  e.dataTransfer.effectAllowed = 'copy'
}

// 内联重命名逻辑
// 开始重命名
function startInlineRename(file) {
  renamingFileName.value = file.name
  renameInputValue.value = file.name

  // 自动聚焦
  nextTick(() => {
    // 这里因为是 v-for 里的 ref，Vue 会返回数组，或者我们需要动态 ref
    // 简单起见，我们在 template 里用 :id 或其他方式定位，或者只允许单行编辑
    // 下面是一种通用的获取 el-input 焦点的 hack
    const inputs = document.querySelectorAll('.rename-input input')
    if (inputs.length > 0) {
      // 找到对应的 input (只有一个会被渲染出来)
      inputs[0].focus()
      // 全选文件名 (不含扩展名) 体验更好
      const lastDot = file.name.lastIndexOf('.')
      if (lastDot > 0) {
        inputs[0].setSelectionRange(0, lastDot)
      } else {
        inputs[0].select()
      }
    }
  })
}

// 提交重命名
async function finishInlineRename(file) {
  // 如果没有变化，直接取消
  if (!renameInputValue.value || renameInputValue.value === file.name) {
    cancelInlineRename()
    return
  }

  const newName = renameInputValue.value.trim()
  // 简单查重
  if (files.value.some((f) => f.name === newName)) {
    ElMessage.warning('文件名已存在')
    return
  }

  isProcessing.value = true
  try {
    const oldPath = pathHelper.join(currentPath.value, file.name)
    const newPath = pathHelper.join(currentPath.value, newName)

    await robotStore.fsRename(props.backendId, oldPath, newPath)
    ElMessage.success('重命名成功')

    // 乐观更新：直接改本地数据，防止刷新闪烁
    file.name = newName

    cancelInlineRename()
    // 重新加载以确保同步
    loadDirectory(currentPath.value)
  } catch (error) {
    ElMessage.error(`重命名失败: ${error.message}`)
    cancelInlineRename() // 失败也退出编辑状态
  } finally {
    isProcessing.value = false
  }
}

// 取消重命名
function cancelInlineRename() {
  renamingFileName.value = null
  renameInputValue.value = ''
}

// 对话框逻辑
function showNameDialog(type) {
  nameDialogType.value = type
  nameDialogVisible.value = true
  if (type === 'new-folder') {
    nameDialogTitle.value = '新建文件夹'
    nameDialogValue.value = ''
    nameDialogPlaceholder.value = '请输入文件夹名称'
  } else if (type === 'new-file') {
    nameDialogTitle.value = '新建文件'
    nameDialogValue.value = ''
    nameDialogPlaceholder.value = '请输入文件名'
  }
  nextTick(() => nameInputRef.value?.focus())
}

async function handleNameDialogConfirm() {
  if (!nameDialogValue.value.trim()) return
  const name = nameDialogValue.value.trim()
  const targetPath = pathHelper.join(currentPath.value, name)
  isProcessing.value = true
  try {
    if (nameDialogType.value === 'new-folder') {
      await robotStore.fsMkdir(props.backendId, targetPath)
      ElMessage.success('文件夹已创建')
    } else if (nameDialogType.value === 'new-file') {
      await robotStore.fsWriteFile(props.backendId, targetPath, '')
      ElMessage.success('文件已创建')
    } else if (nameDialogType.value === 'rename') {
      const oldPath = pathHelper.join(currentPath.value, targetFileForRename.value.name)
      await robotStore.fsRename(props.backendId, oldPath, targetPath)
      ElMessage.success('重命名成功')
    }
    nameDialogVisible.value = false
    loadDirectory(currentPath.value)
  } catch (error) {
    ElMessage.error(`操作失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}
// #endregion

// #region 6. Context Menu & Lifecycle
function getContextMenuItems(file) {
  const items = []

  // --- 如果在回收站中 ---
  if (isInTrash.value) {
    // 1. 还原
    items.push({
      label: '还原',
      icon: h(RefreshLeft, { style: { color: '#67C23A' } }), // 绿色图标
      onClick: () => restoreFile(file)
    })

    // 2. 剪贴板 (仅允许复制/剪切)
    items.push(
      { label: '复制', icon: h(CopyDocument), onClick: () => copyFile(file) },
      { label: '剪切', icon: h(Scissor), onClick: () => cutFile(file) }
    )

    // 3. 彻底删除 (回收站里只有彻底删除)
    items.push({
      label: '彻底删除',
      icon: h(Delete, { style: { color: '#F56C6C' } }),
      onClick: () => deleteFile(file, true)
    })
  }

  // --- 如果在普通目录中 (原有逻辑) ---
  else {
    if (!file.isDir) {
      items.push({
        label: '编辑',
        icon: h(Edit, { style: { color: '#409EFF' } }),
        onClick: () => openEditorWindow(file)
      })
    }
    items.push(
      { label: '复制', icon: h(CopyDocument), onClick: () => copyFile(file) },
      { label: '剪切', icon: h(Scissor), onClick: () => cutFile(file) }
    )
    items.push(
      {
        label: '重命名',
        icon: h(Edit),
        onClick: () => startInlineRename(file)
      },
      { label: '移入回收站', icon: h(Delete), onClick: () => deleteFile(file, false) },
      {
        label: '彻底删除',
        icon: h(Delete, { style: { color: '#F56C6C' } }),
        onClick: () => deleteFile(file, true)
      }
    )
  }

  // 公共部分
  if (items.length > 0) items.push({ divided: true })
  items.push({ label: '刷新', icon: h(Refresh), onClick: () => loadDirectory(currentPath.value) })

  return items
}

function getContainerMenuItems() {
  const items = []
  // 只有不在回收站时，才允许新建
  if (!isInTrash.value) {
    items.push(
      { label: '新建文件夹', icon: h(FolderAdd), onClick: () => showNameDialog('new-folder') },
      { label: '新建文件', icon: h(DocumentAdd), onClick: () => showNameDialog('new-file') },
      { divided: true }
    )
    // 只有不在回收站时，才允许粘贴
    if (globalClipboard.value) {
      items.push({ label: '粘贴', icon: h(Check), onClick: () => pasteFile() }, { divided: true })
    }
  }

  items.push(
    { label: '刷新', icon: h(Refresh), onClick: () => loadDirectory(currentPath.value) },
    { label: '上一级', disabled: isRoot.value, icon: h(Top), onClick: () => goUp() }
  )
  return items
}

// 获取显示名称 (在回收站中去除冲突后缀)
function getDisplayName(file) {
  if (!isInTrash.value) return file.name
  // 简单的正则：匹配文件名末尾的 .数字 (例如 file.txt.1 或 file.2)
  // 注意：这只是为了视觉美观，实际操作还是用 file.name
  return file.name.replace(/\.\d+$/, '')
}

function onRowContextMenu(row, column, event) {
  event.preventDefault()
  event.stopPropagation()
  selectedFile.value = row
  if (fileTableRef.value) fileTableRef.value.setCurrentRow(row)
  ContextMenu.showContextMenu({
    x: event.x,
    y: event.y,
    zIndex: 9999,
    items: getContextMenuItems(row)
  })
}
function onContainerContextMenu(event) {
  ContextMenu.showContextMenu({
    x: event.x,
    y: event.y,
    zIndex: 9999,
    items: getContainerMenuItems()
  })
}
function onIconContextMenu(event, file) {
  event.stopPropagation()
  selectedFile.value = file
  ContextMenu.showContextMenu({
    x: event.x,
    y: event.y,
    zIndex: 9999,
    items: getContextMenuItems(file)
  })
}

onMounted(async () => {
  isInitializing.value = true
  await loadSidebarData() // 获取侧边栏（包含 Home 路径）

  // 确定启动路径
  let startPath = props.initialPath
  if (!startPath) {
    // 优先使用 sidebarItems 里的 home，否则根目录
    const home = sidebarItems.value.places.find((p) => p.icon === 'House')?.path
    startPath = home || '/'
  }

  await loadDirectory(startPath)
  isInitializing.value = false
})

watch(
  () => props.backendId,
  async (newId, oldId) => {
    if (newId && newId !== oldId) {
      console.log('Backend ID changed in FileBrowser')
      isInitializing.value = true
      await loadSidebarData()
      // 重新回到 Home
      const home = sidebarItems.value.places.find((p) => p.icon === 'House')?.path || '/'
      await loadDirectory(home)
      isInitializing.value = false
    }
  }
)
// #endregion
</script>

<style scoped>
.file-browser {
  display: flex;
  flex-direction: row;
  height: 100%;
  width: 100%;
  border-radius: 4px;
  overflow: hidden;
}
/* 1. 侧边栏样式 */
.sidebar {
  width: 200px;
  flex-shrink: 0;
  border-right: 1px solid var(--el-border-color);
  background-color: var(--el-bg-color-page);
  overflow-y: auto;
}
:deep(.el-menu) {
  border-right: none; /* 移除 ElMenu 默认的右边框 */
}
.el-menu-item-group__title {
  font-size: 12px;
  padding-left: 10px !important;
}
.el-menu-item {
  height: 40px;
}
/* 2. 主内容区样式 */
.main-content {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  position: relative;
  overflow: hidden; /* 防止内容溢出 */
  min-width: 0;
}
/* 2.1 顶部工具栏 */
.header-toolbar {
  display: flex;
  align-items: center;
  justify-content: space-between; /* 左右对齐 */
  padding: 8px;
  border-bottom: 1px solid var(--el-border-color);
  flex-shrink: 0;
  gap: 16px;
}
.left-controls {
  display: flex;
  align-items: center;
  flex-shrink: 1; /* <-- 核心修复 1: 允许左侧容器收缩 */
  min-width: 0; /* 让 flex item 可以收缩 */
}
.breadcrumb-nav {
  margin-left: 12px;
  flex-shrink: 1; /* 允许面包屑在空间不足时收缩 */
}
.right-controls {
  display: flex;
  align-items: center;
  flex-shrink: 0;
}
/* 头部右侧控制区布局 */
.right-controls {
  display: flex;
  align-items: center;
  gap: 8px;
}
/* 嵌入式上传状态 */
.inline-upload-panel {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-right: 15px;
  padding: 4px 10px;
  background-color: #f0f9eb;
  border-radius: 15px;
  border: 1px solid #e1f3d8;
  font-size: 12px;
  color: #67c23a;
  height: 32px;
  box-sizing: border-box;
}
.mini-progress-bar {
  width: 60px;
  height: 6px;
  background-color: #e1f3d8;
  border-radius: 3px;
  overflow: hidden;
}
.bar-inner {
  height: 100%;
  background-color: #67c23a;
  transition: width 0.3s ease;
}

/* 2.2 文件列表 */
.file-list-container {
  flex-grow: 1;
  position: relative;
  overflow-y: hidden;
}
.icon-view-placeholder {
  display: flex;
  align-items: center;
  justify-content: center;
  height: 100%;
}
:deep(.el-table .el-table__cell.is-right .cell) {
  justify-content: flex-end;
}
:deep(.unselectable-file) {
  color: var(--el-text-color-disabled);
  cursor: not-allowed;
}
:deep(.unselectable-file .el-icon) {
  opacity: 0.6;
}
:deep(.unselectable-file:hover > td) {
  background-color: transparent !important; /* 禁用 hover 效果 */
}
/* [新增] 内联重命名输入框样式 */
.rename-container {
  display: flex;
  align-items: center;
  width: 100%;
}
.icon-rename {
  width: 110px; /* 稍微比图标格宽一点 */
  margin-top: 5px;
}
/* 覆盖 el-input 默认样式，让其更紧凑 */
:deep(.rename-input .el-input__wrapper) {
  padding: 1px 5px;
  box-shadow: 0 0 0 1px #409eff inset;
}

.footer-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 12px;
  border-top: 1px solid var(--el-border-color);
  flex-shrink: 0;
}
.selected-info {
  color: var(--el-text-color-secondary);
  font-size: 14px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  padding-right: 10px;
}
/* 3. 图标视图样式 */
.icon-view-container {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(100px, 1fr));
  gap: 2px; /* 图标之间的间距，替代之前的 margin */
  padding: 12px;
  height: 100%;
  overflow-y: auto;
  align-content: flex-start; /* 内容不足一屏时靠上对齐 */
}
.icon-item {
  flex-shrink: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 5px 5px;
  border-radius: 6px;
  cursor: pointer;
  transition: background-color 0.2s ease;
  user-select: none;
  -webkit-user-select: none;
}
.icon-item:hover {
  background-color: var(--el-fill-color-light);
}
.item-icon {
  font-size: 48px;
  margin-bottom: 8px;
  /* 为图标颜色也添加过渡效果 */
  transition: color 0.2s ease;
}
.item-name {
  font-size: 13px;
  line-height: 1.3;
  text-align: center;
  word-break: break-all;
  /* 实现最多两行的文字截断，并显示省略号 */
  display: -webkit-box;
  -webkit-box-orient: vertical;
  -webkit-line-clamp: 2;
  line-clamp: 2;
  overflow: hidden;
  text-overflow: ellipsis;
  height: 34px; /* 根据 line-height 和 font-size 估算出的两行高度 */
}
/* 选中状态的样式 */
.icon-item.selected {
  background-color: var(--el-color-primary-light-9);
  border-color: var(--el-color-primary-light-7);
}
.icon-item.selected .item-icon {
  color: var(--el-color-primary);
}
/* 不可选文件的样式 */
.icon-item.unselectable-file {
  color: var(--el-text-color-disabled);
  cursor: not-allowed;
}
.icon-item.unselectable-file:hover {
  background-color: transparent; /* 禁用 hover 效果 */
}
.icon-item.unselectable-file .item-icon {
  opacity: 0.6;
}
/* 拖拽遮罩层 */
.drag-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(64, 158, 255, 0.85); /* 蓝色半透明 */
  z-index: 2000; /* 保证在最上层 */
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 8px;
  border: 4px dashed #fff;
  margin: 10px; /* 留一点边距 */
  pointer-events: auto; /* 必须接收鼠标事件才能触发 drop */
}
.drag-content {
  text-align: center;
  color: #fff;
}
.drag-content h3 {
  margin-top: 20px;
  font-size: 18px;
  font-weight: 500;
}
/* 上传弹窗样式微调 */
.upload-progress-content {
  padding: 10px 0;
}
.upload-filename {
  margin-bottom: 10px;
  font-weight: bold;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.upload-queue-info {
  margin-top: 10px;
  text-align: right;
  color: #909399;
  font-size: 12px;
}

/* 1. 限制左侧控件容器，防止它无限撑开 */
.left-controls {
  display: flex;
  align-items: center;
  gap: 10px;
  flex: 1; /* 占据剩余空间 */
  min-width: 0; /* [关键] 允许 Flex 子元素缩小到内容宽度以下 */
  overflow: hidden; /* 防止溢出 */
}

/* 2. 历史按钮组保持固定，不许缩小 */
.history-nav {
  flex-shrink: 0;
}

/* 3. 面包屑滚动容器 */
.breadcrumb-wrapper {
  flex: 1; /* 占据剩余所有空间 */
  overflow-x: auto; /* 允许横向滚动 */
  overflow-y: hidden; /* 禁止纵向滚动 */
  white-space: nowrap; /* [关键] 强制内容在一行显示 */

  /* 隐藏滚动条 (Chrome/Safari) */
  &::-webkit-scrollbar {
    display: none;
  }
  /* 隐藏滚动条 (Firefox) */
  scrollbar-width: none;

  /* [可选] 两侧添加遮罩渐变，提示用户还有内容 */
  mask-image: linear-gradient(
    to right,
    transparent,
    black 10px,
    black calc(100% - 10px),
    transparent
  );
  -webkit-mask-image: linear-gradient(
    to right,
    transparent,
    black 10px,
    black calc(100% - 10px),
    transparent
  );

  /* 增加一点内边距，防止文字贴边 */
  padding: 0 5px;
}

/* 4. 面包屑组件本体 */
.breadcrumb-nav {
  display: inline-flex; /* 配合 wrapper 的 nowrap */
  align-items: center;
  flex-wrap: nowrap !important; /* [关键] 强制 Element Plus 不换行 */
}

/* 修复 Element Plus 面包屑项可能自带的换行 */
:deep(.el-breadcrumb__item) {
  float: none; /* 覆盖默认 float */
  display: inline-flex;
  align-items: center;
}

/* 让最后一项（当前目录）稍微加粗，且不许被挤压 */
:deep(.el-breadcrumb__item:last-child .el-breadcrumb__inner) {
  font-weight: 600;
  color: #303133;
}

.close-button {
  border: none;
  padding: 0;
  font-size: 16px;
}
</style>

<style>
/* 注意：这里没有 scoped，是为了覆盖挂载在 body 上的菜单样式 */
.mx-context-menu {
  z-index: 9999 !important; /* 强制层级最高，盖过 Element Plus 的 Dialog */
  background-color: var(--el-bg-color-overlay) !important; /* 确保有背景色 */
  border: 1px solid var(--el-border-color-light) !important;
  box-shadow: var(--el-box-shadow-light) !important;
}

/* 修复菜单项的样式，使其更贴合 Element Plus 主题 */
.mx-context-menu-item {
  color: var(--el-text-color-primary) !important;
  font-family: inherit !important;
}
.mx-context-menu-item:hover {
  background-color: var(--el-color-primary-light-9) !important;
  color: var(--el-color-primary) !important;
}
</style>
