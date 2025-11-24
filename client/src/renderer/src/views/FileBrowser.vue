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
    <div class="main-content">
      <!-- 2.1 顶部工具栏 -->
      <div class="header-toolbar">
        <div class="left-controls">
          <el-button-group class="history-nav">
            <el-button :icon="ArrowLeft" :disabled="!canGoBack" @click="goBack" />
            <el-button :icon="ArrowRight" :disabled="!canGoForward" @click="goForward" />
            <el-button :icon="Top" :disabled="isRoot" @click="goUp" />
          </el-button-group>
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
        <div class="right-controls">
          <el-switch v-model="showHiddenFiles" title="显示隐藏文件" style="margin-right: 12px" />
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
          <el-table-column prop="name" label="名称" sortable />
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
            :title="`${file.name}\n大小: ${file.isDir ? '-' : formatSize(file.size)}\n修改于: ${formatDate(file.modified)}`"
            @click="onRowClick(file)"
            @dblclick="onRowDoubleClick(file)"
            @contextmenu.prevent.stop="onIconContextMenu($event, file)"
          >
            <el-icon class="item-icon">
              <component :is="getFileIcon(file)" />
            </el-icon>
            <span class="item-name">{{ file.name }}</span>
          </div>
        </div>

        <el-empty
          v-if="!isLoading && filteredFiles.length === 0 && viewMode === 'list'"
          description="文件夹为空"
        />
      </div>

      <!-- 2.3 底部操作栏 -->
      <div class="footer-toolbar">
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
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, computed, watch, nextTick } from 'vue'
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
  Files, // 压缩包
  DocumentCopy, // 代码/文本
  Reading, // PDF
  Grid,
  FolderAdd,
  DocumentAdd,
  Edit,
  Delete,
  CopyDocument,
  Scissor,
  Check,
  Refresh
} from '@element-plus/icons-vue'
import ContextMenu from '@imengyu/vue3-context-menu'
import '@imengyu/vue3-context-menu/lib/vue3-context-menu.css'

// --- 组件接口定义 (Props & Emits) ---
const props = defineProps({
  initialPath: String,
  allowedExtensions: { type: Array, default: () => [] },
  backendId: {
    type: String,
    required: true
  }
})
const emit = defineEmits(['file-selected', 'cancel'])

// --- 状态定义 ---
const sidebarItems = ref({ places: [], bookmarks: [] })
const isInitializing = ref(true)
const isLoading = ref(false)
const currentPath = ref('/')
const files = ref([])
const selectedFile = ref(null)
const history = ref([])
const historyIndex = ref(-1)
const showHiddenFiles = ref(false) // 隐藏文件状态
const viewMode = ref('list') // 视图模式状态
const fileTableRef = ref(null) // 文件选中高亮
const clipboard = ref({ path: null, mode: null }) // mode: 'copy' | 'cut'
const isProcessing = ref(false) // 用于对话框 loading
// 名称对话框状态
const nameDialogVisible = ref(false)
const nameDialogTitle = ref('')
const nameDialogValue = ref('')
const nameDialogPlaceholder = ref('')
const nameDialogType = ref('') // 'new-folder' | 'new-file' | 'rename'
const nameInputRef = ref(null)
const targetFileForRename = ref(null) // 仅重命名时使用

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
  Grid
}

// --- 辅助函数 (Helpers) ---
const path = {
  join: (...args) => args.filter(Boolean).join('/').replace(/\/+/g, '/'),
  dirname: (p) => (p === '/' ? '/' : p.substring(0, p.lastIndexOf('/')) || '/')
}
const filteredFiles = computed(() => {
  if (showHiddenFiles.value) {
    return files.value
  }
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
  const date = new Date(timestamp)
  // 使用 toLocaleString 来获得符合用户本地习惯的格式
  return date.toLocaleString(undefined, {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit'
  })
}

function isFileSelectable(file) {
  if (!file || file.isDir) {
    return false // 文件夹不可作为最终选择
  }
  // 如果没有设置过滤规则，则所有文件都可选
  if (props.allowedExtensions.length === 0) {
    return true
  }
  // 检查文件扩展名是否在允许列表中 (不区分大小写)
  const fileName = file.name.toLowerCase()
  return props.allowedExtensions.some((ext) => fileName.endsWith(ext.toLowerCase()))
}

// --- 计算属性 (Computed Properties) ---
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

// --- 核心方法 (Core Methods) ---
async function loadSidebarData() {
  try {
    sidebarItems.value = await window.api.getSidebarPlaces(props.backendId)
  } catch (error) {
    ElMessage.error('加载侧边栏失败: ' + error.message)
  }
}
async function loadDirectory(dirPath, isHistoryNav = false) {
  if (!props.backendId) {
    console.warn('没有 Backend ID，无法加载目录')
    return
  }
  isLoading.value = true
  selectedFile.value = null
  try {
    const fileList = await window.api.fs.readdir(dirPath, props.backendId)
    files.value = fileList.sort((a, b) => {
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
  } catch (error) {
    ElMessage.error(`加载目录失败: ${error.message || '未知错误'}`)
    if (history.value.length > 0) {
      currentPath.value = history.value[historyIndex.value]
    }
  } finally {
    isLoading.value = false
  }
}
async function openEditorWindow(file) {
  const fullPath = path.join(currentPath.value, file.name)
  try {
    // 构造后端显示的名称 (Label)
    let backendLabel = '本地'
    if (props.backendId.startsWith('ssh_')) {
      // 从 ID (ssh_user@192.168.1.x) 中提取 IP 或 Host
      backendLabel = props.backendId.split('@')[1] || props.backendId
    }
    await window.api.openFileEditor({
      name: file.name,
      path: fullPath,
      backendId: props.backendId,
      backendLabel: backendLabel
    })
  } catch (error) {
    ElMessage.error('打开编辑器失败: ' + error.message)
  }
}
function getFileIcon(file) {
  if (file.isDir) {
    return Folder
  }
  const name = file.name.toLowerCase()
  if (/\.(zip|rar|7z|tar|gz)$/i.test(name)) {
    return Files
  }
  if (/\.(png|jpg|jpeg|gif|svg|webp)$/i.test(name)) {
    return Picture // 使用重命名后的图标
  }
  if (/\.(pdf)$/i.test(name)) {
    return Reading
  }
  if (/\.(launch|xml)$/i.test(name)) {
    return Tickets
  }
  if (/\.(yaml|yml)$/i.test(name)) {
    return DataAnalysis
  }
  if (/\.(py|js|ts|json|md|txt|sh)$/i.test(name)) {
    return DocumentCopy
  }
  // 默认文件图标
  return Document
}

// --- 事件处理 (Event Handlers) ---
function handleMenuSelect(path) {
  loadDirectory(path)
}
function goUp() {
  if (!isRoot.value) loadDirectory(path.dirname(currentPath.value))
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
  if (row.isDir) {
    loadDirectory(path.join(currentPath.value, row.name))
  } else if (isFileSelectable(row)) {
    selectedFile.value = row
    onConfirm()
  }
}
function onConfirm() {
  if (isSelectionConfirmed.value) {
    emit('file-selected', path.join(currentPath.value, selectedFile.value.name))
  } else {
    ElMessage.warning('请选择一个符合要求的文件类型。')
  }
}
function onCancel() {
  emit('cancel')
}
function getItemClass(file) {
  const classes = []
  if (selectedFile.value && file.name === selectedFile.value.name) {
    classes.push('selected')
  }
  // 如果是文件且不可选，添加 'unselectable-file' class
  if (!file.isDir && !isFileSelectable(file)) {
    classes.push('unselectable-file')
  }
  return classes
}
function tableRowClassName({ row }) {
  const classes = []
  // 如果是文件且不可选，添加 'unselectable-file' class
  if (!row.isDir && !isFileSelectable(row)) {
    classes.push('unselectable-file')
  }
  return classes.join(' ')
}

// --- 右键菜单 (context-menu) ---
// --- 剪贴板逻辑 ---
function copyFile(file) {
  clipboard.value = {
    path: path.join(currentPath.value, file.name),
    mode: 'copy'
  }
  ElMessage.info(`已复制: ${file.name}`)
}
function cutFile(file) {
  clipboard.value = {
    path: path.join(currentPath.value, file.name),
    mode: 'cut'
  }
  ElMessage.info(`已剪切: ${file.name}`)
}
async function pasteFile() {
  if (!clipboard.value.path) return
  isProcessing.value = true
  try {
    await window.api.fs.paste(
      clipboard.value.path,
      currentPath.value,
      clipboard.value.mode,
      props.backendId
    )
    ElMessage.success('粘贴成功')

    // 如果是剪切，粘贴后清空剪贴板
    if (clipboard.value.mode === 'cut') {
      clipboard.value = { path: null, mode: null }
    }
    loadDirectory(currentPath.value)
  } catch (error) {
    ElMessage.error(`粘贴失败: ${error.message}`)
  } finally {
    isProcessing.value = false
  }
}
async function deleteFile(file) {
  try {
    await ElMessageBox.confirm(`确定要将 "${file.name}" 移入回收站吗？`, '删除确认', {
      confirmButtonText: '删除',
      cancelButtonText: '取消',
      type: 'warning'
    })

    isProcessing.value = true
    const fullPath = path.join(currentPath.value, file.name)
    await window.api.fs.trash(fullPath, props.backendId)
    ElMessage.success('已移入回收站')
    loadDirectory(currentPath.value)
  } catch (error) {
    if (error !== 'cancel') {
      ElMessage.error(`删除失败: ${error.message}`)
    }
  } finally {
    isProcessing.value = false
  }
}
// --- 对话框逻辑 ---
function showNameDialog(type, file = null) {
  nameDialogType.value = type
  nameDialogVisible.value = true
  if (type === 'new-folder') {
    nameDialogTitle.value = '新建文件夹'
    nameDialogValue.value = ''
    nameDialogPlaceholder.value = '请输入文件夹名称'
  } else if (type === 'new-file') {
    nameDialogTitle.value = '新建文件'
    nameDialogValue.value = ''
    nameDialogPlaceholder.value = '请输入文件名 (如 notes.txt)'
  } else if (type === 'rename') {
    nameDialogTitle.value = '重命名'
    nameDialogValue.value = file.name
    targetFileForRename.value = file
  }
  // 自动聚焦
  nextTick(() => {
    nameInputRef.value?.focus()
  })
}

async function handleNameDialogConfirm() {
  if (!nameDialogValue.value.trim()) return
  const name = nameDialogValue.value.trim()
  const targetPath = path.join(currentPath.value, name)
  isProcessing.value = true
  try {
    if (nameDialogType.value === 'new-folder') {
      await window.api.fs.mkdir(targetPath, props.backendId)
      ElMessage.success('文件夹已创建')
    } else if (nameDialogType.value === 'new-file') {
      // 创建空文件
      await window.api.fs.writeFile(targetPath, '', props.backendId)
      ElMessage.success('文件已创建')
    } else if (nameDialogType.value === 'rename') {
      const oldPath = path.join(currentPath.value, targetFileForRename.value.name)
      await window.api.fs.rename(oldPath, targetPath, props.backendId)
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

// 文件/文件夹的菜单
function getContextMenuItems(file) {
  const items = []
  // 编辑 (仅文件且可编辑)
  if (!file.isDir) {
    items.push({
      label: '编辑',
      icon: Edit, // 使用导入的组件对象，imengyu 库支持
      onClick: () => openEditorWindow(file)
    })
  }
  // 剪贴板操作
  items.push(
    {
      label: '复制',
      icon: CopyDocument,
      onClick: () => copyFile(file)
    },
    {
      label: '剪切',
      icon: Scissor,
      onClick: () => cutFile(file)
    }
  )
  // 管理操作
  items.push(
    {
      label: '重命名',
      icon: Edit,
      onClick: () => showNameDialog('rename', file)
    },
    {
      label: '删除',
      icon: Delete,
      onClick: () => deleteFile(file)
    }
  )
  // 分割线 (仅当有上述项时添加，防止空行)
  if (items.length > 0) {
    items.push({ divided: true })
  }
  // 公共操作
  items.push({
    label: '刷新',
    icon: Refresh,
    onClick: () => loadDirectory(currentPath.value)
  })
  return items
}

// 空白处/容器的菜单
function getContainerMenuItems() {
  const items = [
    {
      label: '新建文件夹',
      icon: FolderAdd,
      onClick: () => showNameDialog('new-folder')
    },
    {
      label: '新建文件',
      icon: DocumentAdd,
      onClick: () => showNameDialog('new-file')
    },
    { divided: true }
  ]
  // 粘贴 (仅当剪贴板有内容时显示)
  if (clipboard.value.path) {
    items.push({
      label: '粘贴',
      icon: Check,
      onClick: () => pasteFile()
    })
    items.push({ divided: true })
  }
  items.push(
    {
      label: '刷新',
      icon: Refresh,
      onClick: () => loadDirectory(currentPath.value)
    },
    {
      label: '上一级',
      disabled: isRoot.value,
      icon: Top,
      onClick: () => goUp()
    }
  )
  return items
}

// 事件处理：文件右键
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

// 事件处理：容器右键
function onContainerContextMenu(event) {
  ContextMenu.showContextMenu({
    x: event.x,
    y: event.y,
    zIndex: 9999,
    items: getContainerMenuItems()
  })
}

// 处理图标视图的右键
function onIconContextMenu(event, file) {
  event.stopPropagation()
  // 选中当前项
  selectedFile.value = file
  ContextMenu.showContextMenu({
    x: event.x,
    y: event.y,
    zIndex: 9999,
    items: getContextMenuItems(file)
  })
}

// --- 生命周期钩子 (Lifecycle Hooks) ---
onMounted(async () => {
  isInitializing.value = true
  let startPath = props.initialPath
  const sidebarTask = loadSidebarData()
  const mainContentTask = (async () => {
    if (!startPath) {
      try {
        startPath = (await window.api.fs.getHomeDir(props.backendId)) || '/'
      } catch (error) {
        console.error(`获取主目录失败: ${error.message}`)
        startPath = '/'
      }
    }
    // 获取到路径后立即加载目录
    await loadDirectory(startPath)
  })()
  try {
    // 使用 Promise.allSettled 可以防止一个失败导致另一个也被认为失败
    // 但在这里，我们主要关心主内容加载完毕即可取消遮罩
    await Promise.all([sidebarTask, mainContentTask])
  } catch (e) {
    console.error('初始化部分失败', e)
  } finally {
    // 无论成功失败，只要请求结束，就移除初始化遮罩
    isInitializing.value = false
  }
})

watch(
  () => props.backendId,
  async (newId, oldId) => {
    if (newId && newId !== oldId) {
      console.log('机器ID变了，刷新文件列表...')
      // 重新获取新机器的主目录并加载
      const home = (await window.api.fs.getHomeDir(newId)) || '/'
      loadDirectory(home)
      loadSidebarData() // 刷新侧边栏
    }
  }
)
</script>

<style scoped>
.file-browser {
  display: flex;
  flex-direction: row;
  height: 100%;
  width: 100%;
  border: 1px solid var(--el-border-color);
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
  display: flex;
  flex-wrap: wrap;
  align-content: flex-start; /* 让元素从顶部开始排列，而不是垂直居中 */
  padding: 10px;
  height: 100%;
  overflow-y: auto; /* 容器内部滚动 */
}
.icon-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  width: 80px;
  height: 100px;
  padding: 4px;
  margin: 2px;
  border-radius: 4px;
  cursor: pointer;
  border: 1px solid transparent; /* 占位边框，防止选中时跳动 */
  transition: background-color 0.2s ease;
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
