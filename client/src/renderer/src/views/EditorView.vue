<template>
  <div class="ide-layout" tabindex="0" @keydown.ctrl.s.prevent="handleSave">
    <!-- 1. 左侧侧边栏：会话列表 -->
    <aside class="sidebar">
      <div class="sidebar-title">文本编辑器</div>
      <div class="session-list">
        <div
          v-for="session in sessions"
          :key="session.id"
          class="session-item"
          :class="{ active: activeSessionId === session.id }"
          @click="activeSessionId = session.id"
        >
          <div class="session-info">
            <el-icon class="icon"><Monitor /></el-icon>
            <span class="label" :title="session.label">{{ session.label }}</span>
          </div>
          <el-icon class="close-btn" @click.stop="removeSessionTab(session.id)"><Close /></el-icon>
        </div>
        <div class="add-session-group">
          <!-- 触发器 Tab -->
          <div
            class="session-item add-trigger"
            :class="{ 'is-open': isAddMenuOpen }"
            @click="isAddMenuOpen = !isAddMenuOpen"
          >
            <div class="session-info">
              <el-icon class="icon"><Plus /></el-icon>
              <span class="label">添加会话...</span>
            </div>
            <!-- 旋转的小箭头 -->
            <el-icon class="arrow-icon" :class="{ rotated: isAddMenuOpen }"><ArrowRight /></el-icon>
          </div>

          <!-- 下拉菜单内容 -->
          <div v-show="isAddMenuOpen" class="add-dropdown">
            <!-- 情况 1: 有可添加的后端 -->
            <template v-if="availableBackendsToAdd.length > 0">
              <div
                v-for="id in availableBackendsToAdd"
                :key="id"
                class="dropdown-item"
                @click="handleAddSession(id)"
              >
                <span class="dot online"></span>
                <span class="label" :title="id">{{ id }}</span>
                <el-icon class="add-icon"><Plus /></el-icon>
              </div>
            </template>

            <!-- 情况 2: 所有后端都已添加 -->
            <div v-else-if="activeBackendIds.length > 0" class="dropdown-info">
              <el-icon><Check /></el-icon>
              <span>所有会话已添加</span>
            </div>

            <!-- 情况 3: 没有任何后端连接 -->
            <div v-else class="dropdown-info warning">
              <el-icon><Warning /></el-icon>
              <span>请先建立一个会话</span>
            </div>
          </div>
        </div>
      </div>
    </aside>

    <!-- 2. 右侧主区域：文件编辑 -->
    <main class="main-area">
      <!-- 情况 A: 有选中的会话 -->
      <div v-if="currentSession" class="editor-workspace">
        <!-- 只有在线时才显示编辑器 -->
        <template v-if="currentSession.isOnline">
          <div class="tabs-wrapper" @wheel="handleTabsWheel">
            <el-tabs
              ref="tabsRef"
              v-model="currentSession.activeFile"
              type="card"
              class="file-tabs"
              @tab-remove="(path) => removeFileTab(currentSession, path)"
            >
              <!-- 1. 遍历显示已打开的文件 -->
              <el-tab-pane
                v-for="file in currentSession.files"
                :key="file.path"
                :name="file.path"
                closable
              >
                <template #label>
                  <!-- 加上 no-drag 确保 Tab 可点击 -->
                  <span class="tab-label-content" :class="{ 'dirty-file': file.isDirty }">
                    {{ file.name }}{{ file.isDirty ? ' ●' : '' }}
                  </span>
                </template>

                <div v-loading="file.loading" class="monaco-container">
                  <vue-monaco-editor
                    v-model:value="file.content"
                    :language="file.language"
                    theme="vs"
                    :options="monacoOptions"
                    class="monaco-editor-instance"
                    @change="file.isDirty = file.content !== file.originalContent"
                  />
                </div>
              </el-tab-pane>

              <!-- 2. 固定在末尾的“添加”页签 -->
              <!-- name 设置为一个特殊字符串 -->
              <el-tab-pane name="__add_tab__" :closable="false">
                <template #label>
                  <!-- 关键：设置 no-drag，否则因为 Header 可拖拽导致无法点击 -->
                  <span class="add-tab-btn">
                    <el-icon><Plus /></el-icon>
                  </span>
                </template>

                <!-- 嵌入文件浏览器 -->
                <div class="embedded-browser-wrapper">
                  <FileBrowser
                    :backend-id="currentSession.id"
                    :initial-path="'/'"
                    @file-selected="(path) => handleEmbeddedFileSelect(currentSession, path)"
                  />
                </div>
              </el-tab-pane>
            </el-tabs>
          </div>
        </template>

        <!-- 离线状态 -->
        <div v-else class="empty-state offline-state">
          <el-icon :size="50" color="#f56c6c"><Connection /></el-icon>
          <h3>连接断开</h3>
          <p>后端 {{ currentSession.label }} 当前不可用。</p>
        </div>
      </div>

      <!-- 情况 B: 没有选择会话 -->
      <div v-else class="empty-state">
        <div class="empty-content">
          <p>请在左侧选择一个会话</p>
        </div>
      </div>

      <!-- 底部状态栏 -->
      <div class="status-bar">
        <div class="left">
          <!-- 如果当前选中的是添加页签，状态栏显示提示 -->
          <span v-if="currentSession && currentSession.activeFile === '__add_tab__'">
            {{ currentSession.label }} &nbsp;>&nbsp; 文件浏览器
          </span>
          <span v-else-if="currentSession && currentFile">
            {{ currentSession.label }} &nbsp;>&nbsp; {{ currentFile.path }}
          </span>
        </div>
        <div class="right">
          <!-- 语言选择下拉菜单 -->
          <el-dropdown
            v-if="currentFile"
            trigger="hover"
            placement="top"
            @command="handleLanguageChange"
          >
            <!-- 触发文字 -->
            <span class="language-selector">
              {{ getLanguageLabel(currentFile.language) }}
            </span>

            <!-- 下拉菜单内容 -->
            <template #dropdown>
              <el-dropdown-menu class="lang-dropdown">
                <el-dropdown-item
                  v-for="opt in languageOptions"
                  :key="opt.id"
                  :command="opt.id"
                  :class="{ 'is-selected': opt.id === currentFile.language }"
                >
                  {{ opt.label }}
                </el-dropdown-item>
              </el-dropdown-menu>
            </template>
          </el-dropdown>

          <span v-if="currentFile?.isDirty" class="dirty-badge">未保存</span>
        </div>
      </div>
    </main>
    <el-dialog v-model="isFileBrowserVisible" title="打开文件" width="70%" destroy-on-close>
      <!-- 
        currentSession 存在时才渲染 FileBrowser 
        backend-id: 传入当前会话的 ID，这样 FileBrowser 就知道去哪个后端读目录
      -->
      <FileBrowser
        v-if="currentSession"
        :backend-id="currentSession.id"
        :initial-path="'/'"
        @file-selected="handleFileBrowserSelection"
        @cancel="isFileBrowserVisible = false"
      />
    </el-dialog>
  </div>
</template>

<script setup>
import { ref, onMounted, computed, watch, h, reactive } from 'vue'
import { ElMessage, ElMessageBox, ElInput } from 'element-plus'
import {
  Monitor, // 用于左侧会话列表
  Close, // 用于关闭按钮
  Plus, // 用于新增 Tab 的 "+" 号
  Connection, // 用于离线状态提示
  ArrowRight,
  Check,
  Warning
} from '@element-plus/icons-vue'
import { VueMonacoEditor, loader } from '@guolao/vue-monaco-editor'
import FileBrowser from './FileBrowser.vue'

// --- 1. 引入本地 Monaco Editor 及其 Workers ---
import * as monaco from 'monaco-editor'

// Vite 专用的 Worker 导入语法 (?worker)
import editorWorker from 'monaco-editor/esm/vs/editor/editor.worker?worker'
import jsonWorker from 'monaco-editor/esm/vs/language/json/json.worker?worker'
import cssWorker from 'monaco-editor/esm/vs/language/css/css.worker?worker'
import htmlWorker from 'monaco-editor/esm/vs/language/html/html.worker?worker'
import tsWorker from 'monaco-editor/esm/vs/language/typescript/ts.worker?worker'

// --- 2. 配置 Monaco Environment ---
// 这是让 Monaco 在 Vite/Electron 环境下正常工作的关键
self.MonacoEnvironment = {
  getWorker(_, label) {
    if (label === 'json') {
      return new jsonWorker()
    }
    if (label === 'css' || label === 'scss' || label === 'less') {
      return new cssWorker()
    }
    if (label === 'html' || label === 'handlebars' || label === 'razor') {
      return new htmlWorker()
    }
    if (label === 'typescript' || label === 'javascript') {
      return new tsWorker()
    }
    // 对于 XML, YAML, Plain Text 等，使用默认的 editorWorker
    return new editorWorker()
  }
}

// --- 3. 强制 loader 使用本地安装的 monaco，不再请求 CDN ---
loader.config({ monaco })

// --- Monaco Editor 配置 ---
const monacoOptions = {
  automaticLayout: true, // 自动调整大小
  formatOnType: true,
  formatOnPaste: true,
  minimap: { enabled: true }, // 代码缩略图
  fontSize: 14,
  scrollBeyondLastLine: false,
  wordWrap: 'on',
  theme: 'vs'
}
const languageOptions = [
  { id: 'plaintext', label: 'Plain Text' },
  { id: 'xml', label: 'XML' },
  { id: 'yaml', label: 'YAML' },
  { id: 'json', label: 'JSON' },
  { id: 'python', label: 'Python' },
  { id: 'cpp', label: 'C++' },
  { id: 'c', label: 'C' },
  { id: 'javascript', label: 'JavaScript' },
  { id: 'shell', label: 'Shell / Bash' },
  { id: 'markdown', label: 'Markdown' },
  { id: 'ini', label: 'INI / Config' },
  { id: 'sql', label: 'SQL' },
  { id: 'java', label: 'Java' }
]
function getLanguageLabel(id) {
  const option = languageOptions.find((opt) => opt.id === id)
  return option ? option.label : id.toUpperCase() // 找不到时回退到大写
}

// 根据文件名后缀判断语言
function detectLanguage(filename) {
  const ext = filename.split('.').pop().toLowerCase()
  const map = {
    js: 'javascript',
    py: 'python',
    sh: 'shell',
    launch: 'xml',
    xml: 'xml',
    urdf: 'xml',
    xacro: 'xml',
    world: 'xml',
    sdf: 'xml',
    yaml: 'yaml',
    yml: 'yaml',
    json: 'json',
    md: 'markdown',
    cpp: 'cpp',
    hpp: 'cpp',
    h: 'cpp',
    c: 'c',
    txt: 'plaintext',
    ini: 'ini',
    conf: 'ini'
  }
  return map[ext] || 'plaintext'
}

// --- 数据状态 ---
const sessions = ref([])
const activeSessionId = ref('')
const activeBackendIds = ref([]) // 存储所有当前在线的后端ID
const isAddMenuOpen = ref(false) // 控制下拉菜单的展开/收起
const isFileBrowserVisible = ref(false)
const tabsRef = ref(null)
let isClosingAlertShowing = false

// --- 计算属性 ---
const currentSession = computed(() => sessions.value.find((s) => s.id === activeSessionId.value))
const currentFile = computed(() => {
  if (!currentSession.value) return null
  return currentSession.value.files.find((f) => f.path === currentSession.value.activeFile)
})
const availableBackendsToAdd = computed(() => {
  return activeBackendIds.value.filter((id) => {
    return !sessions.value.some((s) => s.id === id)
  })
})

async function initData() {
  try {
    // 并行获取：
    // 1. savedState: 上次关闭时保存的文件列表
    // 2. activeIds: 当前主进程中已连接的后端 ID 列表
    const [savedState, activeIds] = await Promise.all([
      window.api.loadEditorState(),
      window.api.getSessions() // <--- 确保 preload 中暴露了这个 API
    ])
    activeBackendIds.value = activeIds

    console.log('[Editor] Initial Active Backends:', activeIds)
    const mergedMap = new Map()
    // A. 先处理持久化的会话
    if (Array.isArray(savedState)) {
      savedState.forEach((s) => {
        mergedMap.set(s.id, {
          id: s.id,
          label: s.label || s.id,
          activeFile: s.activeFile || '__add_tab__', // 默认激活添加页签
          // 如果 ID 在 activeIds 列表中，则认为在线
          isOnline: activeIds.includes(s.id),
          files: (s.files || []).map((f) => ({
            name: f.name,
            path: f.path,
            content: '',
            loading: true, // 初始设为 loading
            isDirty: false,
            language: f.language || detectLanguage(f.name)
          }))
        })
      })
    }
    // B. 处理虽然没有持久化文件，但当前是在线的后端
    activeIds.forEach((id) => {
      if (!mergedMap.has(id)) {
        mergedMap.set(id, {
          id: id,
          label: id, // 如果能从后端获取 alias 更好，这里暂用 ID
          activeFile: '__add_tab__',
          isOnline: true,
          files: []
        })
      } else {
        // 如果已存在，再次确认在线状态
        const s = mergedMap.get(id)
        s.isOnline = true
      }
    })
    sessions.value = Array.from(mergedMap.values())
    // C. 恢复选中状态
    if (sessions.value.length > 0 && !activeSessionId.value) {
      activeSessionId.value = sessions.value[0].id
    }
    // D. 对在线会话加载内容
    sessions.value.forEach((s) => {
      if (s.isOnline) {
        reloadSessionFiles(s)
      } else {
        // 如果离线，把所有文件的 loading 取消，否则会一直转圈
        s.files.forEach((f) => {
          f.loading = false
          f.content = '// Backend Offline'
        })
      }
    })
  } catch (error) {
    console.error('Failed to init editor data:', error)
  }
}

// 辅助：重新加载会话文件的内容
async function reloadSessionFiles(session) {
  for (const file of session.files) {
    // 复用读取逻辑
    file.loading = true
    try {
      const content = await window.api.fs.readFile(file.path, session.id)
      file.content = content
      file.originalContent = content
      file.isDirty = false
    } catch (e) {
      file.content = `// Load Failed: ${e.message}`
    } finally {
      // 简单的延时防止闪烁
      setTimeout(() => {
        file.loading = false
      }, 200)
    }
  }
}

function handleTabsWheel(e) {
  // 获取 Tabs 组件的根元素
  const tabsEl = tabsRef.value?.$el
  if (!tabsEl) return
  // 找到 Tab 头部 (包含标签页的那个条)
  const headerEl = tabsEl.querySelector('.el-tabs__header')
  // 找到 Tab 滚动容器
  const scrollContainer = tabsEl.querySelector('.el-tabs__nav-scroll')
  // 【核心修复逻辑】
  // 只有当鼠标位于“Tab 头部”且不在“内容区”时，才拦截并手动滚动
  // e.target 是鼠标当前悬停的元素
  if (headerEl && headerEl.contains(e.target)) {
    // 在头部区域：阻止默认垂直滚动，改为水平滚动 Tab
    e.preventDefault()
    if (scrollContainer) {
      scrollContainer.scrollLeft += e.deltaY
    }
  }
  // 否则（在文件浏览器或编辑器区域）：不做任何处理，
  // 允许浏览器执行默认的滚动行为（即文件列表滚动）
}

// --- 生命周期 ---
onMounted(() => {
  // 1. 监听打开文件请求
  window.api.onEditorOpenFile((_event, payload) => handleOpenFile(payload))
  // 2. 监听后端连接状态变化
  // 主进程会在连接成功/断开时发送此消息
  window.api.onBackendStatusChanged(({ id, status }) => {
    // 更新 activeBackendIds 列表
    if (status === 'connected') {
      if (!activeBackendIds.value.includes(id)) {
        activeBackendIds.value.push(id)
      }
    } else {
      activeBackendIds.value = activeBackendIds.value.filter((bid) => bid !== id)
    }
    // 更新已有会话的 isOnline 状态 (原有逻辑)
    const session = sessions.value.find((s) => s.id === id)
    if (session) {
      // 更新在线状态
      session.isOnline = status === 'connected'
      // 如果上线了，且之前文件处于 loading 状态（因为离线没法读），则尝试重载
      if (session.isOnline) {
        reloadSessionFiles(session)
      }
    } else if (status === 'connected') {
      // 如果是一个全新的连接（不在当前列表里），是否要自动加进来？
      // 通常不需要，除非用户主动打开了文件。这里保持逻辑简单，只更新已有的。
    }
  })
  // 3. 监听关闭检查
  window.api.onCheckUnsaved(async () => {
    if (isClosingAlertShowing) return
    // 检查是否有任何文件的 isDirty 为 true
    const hasUnsaved = sessions.value.some((session) => session.files.some((file) => file.isDirty))
    if (hasUnsaved) {
      // 加锁
      isClosingAlertShowing = true
      try {
        // 弹出确认框
        await ElMessageBox.confirm(
          '当前有未保存的文件修改。如果不保存直接关闭，修改将会丢失。',
          '确认关闭',
          {
            confirmButtonText: '丢弃修改并关闭',
            cancelButtonText: '取消',
            type: 'warning',
            distinguishCancelAndClose: true
          }
        )
        // 用户点击了“丢弃修改并关闭”
        // 1. 保存当前的 Tab 布局状态（下次打开还是这些文件，只是没保存的内容丢了）
        await saveState()
        // 2. 告诉主进程：允许关闭
        await window.api.respondClose(true)
        // eslint-disable-next-line no-unused-vars
      } catch (action) {
        // 用户点击了“取消”或关闭了对话框
        // 告诉主进程：取消关闭
        await window.api.respondClose(false)
      } finally {
        // 无论结果如何，解锁
        isClosingAlertShowing = false
      }
    } else {
      // 没有未保存的内容，直接保存状态并关闭
      await saveState()
      await window.api.respondClose(true)
    }
  })
  // 4. 初始化数据
  initData()
})

let saveTimer = null
watch(
  sessions,
  () => {
    if (saveTimer) clearTimeout(saveTimer)
    // 防抖：1秒后自动保存状态
    saveTimer = setTimeout(() => {
      saveState()
    }, 1000)
  },
  { deep: true }
)

// --- 核心逻辑 ---
async function handleOpenFile({ backendId, backendLabel, name, path }) {
  // 1. 查找 Session
  let session = sessions.value.find((s) => s.id === backendId)
  if (!session) {
    const newSession = {
      id: backendId,
      label: backendLabel,
      activeFile: '',
      files: []
    }
    sessions.value.push(newSession)
    session = sessions.value.find((s) => s.id === backendId) // 获取 Proxy
  }
  activeSessionId.value = backendId
  // 2. 查找 File
  let file = session.files.find((f) => f.path === path)
  if (file) {
    session.activeFile = path
    return
  }
  // 在切换到新文件之前，记录下“刚才在哪里”
  // 如果是从添加页签过来的，这里就是 '__add_tab__'
  // 如果是从其他文件过来的，这里就是那个文件的路径
  const previousActiveFile = session.activeFile
  const newFile = {
    name,
    path,
    content: '',
    originalContent: '',
    loading: true,
    isDirty: false,
    language: detectLanguage(name)
  }
  session.files.push(newFile)
  file = session.files.find((f) => f.path === path) // 获取 Proxy
  session.activeFile = path
  // 3. 异步读取
  file.loading = true
  try {
    const content = await window.api.fs.readFile(path, backendId)
    file.content = content
    // 保存原始内容作为比对基准
    file.originalContent = content
    // 刚加载完，肯定没修改
    file.isDirty = false
    // 只有成功时才关闭 loading
    setTimeout(() => {
      // 检查文件是否还在（防止读取期间用户已经手动关闭了）
      if (session.files.includes(file)) {
        file.loading = false
      }
    }, 200)
  } catch (error) {
    ElMessage.error(error.message)
    await removeFileTab(session, path)
    // 强制回退到之前的状态
    // 覆盖 removeFileTab 自动计算出的“相邻 Tab”
    session.activeFile = previousActiveFile
  }
}

function handleEmbeddedFileSelect(session, filePath) {
  const fileName = filePath.split('/').pop()
  handleOpenFile({
    backendId: session.id,
    backendLabel: session.label,
    name: fileName,
    path: filePath
  })
}

function handleFileBrowserSelection(filePath) {
  isFileBrowserVisible.value = false
  // 获取文件名
  // 注意：这里假设是 Linux 路径。如果跨平台可能需要简单处理
  const fileName = filePath.split('/').pop()
  // 直接复用已有的打开逻辑
  handleOpenFile({
    backendId: currentSession.value.id, // 使用当前会话 ID
    backendLabel: currentSession.value.label,
    name: fileName,
    path: filePath
  })
}

async function removeFileTab(session, path) {
  const file = session.files.find((f) => f.path === path)
  // 检查文件是否存在且有未保存修改
  if (file && file.isDirty) {
    try {
      await ElMessageBox.confirm(
        `文件 "${file.name}" 有未保存的修改。确定要关闭吗？`,
        '未保存修改',
        {
          confirmButtonText: '丢弃修改',
          cancelButtonText: '取消',
          type: 'warning'
        }
      )
      // 用户点击了“确认”，继续执行下面的移除逻辑（即丢弃修改）
    } catch {
      // 用户点击了“取消”，直接返回，不执行移除
      return
    }
  }
  // 移除逻辑
  const tabs = session.files
  let activeName = session.activeFile
  if (activeName === path) {
    tabs.forEach((tab, index) => {
      if (tab.path === path) {
        const nextTab = tabs[index + 1] || tabs[index - 1]
        if (nextTab) {
          activeName = nextTab.path
        } else {
          // 如果没有下一个/上一个文件了，跳转到添加页签
          activeName = '__add_tab__'
        }
      }
    })
  }
  session.activeFile = activeName
  session.files = tabs.filter((tab) => tab.path !== path)
  saveState()
}

function handleAddSession(backendId) {
  // 创建新会话对象
  const newSession = {
    id: backendId,
    label: backendId, // 暂时使用 ID 作为标签
    activeFile: '__add_tab__',
    isOnline: true,
    files: []
  }
  sessions.value.push(newSession)
  activeSessionId.value = backendId
  // 添加后自动收起菜单
  isAddMenuOpen.value = false
}

async function removeSessionTab(id) {
  // 检查文件是否存在且有未保存修改
  const session = sessions.value.find((s) => s.id === id)
  // 检查该会话下是否有任何脏文件
  if (session && session.files.some((f) => f.isDirty)) {
    try {
      await ElMessageBox.confirm(
        `会话 "${session.label}" 中有未保存的文件。关闭会话将丢弃这些修改。`,
        '确认关闭会话',
        { type: 'warning', confirmButtonText: '丢弃并关闭' }
      )
    } catch {
      return
    }
  }

  const idx = sessions.value.findIndex((s) => s.id === id)
  if (idx === -1) return

  sessions.value.splice(idx, 1)

  // 如果删除了当前激活的，切换到前一个
  if (activeSessionId.value === id) {
    if (sessions.value.length > 0) {
      activeSessionId.value = sessions.value[Math.max(0, idx - 1)].id
    } else {
      activeSessionId.value = ''
    }
  }
}

// --- 辅助函数：持久化当前编辑器状态 ---
async function saveState() {
  const stateToSave = sessions.value.map((s) => ({
    id: s.id,
    label: s.label,
    activeFile: s.activeFile,
    // 只保存文件路径和名字，不保存 content（内容由文件系统读取）
    files: s.files.map((f) => ({ name: f.name, path: f.path, language: f.language }))
  }))
  await window.api.saveEditorState(stateToSave)
}

async function handleSave() {
  if (!currentFile.value || !currentSession.value) return
  const session = currentSession.value
  const file = currentFile.value

  try {
    // 注意：Monaco 的 v-model 已经同步了 file.content
    await window.api.fs.writeFile(file.path, file.content, session.id)
    file.originalContent = file.content
    file.isDirty = false
    ElMessage.success('保存成功')
  } catch (error) {
    // 2. 捕获错误，检查是否为权限问题
    // 常见的权限错误关键词：EACCES, Permission denied
    const isPermissionError =
      error.message.includes('EACCES') || error.message.includes('Permission denied')
    if (isPermissionError) {
      let suggestedPassword = ''
      try {
        // 3. 获取所有保存的连接配置
        const savedBackends = await window.api.getConfig('saved_backends')
        if (Array.isArray(savedBackends)) {
          // 寻找匹配当前 session.id 的配置
          const matchedBackend = savedBackends.find((item) => {
            // 重构 ID 生成逻辑以进行比对 (逻辑需与主程序 generateIdFromSettings 一致)
            let itemId = ''
            if (item.settings.mode === 'local') {
              itemId = 'local'
            } else if (item.settings.mode === 'ssh') {
              itemId = `ssh_${item.settings.ssh.username}@${item.settings.ssh.host}`
            }
            return itemId === session.id
          })
          // 3. 如果找到了且是 SSH 模式，提取密码
          if (
            matchedBackend &&
            matchedBackend.settings.mode === 'ssh' &&
            matchedBackend.settings.ssh.password
          ) {
            suggestedPassword = matchedBackend.settings.ssh.password
          }
        }
      } catch (e) {
        console.warn('自动获取密码失败，需手动输入', e)
      }
      // 4. 弹出密码输入框
      const formState = reactive({ password: suggestedPassword })
      try {
        await ElMessageBox({
          title: '权限不足',
          // 自定义渲染内容，实现换行
          message: () =>
            h('div', { style: 'padding-top: 5px' }, [
              h('p', { style: 'line-height: 24px; margin-bottom: 10px' }, [
                `保存文件 "${file.name}" 需要管理员权限。`,
                h('br'), // 手动换行
                '输入 sudo 密码：'
              ]),
              // 嵌入 ElInput 组件，开启 show-password
              h(ElInput, {
                modelValue: formState.password,
                'onUpdate:modelValue': (val) => {
                  formState.password = val
                },
                type: 'password',
                showPassword: true, // 开启小眼睛
                placeholder: '请输入密码',
                autofocus: true,
                // 监听回车键
                onKeydown: (e) => {
                  if (e.key === 'Enter') {
                    e.preventDefault() // 防止重复触发
                    // 模拟点击弹窗上的“确认”按钮
                    // 注意：这是 Element Plus 内部 DOM 结构，通常很稳定
                    const confirmBtn = document.querySelector(
                      '.el-message-box__btns .el-button--primary'
                    )
                    if (confirmBtn) confirmBtn.click()
                  }
                }
              })
            ]),
          showCancelButton: true,
          confirmButtonText: '保存',
          cancelButtonText: '取消',
          closeOnClickModal: false,
          // 校验逻辑
          beforeClose: (action, instance, done) => {
            if (action === 'confirm') {
              if (formState.password) {
                done()
              } else {
                ElMessage.warning('密码不能为空')
              }
            } else {
              done()
            }
          }
        })
        // 用户点击确认后，执行保存
        const loading = ElMessage({
          type: 'loading',
          message: '正在保存...',
          duration: 0
        })
        try {
          // 使用 formState.password
          await window.api.fs.writeFileSudo(file.path, file.content, formState.password, session.id)
          file.originalContent = file.content
          file.isDirty = false
          ElMessage.success('保存成功')
        } catch (sudoError) {
          ElMessage.error('保存失败: ' + sudoError.message)
        } finally {
          loading.close()
        }
      } catch (promptError) {
        if (promptError === 'cancel') return
        console.error(promptError)
        ElMessage.error('发生错误: ' + (promptError.message || promptError))
      }
    } else {
      ElMessage.error('保存失败: ' + error.message)
    }
  }
}

function handleLanguageChange(lang) {
  if (currentFile.value) {
    currentFile.value.language = lang
  }
}
</script>

<style scoped>
.ide-layout {
  --bg-color: #ffffff;
  --sidebar-bg: #f5f7fa; /* 浅灰背景 */
  --sidebar-hover: #e6e8eb;
  --sidebar-active: #ffffff;
  --border-color: #dcdfe6;
  --text-primary: #303133;
  --text-secondary: #909399;
  --primary-color: #409eff;
  --header-height: 35px; /* 与 main process 中定义的 overlay 高度一致 */

  display: flex;
  height: 100vh;
  width: 100vw;
  background-color: var(--bg-color);
  color: var(--text-primary);
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
  overflow: hidden;
}

/* --- 左侧侧边栏 --- */
.sidebar {
  width: 220px;
  flex-shrink: 0;
  background-color: var(--sidebar-bg);
  display: flex;
  flex-direction: column;
  border-right: 1px solid var(--border-color);
  height: 100%;
}

.sidebar-title {
  height: var(--header-height); /* 固定高度 */
  display: flex;
  align-items: center;
  padding-left: 15px;
  font-size: 12px;
  text-transform: uppercase;
  font-weight: bold;
  color: var(--text-secondary);
  letter-spacing: 0.5px;

  /* 关键：设置为拖拽区域 */
  -webkit-app-region: drag;
  user-select: none;
}

.session-list {
  flex-grow: 1;
  overflow-y: auto;
  padding-top: 5px;
}

.session-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 8px 15px;
  cursor: pointer;
  font-size: 13px;
  border-left: 3px solid transparent;
  transition: background 0.2s;
  color: var(--text-primary);

  /* 列表项不可拖拽，否则无法点击 */
  -webkit-app-region: no-drag;
}

.session-item:hover {
  background-color: var(--sidebar-hover);
}
.session-item.active {
  background-color: var(--sidebar-active);
  color: var(--primary-color);
  border-left-color: var(--primary-color);
  font-weight: 500;
}

.session-info {
  display: flex;
  align-items: center;
  overflow: hidden;
}
.session-info .icon {
  margin-right: 8px;
  font-size: 16px;
}
.session-info .label {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
.close-btn {
  opacity: 0;
  font-size: 14px;
  padding: 2px;
  border-radius: 3px;
  color: var(--text-secondary);
}
.session-item:hover .close-btn {
  opacity: 1;
}
.close-btn:hover {
  background-color: #dedfe0;
  color: #f56c6c;
}
.add-session-group {
  border-top: 1px solid var(--border-color);
  margin-top: 5px;
}

.add-trigger {
  color: var(--text-secondary);
}

.add-trigger:hover {
  color: var(--primary-color);
  background-color: var(--sidebar-hover);
}

.arrow-icon {
  font-size: 12px;
  transition: transform 0.2s;
}
.arrow-icon.rotated {
  transform: rotate(90deg);
}
/* 下拉菜单容器 */
.add-dropdown {
  background-color: #eaecf0; /* 比侧边栏稍深一点，体现层级 */
  box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.05);
  padding: 5px 0;
}
.dropdown-item {
  display: flex;
  align-items: center;
  padding: 6px 15px 6px 25px; /* 增加左内边距体现缩进 */
  cursor: pointer;
  font-size: 12px;
  color: var(--text-primary);
  transition: background 0.2s;
}
.dropdown-item:hover {
  background-color: #dcdfe6;
}
.dropdown-item .label {
  flex-grow: 1;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  margin-right: 10px;
}
.dropdown-item .dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  margin-right: 8px;
  background-color: #67c23a; /* 绿色表示在线 */
}
.dropdown-item .add-icon {
  font-size: 14px;
  color: var(--text-secondary);
}
.dropdown-item:hover .add-icon {
  color: var(--primary-color);
}
/* 信息提示样式 */
.dropdown-info {
  padding: 10px 15px 10px 25px;
  font-size: 12px;
  color: #909399;
  display: flex;
  align-items: center;
  gap: 8px;
}
.dropdown-info.warning {
  color: #e6a23c;
}
/* --- 右侧主区域 --- */
.main-area {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  background-color: var(--bg-color);
  height: 100%;
  min-width: 0;
}
.editor-workspace {
  flex-grow: 1;
  display: flex;
  flex-direction: column;
  height: 100%;
  position: relative;
}

/* --- Tabs 区域 --- */
.tabs-wrapper {
  height: 100%;
  width: 100%;
  display: flex; /* 使用 flex 布局 */
  flex-direction: column;
}
.file-tabs {
  background-color: var(--sidebar-bg); /* 顶部背景统一为浅灰 */
  height: 100%;
  display: flex;
  flex-direction: column;
}
:deep(.el-tabs--card > .el-tabs__header) {
  height: auto;
}
:deep(.el-tabs__header) {
  height: auto;
  margin: 0;
  background-color: var(--sidebar-bg);
  border-bottom: 1px solid var(--border-color);
  flex-shrink: 0;
  /* 关键：Tabs 头部空白处可以拖拽 */
  -webkit-app-region: drag;
  /* 为右侧的 Window Controls 留出空间 (Electron Overlay 在 Windows 上约 130px) */
  padding-right: 140px;
}

/* 单个 Tab 项 */
:deep(.el-tabs__item) {
  height: var(--header-height);
  line-height: var(--header-height);
  border-radius: 0 !important;
  border-top: none !important;
  border-left: none !important; /* 去除默认边框，更现代 */
  border-right: 1px solid var(--border-color) !important;
  font-size: 13px;
  color: var(--text-secondary);
  /* Tab 本身不可拖拽，否则无法点击 */
  -webkit-app-region: no-drag;
}
/* --- 修复 Tab 加号按钮 --- */
:deep(.el-tabs__new-tab) {
  -webkit-app-region: no-drag !important;
  margin-right: 10px;
  cursor: pointer;
}

/* 嵌入式文件浏览器的容器样式 */
.embedded-browser-wrapper {
  height: 100%;
  width: 100%;
  display: flex;
  flex-grow: 1;
}

:deep(.el-tabs__item.is-active) {
  background-color: #ffffff; /* 激活变为白色，与下方编辑器融为一体 */
  border-bottom-color: transparent !important;
  color: var(--primary-color);
  font-weight: 500;
}
:deep(.el-tabs__nav) {
  border: none !important;
}

:deep(.el-tabs__content) {
  padding: 0;
  flex-grow: 1;
  height: 0;
  background-color: #ffffff;
}

:deep(.el-tab-pane) {
  display: flex;
  height: 100%;
  width: 100%;
}

/* --- Monaco 容器 --- */
.monaco-container {
  height: 100%;
  width: 100%;
  background-color: #ffffff; /* 浅色背景 */
  position: relative;
  overflow: hidden;
}

.monaco-editor-instance {
  height: 100%;
  width: 100%;
}

.empty-state {
  flex-grow: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  background-color: #ffffff;
  color: var(--text-secondary);
}

.dirty-file {
  color: #e6a23c;
}

/* --- 底部状态栏 --- */
.status-bar {
  height: 24px;
  background-color: var(--primary-color); /* 使用 Element Plus 蓝色 */
  color: white;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0 10px;
  font-size: 12px;
  flex-shrink: 0;
  user-select: none;
}
.right {
  display: flex;
  gap: 15px;
}
.dirty-badge {
  font-weight: bold;
  background-color: rgba(255, 255, 255, 0.2);
  padding: 0 4px;
  border-radius: 2px;
}
.language-selector {
  cursor: pointer;
  padding: 2px 6px;
  border-radius: 0px;
  transition: background-color 0.2s;
  color: white; /* 继承状态栏文字颜色 */
  font-weight: 500;
  outline: none !important;
  user-select: none;
}
.language-selector:hover {
  background-color: rgba(255, 255, 255, 0.2);
}
/* 下拉菜单选中项高亮 (需要用 :global 或者放到非 scoped 样式中，因为 dropdown 是挂载在 body 上的) */
/* 但如果是 scoped，我们可以尝试深度选择器，或者 Element Plus 自带 active class */
.lang-dropdown .el-dropdown-menu__item.is-selected {
  color: var(--primary-color);
  background-color: #ecf5ff;
  font-weight: bold;
}
/* 限制下拉菜单高度，防止太长 */
.lang-dropdown {
  max-height: 300px;
  overflow-y: auto;
}
</style>
