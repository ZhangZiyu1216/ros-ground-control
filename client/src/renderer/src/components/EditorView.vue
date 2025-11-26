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
            <el-icon class="icon" :class="getSessionStatus(session.id)"><Monitor /></el-icon>
            <span class="label" :title="session.label">{{ session.label }}</span>
          </div>
          <el-icon class="close-btn" @click.stop="removeSessionTab(session.id)"><Close /></el-icon>
        </div>

        <!-- 添加会话组 -->
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
            <el-icon class="arrow-icon" :class="{ rotated: isAddMenuOpen }"><ArrowRight /></el-icon>
          </div>

          <!-- 下拉菜单内容 -->
          <div v-show="isAddMenuOpen" class="add-dropdown">
            <!-- 情况 1: 有可添加的后端 -->
            <template v-if="availableBackendsToAdd.length > 0">
              <div
                v-for="item in availableBackendsToAdd"
                :key="item.settings.id"
                class="dropdown-item"
                @click="handleAddSession(item.settings.id)"
              >
                <span class="dot" :class="getSessionStatus(item.settings.id)"></span>
                <span class="label" :title="item.settings.ip">
                  {{ item.name || item.settings.ip }}
                </span>
                <el-icon class="add-icon"><Plus /></el-icon>
              </div>
            </template>

            <!-- 情况 2: 所有后端都已添加 -->
            <div v-else-if="savedBackendsConfig.length > 0" class="dropdown-info">
              <el-icon><Check /></el-icon>
              <span>所有会话已添加</span>
            </div>

            <!-- 情况 3: 没有任何后端配置 -->
            <div v-else class="dropdown-info warning">
              <el-icon><Warning /></el-icon>
              <span>未找到连接配置</span>
            </div>
          </div>
        </div>
      </div>
    </aside>

    <!-- 2. 右侧主区域：文件编辑 -->
    <main class="main-area">
      <!-- 情况 A: 有选中的会话 -->
      <div v-if="currentSession" class="editor-workspace">
        <!-- [关键修复] 只有状态严格为 'ready' 时才显示编辑器，其他状态(failed/setting_up/disconnected)全部显示离线页 -->
        <template v-if="getSessionStatus(currentSession.id) === 'ready'">
          <div class="tabs-wrapper" @wheel="handleTabsWheel">
            <el-tabs
              ref="tabsRef"
              v-model="currentSession.activeFile"
              type="card"
              class="file-tabs"
              @tab-remove="(path) => removeFileTab(currentSession, path)"
            >
              <!-- 1. 已打开的文件 -->
              <el-tab-pane
                v-for="file in currentSession.files"
                :key="file.path"
                :name="file.path"
                closable
              >
                <template #label>
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

              <!-- 2. 添加页签 (文件浏览器) -->
              <el-tab-pane name="__add_tab__" :closable="false">
                <template #label>
                  <span class="add-tab-btn">
                    <el-icon><Plus /></el-icon>
                  </span>
                </template>

                <div class="embedded-browser-wrapper">
                  <FileBrowser
                    :backend-id="currentSession.id"
                    :initial-path="'/'"
                    :show-close="false"
                    @file-selected="(path) => handleEmbeddedFileSelect(currentSession, path)"
                  />
                </div>
              </el-tab-pane>
            </el-tabs>
          </div>
        </template>

        <!-- 离线/连接中/失败 状态 -->
        <div v-else class="empty-state offline-state">
          <!-- 根据具体状态显示不同图标 -->
          <template v-if="getSessionStatus(currentSession.id) === 'setting_up'">
            <el-icon :size="50" class="is-loading" color="#E6A23C"><Loading /></el-icon>
            <h3>正在连接...</h3>
            <p>正在尝试连接后端 {{ currentSession.label }}</p>
          </template>

          <template v-else>
            <el-icon :size="50" color="#f56c6c"><Connection /></el-icon>
            <h3>连接不可用</h3>
            <p>
              无法连接到后端 {{ currentSession.label }} ({{ getSessionStatus(currentSession.id) }})
            </p>
            <el-button
              type="primary"
              :icon="RefreshRight"
              style="margin-top: 20px"
              @click="retryConnection"
            >
              重试连接
            </el-button>
          </template>
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
          <span v-if="currentSession && currentSession.activeFile === '__add_tab__'">
            {{ currentSession.label }} &nbsp;>&nbsp; 文件浏览器
          </span>
          <span v-else-if="currentSession && currentFile">
            {{ currentSession.label }} &nbsp;>&nbsp; {{ currentFile.path }}
          </span>
        </div>
        <div class="right">
          <el-dropdown
            v-if="currentFile"
            trigger="hover"
            placement="top"
            @command="handleLanguageChange"
          >
            <span class="language-selector">
              {{ getLanguageLabel(currentFile.language) }}
            </span>
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
        :show-close="false"
        :hide-footer="true"
        @file-selected="handleFileBrowserSelection"
        @cancel="isFileBrowserVisible = false"
      />
    </el-dialog>
  </div>
</template>

<script setup>
// #region 1. Imports
import { ref, onMounted, computed, watch, toRaw } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import {
  Monitor,
  Close,
  Plus,
  Connection,
  ArrowRight,
  Check,
  Warning,
  RefreshRight,
  Loading
} from '@element-plus/icons-vue' // [新增] RefreshRight
import { VueMonacoEditor, loader } from '@guolao/vue-monaco-editor'
import FileBrowser from '../components/FileBrowser.vue'
import { useRobotStore } from '../store/robot'

// Monaco Setup
import * as monaco from 'monaco-editor'
import editorWorker from 'monaco-editor/esm/vs/editor/editor.worker?worker'
import jsonWorker from 'monaco-editor/esm/vs/language/json/json.worker?worker'
import cssWorker from 'monaco-editor/esm/vs/language/css/css.worker?worker'
import htmlWorker from 'monaco-editor/esm/vs/language/html/html.worker?worker'
import tsWorker from 'monaco-editor/esm/vs/language/typescript/ts.worker?worker'

self.MonacoEnvironment = {
  getWorker(_, label) {
    if (label === 'json') return new jsonWorker()
    if (label === 'css' || label === 'scss' || label === 'less') return new cssWorker()
    if (label === 'html' || label === 'handlebars' || label === 'razor') return new htmlWorker()
    if (label === 'typescript' || label === 'javascript') return new tsWorker()
    return new editorWorker()
  }
}
loader.config({ monaco })
// #endregion

// #region 2. Config & Helpers
const robotStore = useRobotStore()

const monacoOptions = {
  automaticLayout: true,
  formatOnType: true,
  formatOnPaste: true,
  minimap: { enabled: true },
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

function detectLanguage(filename) {
  const ext = filename.split('.').pop().toLowerCase()
  const map = {
    js: 'javascript',
    py: 'python',
    sh: 'shell',
    launch: 'xml',
    xml: 'xml',
    yaml: 'yaml',
    yml: 'yaml',
    json: 'json',
    md: 'markdown',
    cpp: 'cpp',
    h: 'cpp',
    txt: 'plaintext',
    ini: 'ini',
    conf: 'ini'
  }
  return map[ext] || 'plaintext'
}

function getLanguageLabel(id) {
  const option = languageOptions.find((opt) => opt.id === id)
  return option ? option.label : id.toUpperCase()
}

function getSessionStatus(backendId) {
  // 直接从 Store 中读取，Store 会自动通过心跳更新状态
  const client = robotStore.clients[backendId]
  return client ? client.status : 'disconnected'
}
// #endregion

// #region 3. State
const sessions = ref([])
const activeSessionId = ref('')
const isAddMenuOpen = ref(false)
const isFileBrowserVisible = ref(false)
const tabsRef = ref(null)
const savedBackendsConfig = ref([])
let isClosingAlertShowing = false

const currentSession = computed(() => sessions.value.find((s) => s.id === activeSessionId.value))
const currentFile = computed(() =>
  currentSession.value?.files.find((f) => f.path === currentSession.value.activeFile)
)

const availableBackendsToAdd = computed(() => {
  return savedBackendsConfig.value.filter(
    (b) => !sessions.value.some((s) => s.id === b.settings.id)
  )
})
// #endregion

// #region 4. Initialization & Connection Logic

// 建立连接 (复用 robotStore)
async function ensureConnection(backendId, settings = null) {
  // 1. 如果 Store 中已经 Ready，直接返回
  if (robotStore.clients[backendId] && robotStore.clients[backendId].status === 'ready') {
    return true
  }

  // 2. 查找配置
  const config = savedBackendsConfig.value.find((b) => b.settings.id === backendId)
  if (!config) {
    console.error('Config not found for UUID:', backendId)
    return false
  }
  if (settings) {
    robotStore.initClientPlaceholder(settings)
  } else {
    // 兜底：用找到的配置初始化
    robotStore.initClientPlaceholder(config.settings)
  }

  try {
    console.log(`[Editor] Establishing connection to ${config.settings.ip}...`)
    await robotStore.addConnection(config.settings)
    return true
  } catch (e) {
    console.warn(`[Editor] Connection failed for ${backendId}:`, e.message)
    return false
  }
}

async function initData() {
  try {
    // 1. 加载配置
    const configs = await window.api.getConfig('saved_backends')
    savedBackendsConfig.value = Array.isArray(configs) ? configs : []

    // 2. 加载编辑器状态
    const savedState = await window.api.loadEditorState()

    if (Array.isArray(savedState) && savedState.length > 0) {
      for (const s of savedState) {
        const configItem = savedBackendsConfig.value.find((b) => b.settings.id === s.id)
        if (configItem) {
          robotStore.initClientPlaceholder(configItem.settings)
        }

        // 创建基础对象
        const sessionObj = {
          id: s.id,
          label: s.label,
          activeFile: s.activeFile || '__add_tab__',
          isOnline: false, // 默认为 false
          files: (s.files || []).map((f) => ({
            name: f.name,
            path: f.path,
            content: '',
            originalContent: '',
            loading: false,
            isDirty: false,
            language: f.language || detectLanguage(f.name)
          }))
        }
        sessions.value.push(sessionObj)

        // 然后从数组中获取响应式 Proxy 对象进行后续操作
        // 这样后续修改 isOnline 时，视图才会更新
        const sessionProxy = sessions.value.find((x) => x.id === s.id)

        // 异步建立连接
        ensureConnection(s.id).then((success) => {
          if (sessionProxy) {
            sessionProxy.isOnline = success
            if (success) reloadSessionFiles(sessionProxy)
          }
        })
      }

      // 恢复选中
      if (!activeSessionId.value) {
        activeSessionId.value = sessions.value[0].id
      }
    }
  } catch (e) {
    console.error('Init failed:', e)
  }
}

// 重新加载文件内容
async function reloadSessionFiles(session) {
  // session 应该是从 sessions.value 中取出的 Proxy
  for (const file of session.files) {
    if (!file.path) continue

    file.loading = true // 这里 file 是 Proxy 的属性，通常是响应式的
    try {
      const content = await robotStore.fsReadFile(session.id, file.path)
      file.content = content
      file.originalContent = content
      file.isDirty = false
    } catch (e) {
      file.content = `// Read Error: ${e.message}`
    } finally {
      // 稍微延时，避免 Loading 闪烁太快
      setTimeout(() => {
        file.loading = false
      }, 300)
    }
  }
}

// 手动重试连接 (供 UI 调用)
async function retryConnection() {
  if (!currentSession.value) return
  const session = currentSession.value
  const success = await ensureConnection(session.id)
  session.isOnline = success
  if (success) {
    ElMessage.success('连接已恢复')
    reloadSessionFiles(session)
  } else {
    ElMessage.error('连接仍然失败')
  }
}
// #endregion

// #region 5. Event Handlers

async function handleOpenFile({ backendId, backendLabel, name, path }) {
  // 1. 确保配置已加载
  if (savedBackendsConfig.value.length === 0) {
    const configs = await window.api.getConfig('saved_backends')
    savedBackendsConfig.value = Array.isArray(configs) ? configs : []
  }
  // [新增] 查找配置，为了获取完整的 settings 对象
  const configItem = savedBackendsConfig.value.find((b) => b.settings.id === backendId)
  const settings = configItem ? configItem.settings : { id: backendId, ip: 'unknown' }

  // 2. 查找或创建 Session
  const isStoreReady = robotStore.clients[backendId]?.status === 'ready'
  let session = sessions.value.find((s) => s.id === backendId)

  if (!session) {
    robotStore.initClientPlaceholder(settings)
    // 先 push
    sessions.value.push({
      id: backendId,
      label: backendLabel || backendId,
      activeFile: '',
      isOnline: isStoreReady,
      files: []
    })
    // 再 find (获取响应式 Proxy)
    session = sessions.value.find((s) => s.id === backendId)

    // 异步连接
    ensureConnection(backendId, settings).then((ok) => {
      if (session) session.isOnline = ok
    })
  }

  activeSessionId.value = backendId

  // 3. 查找 File Tab
  let file = session.files.find((f) => f.path === path)
  if (file) {
    session.activeFile = path
    return
  }

  // 4. 创建新文件对象 (Plain Object)
  const newFileObj = {
    name,
    path,
    content: '',
    originalContent: '',
    loading: true,
    isDirty: false,
    language: detectLanguage(name)
  }

  // 推入数组
  session.files.push(newFileObj)
  session.activeFile = path

  // 【关键修复】从数组中重新获取该文件，得到响应式 Proxy
  const targetFile = session.files.find((f) => f.path === path)

  // 5. 读取内容 (操作 Proxy)
  // 检查连接状态 (注意：ensureConnection 是异步的，这里可能还没连上)
  // 我们再次快速检查一下
  const isConnected = await ensureConnection(backendId)
  if (session) session.isOnline = isConnected

  if (isConnected) {
    try {
      const content = await robotStore.fsReadFile(backendId, path)
      targetFile.content = content
      targetFile.originalContent = content
    } catch (e) {
      targetFile.content = `// Error: ${e.message}`
    } finally {
      targetFile.loading = false // 修改 Proxy，触发 UI 更新
    }
  } else {
    targetFile.content = '// Offline'
    targetFile.loading = false
  }
}

// 保存文件
async function handleSave() {
  if (!currentFile.value || !currentSession.value) return
  const session = currentSession.value
  const file = currentFile.value

  try {
    await robotStore.fsWriteFile(session.id, file.path, file.content)
    file.originalContent = file.content
    file.isDirty = false
    ElMessage.success('保存成功')
  } catch (error) {
    ElMessage.error(`保存失败: ${error.message}`)
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
  const fileName = filePath.split('/').pop()
  handleOpenFile({
    backendId: currentSession.value.id,
    backendLabel: currentSession.value.label,
    name: fileName,
    path: filePath
  })
}

async function removeFileTab(session, path) {
  const file = session.files.find((f) => f.path === path)
  if (file && file.isDirty) {
    try {
      await ElMessageBox.confirm(`文件 "${file.name}" 有未保存的修改。`, '确认', {
        type: 'warning'
      })
    } catch {
      return
    }
  }

  // [关键修复] Tab 移除逻辑
  const tabs = session.files
  let activeName = session.activeFile
  if (activeName === path) {
    tabs.forEach((tab, index) => {
      if (tab.path === path) {
        const nextTab = tabs[index + 1] || tabs[index - 1]
        if (nextTab) {
          activeName = nextTab.path
        } else {
          activeName = '__add_tab__'
        }
      }
    })
  }

  session.activeFile = activeName
  // 使用 filter 产生新数组赋值
  session.files = tabs.filter((t) => t.path !== path)
  saveState()
}

async function removeSessionTab(id) {
  const session = sessions.value.find((s) => s.id === id)
  if (session && session.files.some((f) => f.isDirty)) {
    try {
      await ElMessageBox.confirm(`会话 "${session.label}" 有未保存文件。`, '确认', {
        type: 'warning'
      })
    } catch {
      return
    }
  }

  const idx = sessions.value.findIndex((s) => s.id === id)
  if (idx === -1) return

  sessions.value.splice(idx, 1)
  if (activeSessionId.value === id) {
    activeSessionId.value = sessions.value[Math.max(0, idx - 1)]?.id || ''
  }
  saveState()
}

function handleAddSession(backendId) {
  const config = savedBackendsConfig.value.find((b) => b.settings.id === backendId)
  const label = config ? config.name || config.settings.ip : backendId

  // 先 Push
  sessions.value.push({
    id: backendId,
    label: label,
    activeFile: '__add_tab__',
    isOnline: false,
    files: []
  })

  // 获取 Proxy
  const newSession = sessions.value.find((s) => s.id === backendId)
  activeSessionId.value = backendId
  isAddMenuOpen.value = false

  // 连接
  ensureConnection(backendId).then((ok) => {
    if (newSession) newSession.isOnline = ok
  })
}

function handleLanguageChange(lang) {
  if (currentFile.value) currentFile.value.language = lang
}

// ... State Persistence ...
async function saveState() {
  const stateToSave = sessions.value.map((s) => ({
    id: s.id,
    label: s.label,
    activeFile: s.activeFile,
    files: s.files.map((f) => ({ name: f.name, path: f.path, language: f.language }))
  }))
  await window.api.saveEditorState(toRaw(stateToSave))
}

// #endregion

// #region 6. Lifecycle
onMounted(async () => {
  // 1. 注册监听
  window.api.onEditorOpenFile((_event, payload) => {
    if (!payload || !payload.backendId) return
    handleOpenFile(payload)
  })

  window.api.onCheckUnsaved(async () => {
    if (isClosingAlertShowing) return
    const hasUnsaved = sessions.value.some((s) => s.files.some((f) => f.isDirty))
    if (hasUnsaved) {
      isClosingAlertShowing = true
      try {
        await ElMessageBox.confirm('有未保存的修改，确定丢弃并关闭？', '关闭确认', {
          type: 'warning'
        })
        await saveState()
        window.api.respondClose(true)
      } catch {
        window.api.respondClose(false)
      } finally {
        isClosingAlertShowing = false
      }
    } else {
      await saveState()
      window.api.respondClose(true)
    }
  })

  // 监听主窗口发来的断开信号
  window.api.onBackendStatusChanged(({ id, status }) => {
    // [新增] 处理连接成功信号 (可选，用于 UI 同步)
    if (status === 'connected') {
      const session = sessions.value.find((s) => s.id === id)
      // 如果编辑器里有这个会话，但显示离线，可以尝试自动重连一下
      if (session && !session.isOnline) {
        console.log(`[Editor] Main window connected to ${id}, retrying editor connection...`)
        ensureConnection(id).then((ok) => (session.isOnline = ok))
      }
    }

    // 处理断开信号 (原有逻辑)
    if (status === 'disconnected') {
      robotStore.removeConnection(id)
      const session = sessions.value.find((s) => s.id === id)
      if (session) session.isOnline = false
    }
  })

  // 2. 初始化数据 (等待它完成)
  await initData()

  // 3. 标记就绪并处理队列
  window.api.editorIsReady()
})

let saveTimer = null
watch(
  sessions,
  () => {
    if (saveTimer) clearTimeout(saveTimer)
    saveTimer = setTimeout(() => saveState(), 1000)
  },
  { deep: true }
)

function handleTabsWheel(e) {
  const tabsEl = tabsRef.value?.$el
  if (!tabsEl) return
  const headerEl = tabsEl.querySelector('.el-tabs__header')
  const scrollContainer = tabsEl.querySelector('.el-tabs__nav-scroll')
  if (headerEl && headerEl.contains(e.target)) {
    e.preventDefault()
    if (scrollContainer) scrollContainer.scrollLeft += e.deltaY
  }
}
// #endregion
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
/* 状态颜色映射 (与 MainView 保持一致) */
.session-info .el-icon.ready {
  color: #409eff;
}
.session-info .el-icon.setting_up {
  color: #e6a23c;
}
.session-info .el-icon.failed {
  color: #f56c6c;
}
.session-info .el-icon.disconnected {
  color: #909399;
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
.dropdown-item .dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  margin-right: 8px;
  background-color: #dcdfe6; /* 默认未知 */
}
.dropdown-item .dot.ready {
  background-color: #409eff;
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
