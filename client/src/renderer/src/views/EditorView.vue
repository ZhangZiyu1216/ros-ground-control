<template>
  <div class="ide-layout" tabindex="0" @keydown.ctrl.s.prevent="handleSave">
    <!-- [新增] 自定义窗口控制按钮 (绝对定位在右上角) -->
    <div class="window-controls-tab"></div>
    <div class="window-controls windows-style no-drag">
      <div class="win-ctrl-btn min" @click="minimizeWindow">
        <el-icon><Minus /></el-icon>
      </div>
      <div class="win-ctrl-btn max" @click="maximizeWindow">
        <el-icon><FullScreen /></el-icon>
      </div>
      <div class="win-ctrl-btn close" @click="closeWindow">
        <el-icon><Close /></el-icon>
      </div>
    </div>

    <!-- 1. 左侧侧边栏：会话列表 -->
    <aside class="sidebar">
      <!-- 头部：可拖拽区域 -->
      <div class="sidebar-header drag-region">
        <span class="header-title">文件编辑器</span>
      </div>

      <div class="session-list no-drag">
        <div class="section-title">会话列表</div>

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
          <div class="close-btn-wrapper" @click.stop="removeSessionTab(session.id)">
            <el-icon><Close /></el-icon>
          </div>
        </div>

        <!-- 添加会话组 -->
        <div class="add-session-group">
          <div
            class="session-item add-trigger"
            :class="{ 'is-open': isAddMenuOpen }"
            @click="isAddMenuOpen = !isAddMenuOpen"
          >
            <div class="session-info">
              <el-icon class="arrow-icon" :class="{ rotated: isAddMenuOpen }"
                ><ArrowRight
              /></el-icon>
              <span class="label">添加会话...</span>
            </div>
          </div>

          <!-- 下拉菜单 -->
          <transition name="el-zoom-in-top">
            <div v-show="isAddMenuOpen" class="add-dropdown">
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
              <div v-else-if="savedBackendsConfig.length > 0" class="dropdown-info">
                <el-icon><Check /></el-icon><span>所有连接已添加</span>
              </div>
              <div v-else class="dropdown-info warning">
                <el-icon><Warning /></el-icon><span>无可用配置</span>
              </div>
            </div>
          </transition>
        </div>
      </div>
    </aside>

    <!-- 2. 右侧主区域 -->
    <main class="main-area">
      <!-- 情况 A: 有选中的会话 -->
      <div v-if="currentSession" class="editor-workspace">
        <template v-if="getSessionStatus(currentSession.id) === 'ready'">
          <div class="tabs-wrapper" @wheel="handleTabsWheel">
            <!-- el-tabs 头部作为拖拽区域，但内部 tab item 不可拖拽 -->
            <el-tabs
              ref="tabsRef"
              v-model="currentSession.activeFile"
              type="card"
              class="file-tabs drag-region"
              @tab-remove="(path) => removeFileTab(currentSession, path)"
            >
              <el-tab-pane
                v-for="file in currentSession.files"
                :key="file.path"
                :name="file.path"
                closable
              >
                <template #label>
                  <span class="tab-label-content no-drag" :class="{ 'dirty-file': file.isDirty }">
                    <span class="file-name">{{ file.name }}</span>
                    <span v-if="file.isDirty" class="dirty-indicator">●</span>
                    <el-icon v-if="file.isSaving" class="is-loading"><Loading /></el-icon>
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

              <!-- 添加页签 -->
              <el-tab-pane name="__add_tab__" :closable="false">
                <template #label>
                  <span class="add-tab-btn no-drag">
                    <el-icon><Plus /></el-icon> 文件浏览器
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

        <!-- 离线/连接状态 -->
        <div v-else class="empty-state offline-state">
          <div class="state-card">
            <template v-if="getSessionStatus(currentSession.id) === 'setting_up'">
              <div class="spinner-ring"></div>
              <h3>连接中...</h3>
              <p>正在连接到 {{ currentSession.label }}</p>
            </template>
            <template v-else>
              <el-icon :size="48" color="#f56c6c"><Connection /></el-icon>
              <h3>未连接</h3>
              <p>目标主机 {{ currentSession.label }} 不可达。</p>
            </template>
          </div>
        </div>
      </div>

      <!-- 情况 B: 无会话 -->
      <div v-else class="empty-state">
        <div class="empty-content">
          <el-icon :size="64" color="var(--text-disabled)"><Monitor /></el-icon>
          <p>选择会话以编辑文件</p>
        </div>
      </div>

      <!-- 底部状态栏 -->
      <div class="status-bar">
        <div class="bar-left">
          <div class="bar-item highlight">
            <el-icon><Connection /></el-icon>
            <span>{{ currentSession ? currentSession.label : '无会话' }}</span>
          </div>
          <div v-if="currentFile" class="bar-item path-item">
            {{ currentFile.path }}
          </div>
        </div>
        <div class="bar-right">
          <div v-if="currentFile?.isDirty" class="bar-item warning-bg">未保存</div>
          <el-dropdown
            v-if="currentFile"
            trigger="click"
            placement="top"
            @command="handleLanguageChange"
          >
            <div class="bar-item action">
              {{ getLanguageLabel(currentFile.language) }}
            </div>
            <template #dropdown>
              <el-dropdown-menu class="lang-dropdown">
                <el-dropdown-item v-for="opt in languageOptions" :key="opt.id" :command="opt.id">
                  {{ opt.label }}
                </el-dropdown-item>
              </el-dropdown-menu>
            </template>
          </el-dropdown>
        </div>
      </div>
    </main>

    <!-- 文件弹窗 -->
    <el-dialog
      v-model="isFileBrowserVisible"
      title="Open File"
      width="70%"
      destroy-on-close
      align-center
    >
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
  Loading,
  Minus,
  FullScreen
} from '@element-plus/icons-vue'
import { VueMonacoEditor, loader } from '@guolao/vue-monaco-editor'
import FileBrowser from '../components/FileBrowser.vue'
import { useRobotStore } from '../store/robot'
import { createApi } from '../api/request' // 用于手动注入 Token

// Monaco Worker Setup (标准配置)
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

const minimizeWindow = () => window.electronWindow?.minimize()
const maximizeWindow = () => window.electronWindow?.toggleMaximize()
const closeWindow = () => window.electronWindow?.close()
// #endregion

// #region 2. Config & Constants
const robotStore = useRobotStore()

const monacoOptions = {
  automaticLayout: true,
  formatOnType: true,
  formatOnPaste: true,
  minimap: { enabled: true },
  fontSize: 14,
  theme: 'vs'
}

const languageOptions = [
  { id: 'plaintext', label: 'Plain Text' },
  { id: 'xml', label: 'XML' },
  { id: 'yaml', label: 'YAML' },
  { id: 'json', label: 'JSON' },
  { id: 'python', label: 'Python' },
  { id: 'cpp', label: 'C++' },
  { id: 'javascript', label: 'JavaScript' },
  { id: 'shell', label: 'Shell / Bash' },
  { id: 'markdown', label: 'Markdown' },
  { id: 'ini', label: 'INI / Config' }
]

function detectLanguage(filename) {
  if (!filename) return 'plaintext'
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

// 获取会话状态
function getSessionStatus(backendId) {
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

// #region 4. Connection Logic (Fixed)

// 纯 HTTP 连接器 (复用 Token)
function applyTokenToSession(backendId, token, settings) {
  // 1. 查找或创建 Session UI
  let session = sessions.value.find((s) => s.id === backendId)
  if (!session) {
    // 如果是新发现的会话（之前没保存过），加进来
    const label = settings.name || settings.ip
    sessions.value.push({
      id: backendId,
      label: label,
      activeFile: '__add_tab__',
      isOnline: false,
      files: []
    })
    session = sessions.value.find((s) => s.id === backendId)
  }

  // 2. 初始化 Store Client (仅 HTTP)
  if (!settings.id) settings.id = backendId

  if (!robotStore.clients[backendId]) {
    robotStore.initClientPlaceholder(settings)
  }
  const client = robotStore.clients[backendId]

  if (!client) {
    console.error(`[Editor] Failed to init client for ${backendId}`)
    return
  }

  // 3. 注入 Token
  if (token) {
    const baseUrl = `http://${settings.ip}:${settings.port || 8080}/api`
    const api = createApi(baseUrl, {
      getToken: () => token,
      // 编辑器里如果不操作 sudo，其实可以暂不传，或者也通过 IPC 同步 sudoPassword
      getSudoPassword: () => robotStore.sudoPassword,
      onSessionExpired: () => {
        session.isOnline = false
        ElMessage.warning(`会话 ${session.label} 已过期`)
      }
    })

    client.api = api
    client.token = token
    client.status = 'ready'
    client.ip = settings.ip
    client.id = backendId // 确保 ID 一致

    // 更新 UI 状态
    const wasOffline = !session.isOnline
    session.isOnline = true

    // 如果之前是离线状态，现在连上了，立即刷新文件
    if (wasOffline) {
      console.log(`[Editor] Session ${backendId} is now ONLINE. Reloading files...`)
      reloadSessionFiles(session)
    }
  } else {
    // 收到 Token 为 null/undefined，说明主窗口断开了
    session.isOnline = false
    client.status = 'disconnected'
  }
}

// 刷新文件内容
async function reloadSessionFiles(session) {
  for (const file of session.files) {
    if (!file.path) continue
    file.loading = true
    try {
      const content = await robotStore.fsReadFile(session.id, file.path)
      file.content = content
      file.originalContent = content
      file.isDirty = false
    } catch (e) {
      file.content = `// 无法读取文件: ${e.message}\n// 请检查网络或确认文件是否存在。`
    } finally {
      setTimeout(() => {
        file.loading = false
      }, 300)
    }
  }
}

/**
 * 确保连接可用
 * @param {string} backendId - UUID
 * @param {Object} options - { token, sudoPassword, settings, forceWebSocket }
 * @returns {Promise<boolean>}
 */
async function ensureConnection(backendId, options = {}) {
  // 1. 如果 Store 中已经是 ready，直接成功
  if (robotStore.clients[backendId]?.status === 'ready') return true

  // 2. 准备配置信息 (优先使用 options 传入的 settings，其次查表)
  let settings = options.settings
  if (!settings) {
    const config = savedBackendsConfig.value.find((b) => b.settings.id === backendId)
    if (config) settings = config.settings
  }

  if (!settings) {
    // 如果找不到配置（可能是临时会话），且没有 Token，那就真的没法连了
    console.warn(`[Editor] No settings found for ${backendId}, waiting for IPC payload...`)
    return false
  }
  if (!settings.id) settings.id = backendId

  // 3. 初始化 Store 占位符
  if (!robotStore.clients[backendId]) {
    robotStore.initClientPlaceholder(settings)
  }
  const client = robotStore.clients[backendId]

  // 4. Token 注入
  if (options.token) {
    console.log(`[Editor] Injecting shared token for ${backendId}...`)
    try {
      const baseUrl = `http://${settings.ip}:${settings.port || 8080}/api`

      const api = createApi(baseUrl, {
        getToken: () => options.token,
        getSudoPassword: () => options.sudoPassword || robotStore.sudoPassword,
        setSudoPassword: (pwd) => {
          robotStore.setSudoPassword(pwd)
        },
        onSessionExpired: () => {
          client.status = 'disconnected'
          ElMessage.warning('会话已过期')
        }
      })

      // 手动构造 Ready 状态，不建立 WebSocket
      client.api = api
      client.token = options.token
      client.status = 'ready'
      client.ip = settings.ip
      client.id = backendId

      if (options.sudoPassword) robotStore.setSudoPassword(options.sudoPassword)
      return true
    } catch (e) {
      console.error('[Editor] Token injection failed:', e)
      return false
    }
  }
  // 如果没有 Token，直接拒绝连接。编辑器绝不尝试 WebSocket 握手。
  console.warn(`[Editor] 拒绝连接: 缺少 Token。请从主窗口打开文件以刷新凭证。`)
  return false
}

async function initData() {
  // 1. 恢复配置
  const configs = await window.api.getConfig('saved_backends')
  savedBackendsConfig.value = Array.isArray(configs) ? configs : []

  // 2. 恢复 Tab 状态 (此时是离线的)
  const savedState = await window.api.loadEditorState()
  if (Array.isArray(savedState)) {
    for (const s of savedState) {
      sessions.value.push({
        id: s.id,
        label: s.label,
        activeFile: s.activeFile || '__add_tab__',
        isOnline: false, // 初始离线，等待 IPC 更新
        files: (s.files || []).map((f) => ({
          name: f.name,
          path: f.path,
          content: '', // 暂空
          originalContent: '',
          loading: false,
          isDirty: false,
          isSaving: false,
          language: f.language || detectLanguage(f.name)
        }))
      })
    }
    if (!activeSessionId.value && sessions.value.length > 0) {
      activeSessionId.value = sessions.value[0].id
    }
  }
}
// #endregion

// #region 5. Event Handlers

async function handleOpenFile(payload) {
  const { backendId, token, settings, name, path } = payload

  // 1. 先应用 Token (这会确保 Session 存在且在线)
  if (token && settings) {
    applyTokenToSession(backendId, token, settings)
  }

  // 2. 获取 Session 对象
  const session = sessions.value.find((s) => s.id === backendId)
  if (!session) return // 理论上 applyTokenToSession 会创建

  activeSessionId.value = backendId

  // 3. 打开/激活 Tab
  let file = session.files.find((f) => f.path === path)
  if (!file) {
    file = {
      name,
      path,
      content: '',
      originalContent: '',
      loading: true,
      isDirty: false,
      isSaving: false,
      language: detectLanguage(name)
    }
    session.files.push(file)
    // 重新获取 Proxy
    file = session.files.find((f) => f.path === path)
  }
  session.activeFile = path

  // 4. 读取 (如果在线)
  if (session.isOnline) {
    file.loading = true
    try {
      const content = await robotStore.fsReadFile(backendId, path)
      file.content = content
      file.originalContent = content
    } catch (e) {
      file.content = `// Error: ${e.message}`
    } finally {
      file.loading = false
    }
  }
}

async function handleSave() {
  if (!currentFile.value || !currentSession.value) return
  const session = currentSession.value
  const file = currentFile.value

  // 防止重复提交
  if (file.isSaving) return

  // 1. 锁定本次要保存的内容快照
  const contentToSave = file.content

  // 2. 标记状态 (用于 UI 显示 Spinner)
  file.isSaving = true

  try {
    // 3. 执行写入 (等待完成)
    await robotStore.fsWriteFile(session.id, file.path, contentToSave)

    // 4. 更新基准内容
    // 注意：这里必须使用我们刚才锁定的 contentToSave，而不是 file.content
    // 因为在 await 期间，用户可能又打字了
    file.originalContent = contentToSave

    // 5. 重新计算 Dirty 状态
    // 如果用户在保存期间没打字，content === originalContent -> false
    // 如果用户打字了，content !== originalContent -> true (保持未保存状态)
    file.isDirty = file.content !== file.originalContent

    ElMessage.success('保存成功')
  } catch (error) {
    ElMessage.error(`保存失败: ${error.message}`)
  } finally {
    // 6. 解除锁定
    file.isSaving = false
  }
}

function handleEmbeddedFileSelect(session, filePath) {
  const fileName = filePath.split('/').pop()
  // 内部复用当前 token
  const client = robotStore.clients[session.id]
  handleOpenFile({
    backendId: session.id,
    backendLabel: session.label,
    name: fileName,
    path: filePath,
    token: client?.token,
    settings: { ip: client?.ip, port: client?.port }
  })
}

function handleFileBrowserSelection(filePath) {
  isFileBrowserVisible.value = false
  const fileName = filePath.split('/').pop()
  const session = currentSession.value
  const client = robotStore.clients[session.id]
  handleOpenFile({
    backendId: session.id,
    backendLabel: session.label,
    name: fileName,
    path: filePath,
    token: client?.token,
    settings: { ip: client?.ip, port: client?.port }
  })
}

async function removeFileTab(session, path) {
  const file = session.files.find((f) => f.path === path)
  if (file && file.isDirty) {
    try {
      await ElMessageBox.confirm('文件未保存，确定关闭？', '确认', { type: 'warning' })
    } catch {
      return
    }
  }
  const idx = session.files.findIndex((f) => f.path === path)
  if (idx === -1) return
  if (session.activeFile === path) {
    const next = session.files[idx + 1] || session.files[idx - 1]
    session.activeFile = next ? next.path : '__add_tab__'
  }
  session.files.splice(idx, 1)
  saveState()
}

async function removeSessionTab(id) {
  const session = sessions.value.find((s) => s.id === id)
  if (session && session.files.some((f) => f.isDirty)) {
    try {
      await ElMessageBox.confirm('会话有未保存文件，确定关闭？', '确认', { type: 'warning' })
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
  sessions.value.push({
    id: backendId,
    label,
    activeFile: '__add_tab__',
    isOnline: false,
    files: []
  })
  activeSessionId.value = backendId
  isAddMenuOpen.value = false
  // 手动添加的会话，没有 Token，这里调用 ensureConnection 默认会失败(返回 false)
  // 这是正确的，我们希望显示“连接不可用/重试”，而不是自动触发 409
  ensureConnection(backendId).then((ok) => {
    const s = sessions.value.find((x) => x.id === backendId)
    if (s) s.isOnline = ok
  })
}

function handleLanguageChange(lang) {
  if (currentFile.value) currentFile.value.language = lang
}

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
  window.api.onEditorOpenFile((_event, payload) => {
    if (payload?.backendId) handleOpenFile(payload)
  })

  // [核心修复] 防止关闭检查死锁
  window.api.onCheckUnsaved(async () => {
    // 1. 防御性获取 dirty 状态
    let hasUnsaved = false
    try {
      if (sessions.value && Array.isArray(sessions.value)) {
        hasUnsaved = sessions.value.some((s) => s.files && s.files.some((f) => f.isDirty))
      }
    } catch (e) {
      console.error('Check unsaved error:', e)
    }

    // 2. 如果没有未保存文件，直接允许关闭
    if (!hasUnsaved) {
      await saveState()
      window.api.respondClose(true)
      return
    }

    // 3. 如果正在显示弹窗，此时新的关闭请求应该被拒绝，防止逻辑混乱
    if (isClosingAlertShowing) {
      window.api.respondClose(false) // 拒绝关闭，维持现状
      return
    }

    isClosingAlertShowing = true
    try {
      await ElMessageBox.confirm('有未保存的修改，确定丢弃并关闭窗口？', '关闭确认', {
        confirmButtonText: '丢弃修改并关闭',
        cancelButtonText: '取消',
        type: 'warning',
        distinguishCancelAndClose: true
      })
      // 用户确认 -> 保存状态(Tab列表) -> 允许关闭
      await saveState()
      window.api.respondClose(true)
      // eslint-disable-next-line no-unused-vars
    } catch (action) {
      // 用户取消/关闭弹窗 -> 拒绝关闭
      window.api.respondClose(false)
    } finally {
      isClosingAlertShowing = false
    }
  })

  window.api.onTokenUpdate((payload) => {
    console.log(`[Editor] Received token update for ${payload.id}, status: ${payload.status}`)
    applyTokenToSession(payload.id, payload.token, payload.settings)
  })

  await initData()
  console.log('[Editor] Requesting tokens from Main Window...')
  window.api.requestTokens()
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
  const scrollContainer = tabsEl.querySelector('.el-tabs__nav-scroll')
  if (scrollContainer && tabsEl.querySelector('.el-tabs__header').contains(e.target)) {
    e.preventDefault()
    scrollContainer.scrollLeft += e.deltaY
  }
}
// #endregion
</script>

<style scoped>
/* ============================================
   1. 变量与主题定义
   ============================================ */
.ide-layout {
  /* Light Theme (Default) */
  --bg-color: #ffffff;
  --panel-bg: #f3f3f3; /* Sidebar bg */
  --editor-bg: #ffffff;
  --header-height: 40px; /* Tab 高度 */

  --border-color: #e4e4e4;
  --text-primary: #3b3b3b;
  --text-secondary: #868686;
  --text-disabled: #d1d1d1;

  --accent-color: #007acc; /* VS Code Blue */
  --selection-bg: #e4e6f1;
  --hover-bg: #e8e8e8;

  --status-bar-bg: #007acc;
  --status-bar-fg: #ffffff;

  /* Layout */
  display: flex;
  height: 100vh;
  width: 100vw;
  background-color: var(--bg-color);
  color: var(--text-primary);
  font-family: 'Segoe UI', sans-serif;
  overflow: hidden;
  position: relative;
}

/* Dark Theme */
:global(html.dark) .ide-layout {
  --bg-color: #1e1e1e; /* VS Code Dark */
  --panel-bg: #252526; /* Sidebar Dark */
  --editor-bg: #1e1e1e;

  --border-color: #3e3e42; /* Dark Border */
  --text-primary: #cccccc;
  --text-secondary: #858585;
  --text-disabled: #4d4d4d;

  --accent-color: #007acc;
  --selection-bg: #37373d;
  --hover-bg: #2a2d2e;

  --status-bar-bg: #007acc;
  --status-bar-fg: #ffffff;
}

/* 拖拽区域控制 */
.drag-region {
  -webkit-app-region: drag;
}
.no-drag {
  -webkit-app-region: no-drag;
}

/* ============================================
   2. 窗口控制按钮 (Windows Style)
   ============================================ */
.window-controls-tab {
  position: absolute;
  top: 0;
  right: 0;
  height: var(--header-height);
  width: 100vw;
  display: flex;
  z-index: 9998;
  -webkit-app-region: drag !important;
}
.window-controls.windows-style {
  position: absolute;
  top: 0;
  right: 0;
  height: var(--header-height);
  display: flex;
  z-index: 9999; /* 保证在最上层 */
  -webkit-app-region: no-drag !important;
}

.win-ctrl-btn {
  width: 46px;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-secondary);
  transition: all 0.2s;
  cursor: pointer;
  font-size: 14px;
  -webkit-app-region: no-drag !important;
}
.win-ctrl-btn:hover {
  background-color: rgba(128, 128, 128, 0.1);
  color: var(--text-primary);
}
.win-ctrl-btn.close:hover {
  background-color: #e81123;
  color: white;
}

/* ============================================
   3. 侧边栏 (Sidebar)
   ============================================ */
.sidebar {
  width: 220px;
  flex-shrink: 0;
  background-color: var(--panel-bg);
  display: flex;
  flex-direction: column;
  border-right: 1px solid var(--border-color);
}

.sidebar-header {
  height: var(--header-height);
  display: flex;
  align-items: center;
  padding-left: 20px;
}
.header-title {
  font-size: 14px;
  font-weight: 600;
  padding-top: 4px;
  color: var(--text-secondary);
  letter-spacing: 1px;
}

.session-list {
  flex: 1;
  overflow-y: auto;
  padding-top: 10px;
}

.section-title {
  font-size: 12px;
  font-weight: 700;
  color: var(--text-secondary);
  padding: 8px 20px;
  opacity: 0.7;
}

/* 会话列表项 */
.session-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 6px 15px 6px 20px;
  cursor: pointer;
  font-size: 13px;
  border-left: 2px solid transparent; /* 左侧指示条 */
  color: var(--text-primary);
  transition: all 0.1s;
}

.session-item:hover {
  background-color: var(--hover-bg);
}
.session-item.active {
  background-color: var(--selection-bg);
  color: var(--text-primary); /* VS Code 风格，文字不变色，靠背景区分 */
  border-left-color: var(--accent-color);
}

.session-info {
  display: flex;
  align-items: center;
  gap: 8px;
  overflow: hidden;
}
.session-info .label {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
/* 状态灯 */
.icon {
  font-size: 14px;
}
.icon.ready {
  color: #57c569;
}
.icon.failed {
  color: #f56c6c;
}
.icon.setting_up {
  color: #e6a23c;
}

/* 关闭按钮 */
.close-btn-wrapper {
  opacity: 0;
  display: flex;
  align-items: center;
  padding: 2px;
  border-radius: 4px;
}
.session-item:hover .close-btn-wrapper {
  opacity: 1;
}
.close-btn-wrapper:hover {
  background-color: rgba(128, 128, 128, 0.2);
}

/* 添加会话部分 */
.add-session-group {
  margin-top: 10px;
  border-top: 1px solid var(--border-color);
}
.add-trigger {
  color: var(--text-secondary);
  font-weight: 600;
  font-size: 11px;
  letter-spacing: 0.5px;
}
.add-trigger .arrow-icon {
  font-size: 12px;
  transition: transform 0.2s;
}
.add-trigger .arrow-icon.rotated {
  transform: rotate(90deg);
}

.add-dropdown {
  background: var(--bg-color);
  padding: 5px 0;
}
.dropdown-item {
  padding: 8px 20px 8px 32px;
  display: flex;
  align-items: center;
  font-size: 12px;
  color: var(--text-primary);
  cursor: pointer;
}
.dropdown-item:hover {
  background: var(--hover-bg);
}
.dropdown-item .dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  margin-right: 8px;
  background: #e6a23c;
}
.dropdown-item .dot.ready {
  background: #57c569;
}

/* ============================================
   4. 主编辑区 (Editor Area)
   ============================================ */
.main-area {
  flex: 1;
  display: flex;
  flex-direction: column;
  background-color: var(--editor-bg);
  min-width: 0;
}
.editor-workspace {
  flex: 1;
  display: flex;
  flex-direction: column;
  height: 0; /* 允许收缩 */
}

/* Tabs Header */
.tabs-wrapper {
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden; /* 防止溢出 */
}
.file-tabs {
  height: 100%;
  display: flex;
  flex-direction: column;
}
:deep(.el-tabs__header) {
  margin: 0;
  background-color: var(--panel-bg);
  border-bottom: 1px solid var(--border-color);
  padding-right: 140px;
  flex-shrink: 0; /* 防止头部被压缩 */
  -webkit-app-region: no-drag !important;
}
:deep(.el-tabs__nav-wrap) {
  -webkit-app-region: drag !important;
}
:deep(.el-tabs__content) {
  flex: 1; /* 占据剩余空间 */
  height: 0; /* 关键：允许 Flex 子项收缩，从而触发内部滚动条 */
  padding: 0;
  background-color: #ffffff;
}
:deep(.el-tabs__nav-wrap) {
  margin-bottom: -1px; /* 盖住底边框 */
}
:deep(.el-tabs__item) {
  height: var(--header-height);
  line-height: var(--header-height);
  border: none !important;
  border-right: 1px solid var(--border-color) !important;
  background: transparent;
  color: var(--text-secondary);
  font-size: 13px;
  padding: 0 15px !important;
  font-weight: 400;
  transition: background 0.1s;
}
:deep(.el-tabs__item.is-active) {
  background-color: var(--editor-bg); /* 激活 Tab 与编辑器背景一致 */
  color: var(--text-primary) !important;
  border-top: 1px solid var(--accent-color) !important; /* 顶部高亮条 */
}
:deep(.el-tabs__item:hover:not(.is-active)) {
  background-color: var(--hover-bg);
}
:deep(.el-tabs__item .is-icon-close) {
  /* [核心修复] 显式禁止拖拽，确保点击事件能触发 */
  -webkit-app-region: no-drag !important;
  cursor: pointer;
  border-radius: 50%;
  transition: all 0.2s;
  width: 16px;
  height: 16px;
}

/* 优化关闭按钮的悬浮效果 */
:deep(.el-tabs__item .is-icon-close:hover) {
  background-color: rgba(0, 0, 0, 0.1); /* 浅色背景 */
  color: #f56c6c !important; /* 红色图标 */
}

/* 深色模式适配 */
:global(html.dark) :deep(.el-tabs__item .is-icon-close:hover) {
  background-color: rgba(255, 255, 255, 0.2);
}

/* Tab 内容 */
.tab-label-content {
  display: flex;
  align-items: center;
  gap: 6px;
}
.file-name {
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
}
.dirty-indicator {
  font-size: 14px;
  line-height: 1;
  transform: translateY(1px);
  color: var(--text-primary);
}
.add-tab-btn {
  display: flex;
  align-items: center;
  gap: 4px;
  color: var(--accent-color);
  font-weight: 500;
}

/* Monaco Editor Container */
.monaco-container {
  height: 100%;
  width: 100%;
  background-color: var(--editor-bg);
}
:deep(.el-tab-pane) {
  height: 100%;
  width: 100%;
}

/* 嵌入浏览器 */
.embedded-browser-wrapper {
  height: 100%;
  background: var(--editor-bg);
}

/* ============================================
   5. 状态栏 (Status Bar)
   ============================================ */
.status-bar {
  height: 24px;
  background-color: var(--status-bar-bg);
  color: var(--status-bar-fg);
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0 10px;
  font-size: 12px;
  user-select: none;
  z-index: 100;
  -webkit-app-region: no-drag;
}

.bar-left,
.bar-right {
  display: flex;
  height: 100%;
}

.bar-item {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 0 10px;
  cursor: pointer;
  height: 100%;
  transition: background 0.2s;
  color: inherit;
}
.bar-item:hover {
  background-color: rgba(255, 255, 255, 0.2);
}

.bar-item.highlight {
  font-weight: 600;
}
.bar-item.warning-bg {
  background-color: #e6a23c;
  color: white;
}

.el-dropdown {
  color: inherit; /* 确保 Dropdown 组件本身也继承颜色 */
  height: 100%;
  display: flex;
  align-items: center;
}

/* ============================================
   6. 离线/空状态 (Empty States) - 修复版
   ============================================ */
.empty-state {
  /* [修复1] 确保占满父容器的所有剩余空间 */
  flex: 1;
  height: 100%;
  width: 100%;
  margin-top: var(--header-height);
  /* [修复2] 使用 Flex 居中 */
  display: flex;
  flex-direction: column; /* 明确指定垂直排列，虽然只有一个子元素，但这能避免意外的行布局行为 */
  justify-content: center; /* 垂直居中 */
  align-items: center; /* 水平居中 */
  background-color: var(--editor-bg);
  color: var(--text-secondary);
  /* 允许背景拖拽 */
  -webkit-app-region: drag;
}

/* 内部内容包裹层：确保内部元素也垂直堆叠并居中 */
.empty-content,
.state-card {
  display: flex;
  flex-direction: column; /* 让图标、标题、文字垂直排列 */
  align-items: center; /* 让它们水平居中对齐 */
  justify-content: center;
  text-align: center; /* 确保文字文本居中 */
  gap: 15px; /* 元素之间的间距 */
  max-width: 400px;
}

/* 针对 state-card 的额外装饰 (离线状态) */
.state-card {
  padding: 40px;
  border: 1px dashed var(--border-color);
  border-radius: 8px;
  background: var(--panel-bg);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.03);
}

/* 旋转加载圈 */
.spinner-ring {
  width: 40px;
  height: 40px;
  /* margin-bottom 不需要了，由 gap 控制 */
  border: 4px solid var(--border-color);
  border-top-color: var(--accent-color);
  border-radius: 50%;
  animation: spin 1s linear infinite;
}
@keyframes spin {
  to {
    transform: rotate(360deg);
  }
}

/* 文字样式微调 */
.state-card h3 {
  margin: 0;
  font-size: 18px;
  color: var(--text-primary);
}
.state-card p,
.empty-content p {
  margin: 0;
  font-size: 14px;
  color: var(--text-secondary);
  line-height: 1.5;
}
</style>
