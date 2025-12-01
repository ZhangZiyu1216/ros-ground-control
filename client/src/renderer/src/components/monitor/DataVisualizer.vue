<template>
  <div class="data-visualizer" :style="{ '--depth': depth }">
    <!-- 顶层无数据处理 -->
    <div v-if="data === undefined || data === null" class="no-data">暂无数据</div>

    <div v-else class="json-container">
      <!-- 情况A: 根节点本身就是简单值 -->
      <div v-if="!isComplex(data)" class="root-simple-value">
        <span class="value" :class="typeof data">{{ formatSimpleValue(data) }}</span>
      </div>

      <!-- 情况B: 遍历渲染对象/数组 -->
      <div v-else>
        <div
          v-for="(val, key) in data"
          :key="key"
          class="field-wrapper"
          :class="{ 'is-root': depth === 0 }"
        >
          <!-- 第一行：Header -->
          <div class="field-header">
            <!-- 1. 左侧：折叠控制区 -->
            <div
              class="header-left"
              :class="{ 'is-clickable': isComplex(val) && !isEmpty(val) }"
              @click="toggleCollapse(key, val)"
            >
              <!-- 折叠图标 -->
              <span
                class="collapse-icon"
                :style="{ visibility: isComplex(val) && !isEmpty(val) ? 'visible' : 'hidden' }"
              >
                {{ collapsedState[key] ? '▶' : '▼' }}
              </span>
              <!-- Key 名 -->
              <span class="key" :class="{ 'key-root': depth === 0 }">{{ key }}:</span>
            </div>

            <!-- 2. 右侧：值显示区 -->
            <div class="header-right">
              <!-- 2.1 简单值 -->
              <span v-if="!isComplex(val)" class="value" :class="typeof val">
                {{ formatSimpleValue(val) }}
              </span>

              <!-- 2.2 特殊对象 -->
              <span v-else-if="isTimestamp(val)" class="value timestamp">
                {{ formatTime(val) }}
              </span>
              <span v-else-if="isTrueBinary(val)" class="value binary">
                [Binary: {{ val.byteLength }}B]
              </span>
              <span v-else-if="isTypedArray(val)" class="value typed-array">
                [{{ val.constructor.name }}({{ val.length }})] {{ formatLargeArray(val) }}
              </span>

              <!-- 2.3 复杂对象/数组 -->
              <template v-else>
                <!-- 空对象 -->
                <span v-if="isEmpty(val)" class="value empty">
                  {{ Array.isArray(val) ? '[]' : '{}' }}
                </span>
                <!-- 已折叠：显示摘要 -->
                <span
                  v-else-if="collapsedState[key]"
                  class="collapsed-summary"
                  @click="toggleCollapse(key, val)"
                >
                  {{ getSummary(val) }}
                </span>
              </template>
            </div>
          </div>

          <!-- 第二行：子级内容 (Children) -->
          <div
            v-if="isComplex(val) && !isEmpty(val) && !collapsedState[key]"
            class="field-children"
          >
            <!-- 模式一：浅层对象 (横向 Flex 布局) -->
            <div v-if="isShallow(val)" class="compact-grid">
              <div v-for="(subVal, subKey) in val" :key="subKey" class="compact-item">
                <span class="compact-key">{{ subKey }}:</span>
                <span class="value" :class="typeof subVal">{{ formatSimpleValue(subVal) }}</span>
              </div>
            </div>

            <!-- 模式二：深层对象 (递归竖向布局) -->
            <DataVisualizer v-else :data="val" :depth="depth + 1" />
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { reactive } from 'vue'

// eslint-disable-next-line no-unused-vars
const props = defineProps({
  data: [Object, String, Number, Boolean, Array],
  depth: { type: Number, default: 0 }
})

const collapsedState = reactive({})

// --- 逻辑判断 ---

const isShallow = (val) => {
  if (!isComplex(val)) return false
  for (const k in val) {
    if (isComplex(val[k])) return false
  }
  // 如果属性太多，挤在一起容易看错行，限制横排数量
  if (Object.keys(val).length > 12) return false
  return true
}

const toggleCollapse = (key, val) => {
  if (isComplex(val) && !isEmpty(val)) {
    collapsedState[key] = !collapsedState[key]
  }
}

// --- Formatters ---

const formatNumber = (num) => {
  // 1. 如果是绝对整数（如 seq, id），保持原样，避免变成 1.00000000
  if (Number.isInteger(num)) return String(num)
  // 2. 浮点数：强制保留8位小数
  let str = num.toFixed(8)
  // 3. 符号位对齐：
  // 如果是正数（>=0），在前面加一个空格，占据负号的位置
  // 这样 " 0.12345678" 和 "-0.12345678" 的长度就一致了
  if (num >= 0) {
    str = ' ' + str
  }
  return str
}

const formatSimpleValue = (val) => {
  if (typeof val === 'string') return `"${val}"`
  if (typeof val === 'number') return formatNumber(val)
  return String(val)
}

const getSummary = (val) => {
  if (Array.isArray(val)) return `Array(${val.length})`
  return `{...}`
}

const isEmpty = (val) => {
  if (Array.isArray(val)) return val.length === 0
  if (val && typeof val === 'object') return Object.keys(val).length === 0
  return false
}

// --- Type Checks ---
const isTrueBinary = (val) => val && val.constructor && val.constructor.name === 'Uint8Array'
const isTypedArray = (val) =>
  val && val.buffer && val.byteLength !== undefined && !isTrueBinary(val)
const isComplex = (val) => (typeof val === 'object' && val !== null) || Array.isArray(val)
const isTimestamp = (val) => val && typeof val === 'object' && 'sec' in val && 'nsec' in val

const formatTime = (time) => {
  const date = new Date(time.sec * 1000 + time.nsec / 1e6)
  return date.toLocaleTimeString() + `.${Math.floor(time.nsec / 1000)}`
}

const formatLargeArray = (arr) => {
  const preview = Array.from(arr.slice(0, 5))
    // 数组预览也应用 3 位小数固定格式，保持整齐
    .map((n) => (typeof n === 'number' && !Number.isInteger(n) ? n.toFixed(3) : n))
    .join(', ')
  return arr.length > 5 ? `[${preview}, ...]` : `[${preview}]`
}
</script>

<style scoped>
/* 变量定义 */
.data-visualizer {
  /* 浅色模式 */
  --json-key: #0451a5;
  --json-string: #a31515;
  --json-number: #098658;
  --json-boolean: #0000ff;
  --json-guide: #e0e0e0;
  --json-hover: rgba(0, 0, 0, 0.04);
  --compact-bg: rgba(0, 0, 0, 0.02);

  /* [重要] 必须使用等宽字体，否则即使字符数相同，宽度也会跳动 */
  font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
  font-size: 13px;
  line-height: 1.5;
  color: #333;
  width: 100%;
}

/* 深色模式 */
:global(html.dark) .data-visualizer {
  --json-key: #9cdcfe;
  --json-string: #ce9178;
  --json-number: #b5cea8;
  --json-boolean: #569cd6;
  --json-guide: #444;
  --json-hover: rgba(255, 255, 255, 0.06);
  --compact-bg: rgba(255, 255, 255, 0.04);
  color: #d4d4d4;
}

.no-data {
  color: var(--text-secondary, #909399);
  font-style: italic;
  padding: 8px;
}

.root-simple-value {
  padding: 4px;
}

/* --- 字段包装器 --- */
.field-wrapper {
  display: flex;
  flex-direction: column;
}
.field-wrapper.is-root {
  margin-bottom: 4px;
}

/* --- Header 行 --- */
.field-header {
  display: flex;
  align-items: flex-start;
  padding: 1px 0;
  border-radius: 2px;
  font-size: 15px;
}
.field-header:hover {
  background-color: var(--json-hover);
}

/* 左侧 Key */
.header-left {
  display: flex;
  align-items: center;
  flex-shrink: 0;
  margin-right: 6px;
  min-height: 18px;
  cursor: default;
}
.header-left.is-clickable {
  cursor: pointer;
}

.collapse-icon {
  width: 14px;
  text-align: center;
  font-size: 10px;
  color: #909399;
  user-select: none;
  margin-right: 2px;
  opacity: 0.7;
}

.key {
  color: var(--json-key);
  font-weight: 500;
}
.key-root {
  font-weight: 700;
  font-size: 13px;
}

/* 右侧 Value */
.header-right {
  flex: 1;
  min-width: 0;
  word-break: break-all;
}

/* --- 子级容器 --- */
.field-children {
  margin-left: 7px;
  padding-left: 14px;
  border-left: 1px solid var(--json-guide);
  margin-top: 2px;
  margin-bottom: 2px;
}
.field-wrapper:hover > .field-children {
  border-left-color: #999;
}

/* --- 紧凑网格布局 (Compact Grid) --- */
.compact-grid {
  display: flex;
  flex-wrap: wrap;
  gap: 4px 16px;
  background-color: var(--compact-bg);
  padding: 4px 8px;
  border-radius: 4px;
}

.compact-item {
  display: flex;
  align-items: center;
  white-space: nowrap;
}

.compact-key {
  color: var(--json-key);
  margin-right: 6px;
  font-size: 11px;
  opacity: 0.85;
}

/* --- 值样式 --- */
.value {
  word-break: break-all;
  white-space: pre;
}
.value.string {
  color: var(--json-string);
}
.value.number {
  color: var(--json-number);
}
.value.boolean {
  color: var(--json-boolean);
}
.value.timestamp {
  color: #d7ba7d;
}
:global(html:not(.dark)) .value.timestamp {
  color: #795e26;
}
.value.binary,
.value.typed-array,
.value.empty,
.collapsed-summary {
  color: #808080;
  font-style: italic;
}
.collapsed-summary {
  cursor: pointer;
  user-select: none;
}
</style>
