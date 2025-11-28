<template>
  <div class="data-visualizer">
    <div v-if="data === undefined || data === null" class="no-data">暂无数据</div>
    <div v-else class="json-container">
      <!-- 简单值 -->
      <div v-if="!isComplex(data)">
        <span class="value">{{ data }}</span>
      </div>

      <div v-for="(val, key) in data" v-else :key="key" class="data-row">
        <span class="key">{{ key }}:</span>

        <!-- 1. 时间戳 -->
        <template v-if="isTimestamp(val)">
          <span class="value number">{{ formatTime(val) }}</span>
        </template>

        <!-- 2. 真正的二进制 (Uint8Array) -->
        <template v-else-if="isTrueBinary(val)">
          <span class="value array">[Binary Stream: {{ val.byteLength }} bytes]</span>
        </template>

        <!-- 3. TypedArray (Float64Array 等) -> 转为普通数组显示 -->
        <template v-else-if="isTypedArray(val)">
          <!-- 如果数据太长 (比如点云)，只显示前几个 -->
          <span v-if="val.length > 20" class="value array">
            [{{ val.constructor.name }}({{ val.length }})] {{ formatLargeArray(val) }}
          </span>
          <div v-else class="nested">
            <span class="value number">{{ Array.from(val).join(', ') }}</span>
          </div>
        </template>

        <!-- 4. 普通对象 -->
        <template v-else-if="isPlainObject(val)">
          <div class="nested">
            <DataVisualizer :data="val" />
          </div>
        </template>

        <!-- 5. 普通数组 -->
        <template v-else-if="Array.isArray(val)">
          <span v-if="val.length > 20" class="value array">[Array({{ val.length }})] ...</span>
          <div v-else class="nested">
            <DataVisualizer :data="val" />
          </div>
        </template>

        <!-- 6. 兜底 -->
        <template v-else>
          <span class="value" :class="typeof val">{{ val }}</span>
        </template>
      </div>
    </div>
  </div>
</template>

<script setup>
defineProps({ data: [Object, String, Number, Boolean, Array] })

// --- 类型检查工具 ---

// 1. 严格的二进制检查：只认 Uint8Array
const isTrueBinary = (val) => {
  return val && val.constructor && val.constructor.name === 'Uint8Array'
}

// 2. 检查其他 TypedArray (Float64Array, Int32Array 等)
const isTypedArray = (val) => {
  return val && val.buffer && val.byteLength !== undefined && !isTrueBinary(val)
}

const isPlainObject = (val) => {
  return (
    val &&
    typeof val === 'object' &&
    !Array.isArray(val) &&
    !isTypedArray(val) &&
    !isTrueBinary(val)
  )
}

const isComplex = (val) => {
  return (typeof val === 'object' && val !== null) || Array.isArray(val)
}

const isTimestamp = (val) => {
  return val && typeof val === 'object' && 'sec' in val && 'nsec' in val
}

const formatTime = (time) => {
  const date = new Date(time.sec * 1000 + time.nsec / 1e6)
  return date.toLocaleTimeString() + `.${Math.floor(time.nsec / 1000)}`
}

const formatLargeArray = (arr) => {
  const preview = Array.from(arr.slice(0, 5))
    .map((n) => (n.toFixed ? n.toFixed(3) : n))
    .join(', ')
  return `[${preview}, ...]`
}
</script>

<style scoped>
.data-visualizer {
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 12px;
  line-height: 1.5;
  color: #d4d4d4;
  overflow-x: hidden;
}
.no-data {
  color: #666;
  font-style: italic;
}
.json-container {
  padding-left: 4px;
}
.data-row {
  display: flex;
  flex-wrap: wrap;
  margin-bottom: 2px;
}
.key {
  color: #9cdcfe;
  margin-right: 8px;
  font-weight: 600;
  min-width: 80px;
}
.nested {
  width: 100%;
  border-left: 1px solid #444;
  margin-left: 4px;
  padding-left: 8px;
}
.value {
  word-break: break-all;
}
.value.string {
  color: #ce9178;
}
.value.number {
  color: #b5cea8;
}
.value.boolean {
  color: #569cd6;
}
.value.array {
  color: #dcdcaa;
  font-style: italic;
}
</style>
