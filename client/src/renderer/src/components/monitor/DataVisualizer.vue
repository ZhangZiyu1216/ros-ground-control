<template>
  <div class="data-visualizer">
    <div v-if="data === undefined || data === null" class="no-data">暂无数据</div>
    <div v-else class="json-container">
      <!-- 如果 data 本身不是对象（比如解析失败返回了字符串） -->
      <div v-if="!isComplex(data)">
        <span class="value">{{ data }}</span>
      </div>

      <div v-for="(val, key) in data" v-else :key="key" class="data-row">
        <span class="key">{{ key }}:</span>

        <!-- 1. 时间戳特殊处理 -->
        <template v-if="isTimestamp(val)">
          <span class="value number">{{ formatTime(val) }}</span>
        </template>

        <!-- 2. Uint8Array (二进制) 特殊处理 -->
        <template v-else-if="isBinary(val)">
          <span class="value array">[Binary Data: {{ val.byteLength || val.length }} bytes]</span>
        </template>

        <!-- 3. 对象递归 -->
        <template v-else-if="isPlainObject(val)">
          <div class="nested">
            <DataVisualizer :data="val" />
          </div>
        </template>

        <!-- 4. 数组 -->
        <template v-else-if="Array.isArray(val)">
          <span v-if="val.length > 20" class="value array">[Array({{ val.length }})] ...</span>
          <div v-else class="nested">
            <DataVisualizer :data="val" />
          </div>
        </template>

        <!-- 5. 基础类型 -->
        <template v-else>
          <span class="value" :class="typeof val">{{ val }}</span>
        </template>
      </div>
    </div>
  </div>
</template>

<script setup>
defineProps({ data: [Object, String, Number, Boolean, Array] })

// --- 安全的类型检查函数 ---

// 检查是否是二进制数据 (Uint8Array 或类似的 TypedArray)
const isBinary = (val) => {
  if (!val) return false
  // 不使用 instanceof，改用构造函数名称或 duck typing
  return val.constructor?.name === 'Uint8Array' || (val.buffer && val.byteLength !== undefined)
}

// 检查是否是纯对象 (排除 Array 和 Binary)
const isPlainObject = (val) => {
  return val && typeof val === 'object' && !Array.isArray(val) && !isBinary(val)
}

// 检查是否是复杂类型 (对象或数组)
const isComplex = (val) => {
  return (typeof val === 'object' && val !== null) || Array.isArray(val)
}

// 检查是否是 ROS Time 对象 { sec: 123, nsec: 456 }
const isTimestamp = (val) => {
  return val && typeof val === 'object' && 'sec' in val && 'nsec' in val
}

const formatTime = (time) => {
  const date = new Date(time.sec * 1000 + time.nsec / 1e6)
  return date.toLocaleTimeString() + `.${Math.floor(time.nsec / 1000)}`
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
