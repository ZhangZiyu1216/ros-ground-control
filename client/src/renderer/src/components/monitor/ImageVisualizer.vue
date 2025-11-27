<!-- src/renderer/src/components/monitor/ImageVisualizer.vue -->
<template>
  <div class="image-container">
    <img v-if="imgSrc" :src="imgSrc" alt="Stream" class="live-image" />
    <div v-else class="placeholder">等待图像流...</div>
  </div>
</template>

<script setup>
import { computed } from 'vue'

const props = defineProps({
  data: Object, // ROS 消息对象
  topicType: String
})

const imgSrc = computed(() => {
  if (!props.data) return null

  // 1. 如果是解析后的对象 (rosmsg-serialization 输出)
  if (props.data.data && props.data.data instanceof Uint8Array) {
    const binary = props.data.data
    // 如果是 CompressedImage, 格式通常在 format 字段 (jpeg/png)
    // 如果是 Image (Raw), 需要 Canvas 绘制 (太复杂，暂不支持，建议用 Compressed)

    // 尝试转 Base64 显示 CompressedImage
    // 性能警告：对于大图片，在大循环中拼字符串会卡顿。
    // 生产环境应用 Blob.
    const blob = new Blob([binary], { type: 'image/jpeg' }) // 默认 jpeg
    return URL.createObjectURL(blob)
  }

  // 2. 如果是之前代码里的 Base64 降级方案
  if (props.data._isBinary && props.data.data) {
    return `data:image/jpeg;base64,${props.data.data}`
  }

  return null
})
</script>

<style scoped>
.image-container {
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #000;
  overflow: hidden;
}
.live-image {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
}
.placeholder {
  color: #555;
}
</style>
