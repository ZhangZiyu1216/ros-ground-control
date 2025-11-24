<!-- src/renderer/src/components/TreeView.vue -->
<template>
  <div class="tree-view">
    <div v-for="(value, key) in data" :key="key" class="tree-node">
      <span class="tree-key">{{ key }}:</span>
      <template v-if="isObject(value)">
        <TreeView :data="value" />
      </template>
      <template v-else>
        <span class="tree-value">{{ value }}</span>
      </template>
    </div>
  </div>
</template>

<script setup>
defineProps({ data: Object })
const isObject = (val) => val && typeof val === 'object' && !Array.isArray(val)
</script>

<style scoped>
.tree-view {
  padding-left: 20px;
}
.tree-node {
  margin-top: 5px;
}
.tree-key {
  color: #c586c0;
  font-weight: bold;
}
.tree-value {
  color: #9cdcfe;
}
</style>
