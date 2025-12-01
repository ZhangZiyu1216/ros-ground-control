<template>
  <div class="log-manager-root">
    <!-- 情况1: 当前未选择任何机器人，或者 Store 中没有任何连接 -->
    <div v-if="!currentBackendId || clientKeys.length === 0" class="empty-state">
      <el-empty description="未连接到目标主机" />
    </div>

    <!-- 情况2: 有选中的机器人 -->
    <!-- 核心逻辑：遍历所有 Client，但只显示当前选中的那个。
         这样做的目的是利用 Vue 的组件复用机制配合 v-show，
         确保当用户切换机器人时，后台的终端连接(WebSocket)不会断开，
         再次切回来时日志和终端上下文依然存在。 -->
    <template v-else>
      <div
        v-for="id in clientKeys"
        v-show="id === currentBackendId"
        :key="id"
        class="log-panel-wrapper"
      >
        <!-- 传入 connected 属性 -->
        <!-- 只有当 ID 匹配且后端 Ready 时，才算已连接 -->
        <LogPanel :backend-id="id" :connected="id === currentBackendId && isCurrentBackendReady" />
      </div>
    </template>
  </div>
</template>

<script setup>
import { computed } from 'vue'
import LogPanel from '../components/log/LogPanel.vue'
import { useRobotStore } from '../store/robot'

// eslint-disable-next-line no-unused-vars
const props = defineProps({
  currentBackendId: {
    type: String,
    default: ''
  },
  isCurrentBackendReady: {
    type: Boolean,
    default: false
  }
})

const store = useRobotStore()

// 获取所有已连接机器人的 ID 列表
const clientKeys = computed(() => Object.keys(store.clients))
</script>

<style scoped>
/* 根容器：应用统一的玻璃面板风格 */
.log-manager-root {
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden;

  /* 玻璃拟态 */
  background: var(--panel-bg-color);
  backdrop-filter: blur(16px);
  -webkit-backdrop-filter: blur(16px);
  border: 1px solid var(--panel-border-color);
  border-radius: var(--panel-radius);
  box-shadow: var(--panel-shadow);

  /* [关键] 去掉 padding，让内部的 LogPanel (含二级 Tabs) 能够完整贴边填充 */
  padding: 0;
  box-sizing: border-box;
}

/* 空状态居中 */
.empty-state {
  height: 100%;
  width: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* 包装器：确保占满父容器 */
.log-panel-wrapper {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  /* 防止溢出 */
  overflow: hidden;
}

/* 
   LogPanel 内部也是 height: 100%，
   配合这里的结构，实现了完美的铺满效果。
*/
</style>
