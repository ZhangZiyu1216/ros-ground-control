<template>
  <!-- 路由匹配到的组件将渲染在这里 -->
  <router-view />
</template>

<script setup>
import { onMounted } from 'vue'
import { useRobotStore } from './store/robot'
const store = useRobotStore()

onMounted(() => {
  store.setupSyncListeners()
})
</script>

<style>
/* ============================================
   GLOBAL THEME SYSTEM (全局主题定义)
   此处的样式不加 scoped，应用于整个应用
   ============================================ */

/* 1. 浅色模式 (默认) */
:root {
  /* 基础背景与文字 */
  --bg-color: #f6f8fa;
  --text-primary: #303133;
  --text-secondary: #606266;
  --divider-color: #e4e7ed;

  /* 玻璃拟态 (Glassmorphism) */
  --glass-bg: rgba(255, 255, 255, 0.85);
  --glass-border: rgba(255, 255, 255, 0.6);
  --glass-shadow: 0 8px 32px rgba(0, 0, 0, 0.05);

  /* 面板/卡片 */
  --panel-bg-color: rgba(245, 245, 245, 0.75);
  --panel-border-color: rgba(200, 200, 200, 0.5);
  --panel-shadow: 0 8px 32px rgba(0, 0, 0, 0.05);
  --panel-radius: 20px;

  /* 节点卡片内部 */
  --card-inner-bg: #ffffff;
  --card-border-gradient: linear-gradient(135deg, #409eff, #8cc5ff, #ecf5ff);
  --card-shadow: 0 4px 12px rgba(64, 158, 255, 0.12);
  --add-card-text: #409eff;
  --add-card-disabled-bg: #f5f7fa;
}

/* 2. 深色模式 (当 html 标签有 .dark 类时生效) */
html.dark {
  /* 基础背景与文字 */
  --bg-color: #141414;
  --text-primary: #e5eaf3;
  --text-secondary: #a3a6ad;
  --divider-color: #4c4d4f;

  /* 玻璃拟态 */
  --glass-bg: rgba(30, 30, 30, 0.85);
  --glass-border: rgba(255, 255, 255, 0.1);
  --glass-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);

  /* 面板/卡片 */
  --panel-bg-color: rgba(60, 60, 60, 0.75);
  --panel-border-color: rgba(255, 255, 255, 0.1);
  --panel-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);

  /* 节点卡片内部 */
  --card-inner-bg: #1d1e1f;
  --card-border-gradient: linear-gradient(135deg, #337ecc, #1d3d5e, #1d1e1f);
  --card-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
  --add-card-text: #337ecc;
  --add-card-disabled-bg: #2b2b2b;
}

/* 3. 全局 Body 设置 */
body {
  margin: 0;
  padding: 0;
  width: 100vw;
  height: 100vh;
  font-family: 'Segoe UI', sans-serif;
  background-color: var(--bg-color); /* 关键：让 body 背景随主题变化 */
  color: var(--text-primary);
  transition:
    background-color 0.3s,
    color 0.3s;
  overflow: hidden;
}
</style>

<style>
:root {
  --font-main:
    'consola', 'SiYuan', 'Segoe UI', 'Helvetica Neue', 'Roboto', 'Arial', 'PingFang SC',
    'Microsoft YaHei', 'Heiti SC', sans-serif;
  --font-mono:
    'InterVariable', 'Consolas', 'Menlo', 'Monaco', 'Courier New', 'Microsoft YaHei', monospace;
  /* 基础字号 */
  --font-size-base: 14px;
}
html,
body,
#app {
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
  overflow: hidden; /* 禁止顶层滚动 */
  font-family: var(--font-main);
  font-size: var(--font-size-base);
  -webkit-font-smoothing: antialiased; /* 让字体更清晰 */
  -moz-osx-font-smoothing: grayscale;
}
/* 强制 Element Plus 组件也使用该字体 */
button,
input,
textarea,
select,
.el-button,
.el-input__inner {
  font-family: inherit; /* 继承 body 的字体设置 */
}
</style>
