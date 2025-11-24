<template>
  <!-- 路由匹配到的组件将渲染在这里 -->
  <router-view />
</template>

<style>
/* 全局样式：确保根元素占满视口 */
html,
body,
#app {
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
  overflow: hidden; /* 禁止顶层滚动 */
  font-family: 'consola', sans-serif;
}
</style>
