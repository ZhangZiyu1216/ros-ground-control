<template>
  <el-dialog
    v-model="visible"
    width="680px"
    :close-on-click-modal="false"
    class="info-dialog"
    destroy-on-close
    append-to-body
    align-center
    :show-close="false"
  >
    <!-- 1. Hero Header: 品牌展示区 -->
    <div class="hero-header">
      <div class="logo-wrapper">
        <div class="logo-glow"></div>
        <img :src="logoURL" alt="Logo" class="app-logo" />
      </div>
      <div class="title-group">
        <h1 class="app-name">ROS GROUND CONTROL</h1>
      </div>
      <p class="app-slogan">ROS开发者的可视化控制终端</p>
    </div>

    <!-- 2. 内容区域 -->
    <div class="dialog-content">
      <el-tabs v-model="activeTab" class="modern-tabs">
        <!-- Tab 1: 快速上手 (引导流) -->
        <el-tab-pane label="快速上手" name="guide">
          <el-scrollbar height="320px" class="guide-scroll">
            <el-timeline class="custom-timeline">
              <el-timeline-item placement="top" :hollow="true" type="primary">
                <div class="timeline-card">
                  <div class="step-header">
                    <div class="step-icon-box blue">
                      <el-icon><MagicStick /></el-icon>
                    </div>
                    <span class="step-title">首次部署 Agent</span>
                  </div>
                  <div class="step-content">
                    悬停左下角 <span class="key-highlight">新建连接</span>，点击
                    <span class="icon-inline"><MagicStick /></span> 按钮。 输入 SSH
                    信息后，系统将自动向机器人推送并配置守护进程。
                  </div>
                </div>
              </el-timeline-item>

              <el-timeline-item placement="top" :hollow="true" type="success">
                <div class="timeline-card">
                  <div class="step-header">
                    <div class="step-icon-box green">
                      <el-icon><Connection /></el-icon>
                    </div>
                    <span class="step-title">建立连接</span>
                  </div>
                  <div class="step-content">
                    选择已配置的连接。握手成功后，点击底部的
                    <span class="key-highlight green">启动 ROS 服务</span> 按钮以初始化 Roscore 和
                    Bridge。
                  </div>
                </div>
              </el-timeline-item>

              <el-timeline-item placement="top" :hollow="true" type="warning">
                <div class="timeline-card">
                  <div class="step-header">
                    <div class="step-icon-box orange">
                      <el-icon><VideoPlay /></el-icon>
                    </div>
                    <span class="step-title">节点编排</span>
                  </div>
                  <div class="step-content">
                    在左侧面板添加 Launch 文件。点击
                    <span class="key-highlight blue">启动</span> 即可运行。
                    支持实时编辑参数文件、查看节点日志和状态监控。
                  </div>
                </div>
              </el-timeline-item>

              <el-timeline-item placement="top" :hollow="true" type="info">
                <div class="timeline-card">
                  <div class="step-header">
                    <div class="step-icon-box purple">
                      <el-icon><EditPen /></el-icon>
                    </div>
                    <span class="step-title">远程开发</span>
                  </div>
                  <div class="step-content">
                    点击顶部 <span class="icon-inline"><EditPen /></span> 进入多标签编辑器。 像使用
                    VS Code 一样浏览、修改远程文件，或直接拖拽上传。
                  </div>
                </div>
              </el-timeline-item>
            </el-timeline>
          </el-scrollbar>
        </el-tab-pane>

        <!-- Tab 2: 关于软件 (参数矩阵) -->
        <el-tab-pane label="技术规格" name="about">
          <div class="about-container">
            <div class="tech-grid">
              <div class="tech-item">
                <span class="tech-label">核心架构</span>
                <span class="tech-value">Electron + Go (C/S)</span>
              </div>
              <div class="tech-item">
                <span class="tech-label">通信协议</span>
                <span class="tech-value">RESTful API / WebSocket</span>
              </div>
              <div class="tech-item">
                <span class="tech-label">ROS 兼容性</span>
                <span class="tech-value">ROS 1 (Noetic)</span>
              </div>
              <div class="tech-item">
                <span class="tech-label">开源协议</span>
                <span class="tech-value">MIT License</span>
              </div>
            </div>

            <div class="dev-card">
              <div class="dev-info">
                <h3>Open Source Project</h3>
                <p>如果你觉得这个工具有用，欢迎 Star 或贡献代码。</p>
              </div>
              <el-button
                type="primary"
                round
                tag="a"
                href="https://github.com/ZhangZiyu1216/ros-ground-control"
                target="_blank"
              >
                <template #icon
                  ><el-icon><Document /></el-icon
                ></template>
                GitHub Repository
              </el-button>
            </div>

            <div class="copyright">Zhang Ziyu 个人开发. <br />Designed for efficiency.</div>
          </div>
        </el-tab-pane>
      </el-tabs>
    </div>

    <!-- 底部 -->
    <template #footer>
      <div class="dialog-footer-centered">
        <el-button class="close-btn-wide" @click="visible = false">关 闭</el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script setup>
import { ref, computed } from 'vue'
import logoURL from '../../../../resources/icon.png'
import { MagicStick, Connection, VideoPlay, Document, EditPen } from '@element-plus/icons-vue'

const props = defineProps({
  modelValue: Boolean
})

const emit = defineEmits(['update:modelValue'])

const visible = computed({
  get: () => props.modelValue,
  set: (val) => emit('update:modelValue', val)
})

const activeTab = ref('guide')
</script>

<style scoped>
/* ============================================
   1. Hero Header (头部)
   ============================================ */
.hero-header {
  text-align: center;
  padding: 10px 0 30px;
  position: relative;
}

.logo-wrapper {
  position: relative;
  width: 80px;
  height: 80px;
  margin: 0 auto 15px;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* Logo 背后的光晕动效 */
.logo-glow {
  position: absolute;
  width: 100%;
  height: 100%;
  background: radial-gradient(circle, rgba(64, 158, 255, 0.4) 0%, rgba(64, 158, 255, 0) 70%);
  filter: blur(10px);
  animation: glow-pulse 3s infinite alternate;
}
@keyframes glow-pulse {
  from {
    transform: scale(0.8);
    opacity: 0.6;
  }
  to {
    transform: scale(1.1);
    opacity: 1;
  }
}

.app-logo {
  width: 64px;
  height: 64px;
  position: relative;
  z-index: 2;
  filter: drop-shadow(0 4px 10px rgba(0, 0, 0, 0.1));
}

.title-group {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 10px;
  margin-bottom: 8px;
}

.app-name {
  margin: 0;
  font-size: 22px;
  font-weight: 800;
  letter-spacing: 1px;
  /* 渐变色保持独特风格，不使用全局变量 */
  background: linear-gradient(135deg, #303133 0%, #606266 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}
:global(html.dark) .app-name {
  background: linear-gradient(135deg, #ffffff 0%, #a3a6ad 100%);
  -webkit-background-clip: text;
  background-clip: text;
}

.version-badge {
  display: flex;
  align-items: center;
  font-size: 10px;
  font-weight: 700;
  border-radius: 4px;
  overflow: hidden;
  border: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
}
.v-tag {
  background: #409eff;
  color: white;
  padding: 2px 6px;
}
.v-num {
  background: var(--panel-bg-color); /* [修改] 全局变量 */
  color: var(--text-secondary); /* [修改] 全局变量 */
  padding: 2px 6px;
}

.app-slogan {
  margin: 0;
  font-size: 13px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  letter-spacing: 2px;
  text-transform: uppercase;
  opacity: 0.8;
}

/* ============================================
   2. Tabs & Guide (快速上手)
   ============================================ */
.modern-tabs :deep(.el-tabs__nav-wrap::after) {
  height: 1px;
  background-color: var(--panel-border-color); /* [修改] 全局变量 */
}
.modern-tabs :deep(.el-tabs__item) {
  font-size: 14px;
  color: var(--text-secondary); /* [修改] 全局变量 */
}
.modern-tabs :deep(.el-tabs__item.is-active) {
  color: #409eff;
  font-weight: 600;
}

.guide-scroll {
  padding: 0 20px;
}

/* 改造 Timeline 样式 */
.custom-timeline {
  padding-left: 5px;
}
.timeline-card {
  background: var(--panel-bg-color); /* [修改] 全局变量 */
  border: 1px solid var(--panel-border-color); /* [修改] 全局变量 */
  border-radius: 8px;
  padding: 12px 16px;
  margin-top: 4px; /* 对齐时间轴点 */
  transition: transform 0.2s;
}
.timeline-card:hover {
  transform: translateX(4px);
  border-color: #409eff;
}

.step-header {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 6px;
}
.step-icon-box {
  width: 24px;
  height: 24px;
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 14px;
}
/* Icon Colors */
.blue {
  background: rgba(64, 158, 255, 0.15);
  color: #409eff;
}
.green {
  background: rgba(103, 194, 58, 0.15);
  color: #67c23a;
}
.orange {
  background: rgba(230, 162, 60, 0.15);
  color: #e6a23c;
}
.purple {
  background: rgba(139, 92, 246, 0.15);
  color: #8b5cf6;
}

.step-title {
  font-weight: 600;
  color: var(--text-primary); /* [修改] 全局变量 */
  font-size: 14px;
}
.step-content {
  font-size: 13px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  line-height: 1.6;
}

/* 重点文字高亮 */
.key-highlight {
  font-weight: 600;
  color: var(--text-primary); /* [修改] 全局变量 */
  background: rgba(0, 0, 0, 0.05);
  padding: 0 4px;
  border-radius: 3px;
}
:global(html.dark) .key-highlight {
  background: rgba(255, 255, 255, 0.1);
}
.key-highlight.green {
  color: #67c23a;
}
.key-highlight.blue {
  color: #409eff;
}
.icon-inline {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  vertical-align: middle;
  font-size: 14px;
  width: 16px;
  height: 16px;
  margin: 0 2px;
  position: relative;
  top: -1px;
  color: #409eff;
}
.icon-inline :deep(.el-icon) {
  width: 100%;
  height: 100%;
  font-size: inherit;
}

/* ============================================
   3. About Section (技术规格)
   ============================================ */
.about-container {
  padding: 10px 20px;
}

.tech-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 15px;
  margin-bottom: 30px;
}

.tech-item {
  display: flex;
  flex-direction: column;
  padding: 15px;
  background: var(--panel-bg-color); /* [修改] 全局变量 */
  border-radius: 8px;
  border: 1px solid transparent;
  transition: all 0.3s;
}
.tech-item:hover {
  background: var(--bg-color); /* [修改] 全局变量 */
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05);
  border-color: var(--panel-border-color); /* [修改] 全局变量 */
}

.tech-label {
  font-size: 12px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  margin-bottom: 4px;
}
.tech-value {
  font-size: 14px;
  font-weight: 600;
  color: var(--text-primary); /* [修改] 全局变量 */
  font-family: 'Consolas', monospace;
}

/* GitHub Card */
.dev-card {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: linear-gradient(135deg, #1e1e20 0%, #2d2d30 100%);
  padding: 20px;
  border-radius: 8px;
  color: white;
  box-shadow: 0 8px 20px rgba(0, 0, 0, 0.15);
}
.dev-info h3 {
  margin: 0 0 5px 0;
  font-size: 16px;
}
.dev-info p {
  margin: 0;
  font-size: 12px;
  opacity: 0.7;
}

.copyright {
  text-align: center;
  margin-top: 30px;
  font-size: 12px;
  color: var(--text-secondary); /* [修改] 全局变量 */
  line-height: 1.5;
  opacity: 0.6;
}

/* ============================================
   4. Footer
   ============================================ */
.dialog-footer-centered {
  display: flex;
  justify-content: center;
  padding-bottom: 10px;
}
.close-btn-wide {
  width: 120px;
  border-radius: 20px;
  font-weight: 500;
  letter-spacing: 2px;
}
</style>
