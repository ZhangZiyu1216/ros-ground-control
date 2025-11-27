import { createRouter, createWebHashHistory } from 'vue-router'
import MainView from '../views/MainView.vue'
import EditorView from '../views/EditorView.vue'

const router = createRouter({
  history: createWebHashHistory(),
  routes: [
    {
      path: '/',
      name: 'main',
      component: MainView
    },
    {
      path: '/editor',
      name: 'editor',
      component: EditorView
    }
  ]
})

export default router
