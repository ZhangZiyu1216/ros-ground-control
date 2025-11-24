import axios from 'axios'
import { useRobotStore } from '../store/robot' // 假设你用 Pinia 存储机器人 IP
import { encryptPassword } from '../utils/crypto'
import { ElMessageBox, ElMessage } from 'element-plus'

// 创建实例的工厂函数（因为 IP 是动态的）
export const createApi = (baseUrl) => {
  const service = axios.create({
    baseURL: baseUrl,
    timeout: 10000
  })

  // 响应拦截器
  service.interceptors.response.use(
    (response) => response.data,
    async (error) => {
      const originalRequest = error.config
      // 检查是否是权限错误 (Agent 约定返回 500 且包含 permission denied，或者你可以约定 401/403)
      // 根据描述：API 返回 HTTP 500 错误，并在 error 字段中包含 "permission denied"
      if (
        error.response &&
        error.response.status === 500 &&
        JSON.stringify(error.response.data).includes('permission denied') &&
        !originalRequest._retry
      ) {
        originalRequest._retry = true
        try {
          // 1. 获取用户密码 (这里简化处理，实际应从 Store 获取缓存的密码或弹窗询问)
          // 建议在 Pinia 中维护一个 sessionPassword，用户输入一次后暂存
          const robotStore = useRobotStore()
          let password = robotStore.sudoPassword

          if (!password) {
            // 如果没有缓存密码，弹窗询问
            const { value } = await ElMessageBox.prompt(
              '该操作需要 Sudo 权限，请输入密码',
              '权限验证',
              {
                inputType: 'password'
              }
            )
            password = value
            robotStore.setSudoPassword(password) // 缓存它
          }
          // 2. 获取公钥
          const pubKeyRes = await axios.get(`${baseUrl}/sys/pubkey`)
          const pubKey = pubKeyRes.data
          // 3. 加密
          const encryptedPwd = encryptPassword(password, pubKey)
          // 4. 添加 Header 并重试
          originalRequest.headers['X-Sudo-Password'] = encryptedPwd
          return service(originalRequest)
        } catch (e) {
          ElMessage.error('提权失败: ' + e.message)
          return Promise.reject(error)
        }
      }

      return Promise.reject(error)
    }
  )

  return service
}
