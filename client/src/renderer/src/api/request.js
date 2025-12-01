/* eslint-disable no-unused-vars */
/**
 * api/request.js
 * 负责 HTTP 请求的封装、拦截、鉴权与提权
 */
import axios from 'axios'
import { encryptPassword } from '../utils/crypto'
import { ElMessageBox, ElMessage, ElNotification } from 'element-plus'

/**
 * 创建 API 实例
 * @param {string} baseUrl - 基础 URL (http://<ip>:8080/api)
 * @param {Object} options - 配置项
 * @param {Function} options.getToken - () => string | null，获取当前 Token 的回调
 * @param {Function} options.getSudoPassword - () => string | null，获取当前缓存密码的回调
 * @param {Function} options.setSudoPassword - (pwd) => void，保存密码的回调
 * @param {Function} options.onSessionExpired - () => void，当 Token 失效时的回调
 */
export const createApi = (baseUrl, options = {}) => {
  const service = axios.create({
    baseURL: baseUrl,
    timeout: 10000 // 默认 10秒超时
  })

  // --- 1. 请求拦截器：注入 Session Token ---
  service.interceptors.request.use(
    (config) => {
      // 如果调用方提供了获取 Token 的方法，则注入 Header
      if (options.getToken) {
        const token = options.getToken()
        if (token) {
          config.headers['X-Session-Token'] = token
        }
      }
      return config
    },
    (error) => Promise.reject(error)
  )

  // --- 2. 响应拦截器：错误处理与提权 ---
  service.interceptors.response.use(
    (response) => response.data,
    async (error) => {
      const originalRequest = error.config
      const status = error.response ? error.response.status : 0
      const errorData = error.response ? error.response.data : {}

      // --- 情况 A: 409 Conflict (Session 互斥锁) ---
      if (status === 409) {
        ElNotification({
          title: '设备繁忙',
          message: '已有其他客户端连接控制该机器人，连接被拒绝。',
          type: 'warning',
          duration: 5000
        })
        if (options.onSessionExpired) options.onSessionExpired()
        return Promise.reject(new Error('Device Busy'))
      }

      // --- 情况 B: 500 Permission Denied (Sudo 提权) ---
      // 检查是否包含 permission denied 关键字
      const isPermissionError =
        status === 500 &&
        (JSON.stringify(errorData).includes('permission denied') ||
          JSON.stringify(errorData).includes('EACCES'))

      if (isPermissionError && !originalRequest._retry) {
        originalRequest._retry = true

        try {
          // 1. 尝试获取缓存密码
          let password = options.getSudoPassword ? options.getSudoPassword() : ''

          // 2. 如果无密码，弹窗询问
          if (!password) {
            try {
              const { value } = await ElMessageBox.prompt(
                '该操作需要 Sudo 权限，请输入机器人系统密码：',
                '权限验证',
                {
                  inputType: 'password',
                  confirmButtonText: '确定',
                  cancelButtonText: '取消',
                  inputErrorMessage: '密码不能为空',
                  inputValidator: (val) => !!val
                }
              )
              password = value
              // 保存密码以便下次使用
              if (options.setSudoPassword) options.setSudoPassword(password)
            } catch (userCancel) {
              throw new Error('用户取消提权')
            }
          }

          // 3. 获取公钥 (使用当前的 service 实例，不带 Token 也是公开接口)
          // 注意：公钥接口通常不需要 Token，如果需要，上方拦截器已经注入了
          const pubKeyRes = await axios.get(`${baseUrl}/sys/pubkey`)
          const pubKey = typeof pubKeyRes.data === 'string' ? pubKeyRes.data : pubKeyRes.data.key

          // 4. 加密密码
          const encryptedPwd = encryptPassword(password, pubKey)

          // 5. 注入 Sudo Header 并重发请求
          originalRequest.headers['X-Sudo-Password'] = encryptedPwd
          return service(originalRequest)
        } catch (e) {
          ElMessage.error(`提权失败: ${e.message}`)
          return Promise.reject(e)
        }
      }

      // --- 其他错误 ---
      // 尝试提取后端返回的友好错误信息
      const friendlyMsg = errorData?.error || errorData?.message || error.message
      error.message = friendlyMsg
      return Promise.reject(error)
    }
  )

  return service
}
