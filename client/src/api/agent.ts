import axios from 'axios'

// 创建一个 axios 实例，baseUrl 将在连接后动态设置
const agentRequest = axios.create({
  timeout: 5000
})

export const setAgentHost = (ip: string, port: number) => {
  agentRequest.defaults.baseURL = `http://${ip}:${port}`
}

export const checkHealth = async () => {
  return agentRequest.get('/ping')
}

// 后续所有的文件操作、进程控制都写在这里
export const fsList = async (path: string) => {
  return agentRequest.get('/api/fs/list', { params: { path } })
}