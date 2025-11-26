import { NodeSSH } from 'node-ssh'
import path from 'path'
import fs from 'fs'

// 资源路径：在开发环境是 resources, 在生产环境是 resources/app.asar.unpacked/...
// 假设你把 dist 文件夹放到了 extraResources 中
const getBinaryPath = (arch) => {
  // arch: 'arm64' | 'amd64'
  const folderName = arch === 'arm64' ? 'linux-arm64' : 'linux-amd64'

  // 开发环境与生产环境路径判定
  // 在 Electron Builder 中，extraResources 会被放到 app 根目录的 resources 下
  // process.resourcesPath 指向该目录
  const basePath = process.resourcesPath
  const binaryPath = path.join(basePath, 'dist', folderName, 'ros-ground-control')

  // 检查文件是否存在 (开发环境 fallback)
  if (!fs.existsSync(binaryPath)) {
    // 尝试回退到项目源码目录 (假设 electron/main 是当前目录)
    return path.resolve(__dirname, '../../resources/dist', folderName, 'ros-ground-control')
  }
  return binaryPath
}

// [新增] 辅助函数：发送进度
const reportProgress = (sender, percent, message) => {
  if (sender && !sender.isDestroyed()) {
    sender.send('deploy-progress', { percent, message })
  }
}

// [修改] 增加 sender 参数
export async function deployAgent(config, sender) {
  const { host, port, username, password, arch, installFoxglove } = config
  const ssh = new NodeSSH()

  try {
    // 0%
    reportProgress(sender, 5, `正在连接到 ${host}...`)

    await ssh.connect({
      host,
      port,
      username,
      password,
      readyTimeout: 10000
    })

    const sudoPrefix = username === 'root' ? '' : `echo '${password}' | sudo -S `

    // [新增] 获取真实主机名
    const hostnameResult = await ssh.execCommand('hostname')
    const realHostname = hostnameResult.stdout.trim() || host

    // === 自动安装 Foxglove ===
    if (installFoxglove) {
      reportProgress(sender, 15, '正在配置 ROS 环境 (Foxglove Bridge)...')
      try {
        await ssh.execCommand(`${sudoPrefix}apt-get update`)
        reportProgress(sender, 25, '正在安装 foxglove-bridge...')
        const installResult = await ssh.execCommand(
          `${sudoPrefix}DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-foxglove-bridge`
        )
        if (installResult.code !== 0) throw new Error(installResult.stderr)
      } catch (aptError) {
        console.warn('Foxglove install failed', aptError)
      }
    }

    reportProgress(sender, 40, '正在上传 Agent 程序...')
    const localBinary = getBinaryPath(arch)
    const remoteTempPath = `/tmp/ros-ground-control`
    // 安装到 /usr/local/bin 依然是好的选择，所有用户都可执行
    const remoteInstallPath = `/usr/local/bin/ros-ground-control`

    await ssh.putFile(localBinary, remoteTempPath)
    await ssh.execCommand(`chmod +x ${remoteTempPath}`)
    await ssh.execCommand(`${sudoPrefix}mv ${remoteTempPath} ${remoteInstallPath}`)

    // [关键修改] Systemd 服务文件
    // User=${username}: 以登录用户身份运行，而不是 root
    // Environment=HOME=/home/${username}: 修正 HOME 变量，确保文件浏览正常
    // WorkingDirectory: 设置工作目录
    const serviceFileContent = `[Unit]
Description=ROS Ground Control Agent
After=network.target

[Service]
ExecStart=${remoteInstallPath}
Restart=always
User=${username}
Environment=HOME=${username === 'root' ? '/root' : '/home/' + username}
WorkingDirectory=${username === 'root' ? '/root' : '/home/' + username}

[Install]
WantedBy=multi-user.target
`
    reportProgress(sender, 70, '正在配置 Systemd 服务...')

    const tempServicePath = '/tmp/ros-agent.service'
    // 使用 Buffer 转 base64 传输内容，防止特殊字符导致 echo 写入失败
    const base64Content = Buffer.from(serviceFileContent).toString('base64')
    // remote: echo <base64> | base64 -d > target
    await ssh.execCommand(`echo "${base64Content}" | base64 -d > ${tempServicePath}`)

    await ssh.execCommand(
      `${sudoPrefix}mv ${tempServicePath} /etc/systemd/system/ros-agent.service`
    )

    reportProgress(sender, 90, '正在启动服务...')
    await ssh.execCommand(`${sudoPrefix}systemctl daemon-reload`)
    await ssh.execCommand(`${sudoPrefix}systemctl enable ros-agent`)
    await ssh.execCommand(`${sudoPrefix}systemctl restart ros-agent`)

    reportProgress(sender, 100, '部署完成')

    ssh.dispose()

    // [修改] 返回 hostname
    return { success: true, hostname: realHostname }
  } catch (error) {
    console.error('[Deploy] Error:', error)
    ssh.dispose()
    return { success: false, message: error.message }
  }
}
