/* eslint-disable no-unused-vars */
import { NodeSSH } from 'node-ssh'
import path from 'path'
import fs from 'fs'

const getBinaryPath = (arch) => {
  const folderName = arch === 'arm64' ? 'linux-arm64' : 'linux-amd64'
  const basePath = process.resourcesPath
  const binaryPath = path.join(basePath, 'dist', folderName, 'ros-ground-control')

  if (!fs.existsSync(binaryPath)) {
    return path.resolve(__dirname, '../../resources/dist', folderName, 'ros-ground-control')
  }
  return binaryPath
}

const reportProgress = (sender, percent, message) => {
  if (sender && !sender.isDestroyed()) {
    sender.send('deploy-progress', { percent, message })
  }
}

export async function deployAgent(config, sender) {
  const { host, port, username, password, arch, installFoxglove } = config
  const ssh = new NodeSSH()

  try {
    reportProgress(sender, 5, `正在连接到 ${host}...`)

    await ssh.connect({
      host,
      port,
      username,
      password,
      readyTimeout: 10000
    })

    const sudoPrefix = username === 'root' ? '' : `echo '${password}' | sudo -S `

    const hostnameResult = await ssh.execCommand('hostname')
    const realHostname = hostnameResult.stdout.trim() || host

    // === 1. 依赖安装逻辑 (根据 installFoxglove 决定是否联网安装) ===
    if (installFoxglove) {
      reportProgress(sender, 15, '正在通过网络安装系统依赖...')

      // 包含所有依赖：基础工具 + 视频服务 + Foxglove
      const packages = ['avahi-daemon', 'psmisc', 'net-tools', 'ros-noetic-foxglove-bridge']

      const packageStr = packages.join(' ')

      try {
        await ssh.execCommand(`${sudoPrefix}apt-get update`)

        reportProgress(sender, 25, `正在下载并安装必要的软件包...`)

        // DEBIAN_FRONTEND=noninteractive 防止卡住
        const installResult = await ssh.execCommand(
          `${sudoPrefix}DEBIAN_FRONTEND=noninteractive apt-get install -y ${packageStr}`
        )

        if (installResult.code !== 0) {
          console.warn('Apt install warning:', installResult.stderr)
        }
      } catch (aptError) {
        console.warn('Dependency install failed (Network issue?):', aptError)
        // 这里不抛出致命错误，尝试继续部署 Agent，因为用户可能已经装好了
      }
    } else {
      // 离线模式：跳过所有 apt 操作
      reportProgress(sender, 15, '跳过依赖安装 (离线模式/已配置)...')
    }

    // [关键] 无论是否安装，都尝试启动 Avahi (如果已存在)
    // 这是为了确保 mDNS 发现能工作。如果没安装，这条命令会报错但我们忽略它。
    try {
      await ssh.execCommand(`${sudoPrefix}systemctl enable --now avahi-daemon`)
    } catch (e) {
      /* ignore if not installed */
    }

    // === 2. 上传 Agent ===
    reportProgress(sender, 50, '正在上传 Agent 程序...')
    const localBinary = getBinaryPath(arch)
    const remoteTempPath = `/tmp/ros-ground-control`
    const remoteInstallPath = `/usr/local/bin/ros-ground-control`

    await ssh.putFile(localBinary, remoteTempPath)
    await ssh.execCommand(`chmod +x ${remoteTempPath}`)
    await ssh.execCommand(`${sudoPrefix}mv ${remoteTempPath} ${remoteInstallPath}`)

    // === 3. 配置服务 ===
    // 确保以登录用户身份运行，且加载 ROS 环境
    const serviceFileContent = `[Unit]
Description=ROS Ground Control Agent
After=network.target avahi-daemon.service

[Service]
ExecStart=${remoteInstallPath}
Restart=always
User=${username}
Environment=HOME=${username === 'root' ? '/root' : '/home/' + username}
Environment="PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="ROS_DISTRO=noetic"
WorkingDirectory=${username === 'root' ? '/root' : '/home/' + username}

[Install]
WantedBy=multi-user.target
`
    reportProgress(sender, 75, '正在配置 Systemd 服务...')

    const tempServicePath = '/tmp/ros-agent.service'
    const base64Content = Buffer.from(serviceFileContent).toString('base64')
    await ssh.execCommand(`echo "${base64Content}" | base64 -d > ${tempServicePath}`)
    await ssh.execCommand(
      `${sudoPrefix}mv ${tempServicePath} /etc/systemd/system/ros-agent.service`
    )

    // === 4. 启动 ===
    reportProgress(sender, 90, '正在启动服务...')
    await ssh.execCommand(`${sudoPrefix}systemctl daemon-reload`)
    await ssh.execCommand(`${sudoPrefix}systemctl enable ros-agent`)
    await ssh.execCommand(`${sudoPrefix}systemctl restart ros-agent`)

    reportProgress(sender, 100, '部署完成')

    ssh.dispose()
    return { success: true, hostname: realHostname }
  } catch (error) {
    console.error('[Deploy] Error:', error)
    ssh.dispose()
    return { success: false, message: error.message }
  }
}

// [新增] 卸载 Agent
export async function uninstallAgent(config, sender) {
  const { host, port, username, password } = config
  const ssh = new NodeSSH()

  try {
    reportProgress(sender, 10, `正在连接到 ${host} 以执行卸载...`)

    await ssh.connect({
      host,
      port,
      username,
      password,
      readyTimeout: 10000
    })

    const sudoPrefix = username === 'root' ? '' : `echo '${password}' | sudo -S `

    // 1. 停止并禁用服务
    reportProgress(sender, 30, '正在停止服务...')
    // 忽略错误（可能服务本身就不存在或未运行）
    try {
      await ssh.execCommand(`${sudoPrefix}systemctl stop ros-agent`)
      await ssh.execCommand(`${sudoPrefix}systemctl disable ros-agent`)
    } catch (e) {
      console.log('Stop service warning:', e)
    }

    // 2. 删除服务文件
    reportProgress(sender, 50, '正在清理系统配置...')
    await ssh.execCommand(`${sudoPrefix}rm -f /etc/systemd/system/ros-agent.service`)
    await ssh.execCommand(`${sudoPrefix}systemctl daemon-reload`)

    // 3. 删除二进制文件
    reportProgress(sender, 80, '正在删除程序文件...')
    await ssh.execCommand(`${sudoPrefix}rm -f /usr/local/bin/ros-ground-control`)

    // 注意：我们不自动卸载依赖包 (foxglove-bridge, avahi 等)，因为这可能会破坏用户的其他环境

    reportProgress(sender, 100, '卸载完成')
    ssh.dispose()
    return { success: true }
  } catch (error) {
    console.error('[Uninstall] Error:', error)
    ssh.dispose()
    return { success: false, message: error.message }
  }
}
