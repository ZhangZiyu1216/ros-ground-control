import { Bonjour } from 'bonjour-service'
import os from 'os'
import net from 'net'
import dns from 'dns'
import { BrowserWindow } from 'electron'

class SshDiscovery {
  constructor() {
    // [修改] 保持单例，初始化一次
    this.bonjour = new Bonjour()
    this.browser = null // 只维护一个活动的 browser
    this.devices = new Map()
    this.isScanning = false
  }

  startScan() {
    // 防止重复启动
    if (this.isScanning) return

    console.log('[SshDiscovery] Starting Scan...')
    this.isScanning = true
    this.devices.clear()

    // 1. 启动被动 mDNS 监听
    this.startPassiveScan()

    // 2. 启动主动 IP 探测
    this.startActiveScan().catch((err) => console.error('[SshDiscovery] Active scan error:', err))
  }

  stopScan() {
    if (!this.isScanning) return

    console.log('[SshDiscovery] Stopping scan.')
    this.isScanning = false

    // [修改] 仅停止查询，不销毁 bonjour 实例
    if (this.browser) {
      try {
        this.browser.stop()
      } catch (e) {
        console.warn('[SshDiscovery] Error stopping browser:', e)
      }
      this.browser = null
    }
  }

  // [新增] 程序退出时的彻底清理
  destroy() {
    this.stopScan()
    if (this.bonjour) {
      try {
        this.bonjour.destroy()
      } catch (e) {
        console.error(e)
      }
      this.bonjour = null
    }
  }

  startPassiveScan() {
    try {
      // 停止旧的（如果有）
      if (this.browser) this.browser.stop()

      // 启动新的查找
      this.browser = this.bonjour.find({ type: 'ssh' }, (service) => {
        this.handleServiceUp(service)
      })
    } catch (e) {
      console.error(`[SshDiscovery] mDNS error:`, e.message)
    }
  }

  async startActiveScan() {
    // 使用标志位控制循环
    while (this.isScanning) {
      try {
        const { subnets, localIPs } = this.getLocalNetworkInfo()
        if (subnets.length > 0) {
          const scanPromises = []
          for (const subnetPrefix of subnets) {
            for (let i = 1; i < 255; i++) {
              // 循环中的高频检查，确保能及时停止
              if (!this.isScanning) break

              const targetIP = `${subnetPrefix}.${i}`
              if (localIPs.has(targetIP)) continue

              scanPromises.push(this.checkSSHAndResolve(targetIP))
            }
          }
          // 等待本轮所有探测结束
          await Promise.allSettled(scanPromises)
        }
      } catch (error) {
        console.error('[SshDiscovery] Active scan cycle error:', error)
      }

      // 休息 5 秒，如果期间停止了，下一次 while 检查会退出
      if (this.isScanning) {
        await new Promise((r) => setTimeout(r, 5000))
      }
    }
  }

  getLocalNetworkInfo() {
    const subnets = []
    const localIPs = new Set()
    const interfaces = os.networkInterfaces()
    for (const name of Object.keys(interfaces)) {
      for (const iface of interfaces[name]) {
        if (iface.family === 'IPv4' && !iface.internal) {
          localIPs.add(iface.address)
          const prefix = iface.address.split('.').slice(0, 3).join('.')
          subnets.push(prefix)
        }
      }
    }
    return { subnets: [...new Set(subnets)], localIPs }
  }

  async checkSSHAndResolve(ip) {
    const existing = this.devices.get(ip)
    if (existing && !existing.type.includes('probe')) return
    // 调用新的探测方法
    const result = await this.probeSshPort(ip, 22, 1000)
    if (result.isOpen && this.isScanning) {
      let hostname = ip // 默认显示 IP
      let type = 'ssh-probe (ip)'
      let extraInfo = ''
      // 1. 尝试使用 Banner 增强显示
      if (result.banner) {
        // Banner 示例: "SSH-2.0-OpenSSH_8.9p1 Ubuntu-3ubuntu0.1"
        // 提取 OS 信息
        if (result.banner.toLowerCase().includes('ubuntu')) extraInfo = 'Ubuntu'
        else if (result.banner.toLowerCase().includes('debian')) extraInfo = 'Debian'
        else if (result.banner.toLowerCase().includes('raspbian')) extraInfo = 'Raspbian'
        else extraInfo = 'SSH Device'
      }
      // 2. 尝试 DNS 反查 (保持原有逻辑，作为补充)
      try {
        const hostnames = await dns.promises.reverse(ip)
        if (hostnames && hostnames.length > 0) {
          hostname = hostnames[0].replace(/\.$/, '').replace(/\.local$/, '')
          type = 'ssh-probe (dns)'
        }
        // eslint-disable-next-line no-unused-vars
      } catch (e) {
        // ignore
      }
      // 3. 构造最终显示名称
      // 如果没有解析出 Hostname，但有 OS 信息，则显示 "IP (OS)"
      // 如果解析出了 Hostname，则显示 "Hostname (IP)"
      let displayName = hostname
      if (hostname === ip && extraInfo) {
        displayName = `${ip} (${extraInfo})` // 例如: 192.168.1.10 (Ubuntu)
      } else if (hostname !== ip) {
        // hostname 已经很有意义了
      }
      // 注意：这里我们将 displayName 存入 hostname 字段，或者存入一个新的 display字段供前端使用
      // 为了兼容前端 <el-autocomplete> 的显示，我们尽量让 hostname 字段有意义
      this.addDevice({
        hostname: displayName, // 前端显示用
        ip,
        name: hostname,
        mac: 'N/A',
        type,
        rawBanner: result.banner // 保留原始 banner 备查
      })
    }
  }

  probeSshPort(ip, port, timeout = 1000) {
    return new Promise((resolve) => {
      const socket = new net.Socket()
      let isOpen = false
      let banner = ''
      let isResolved = false

      const cleanup = () => {
        if (!socket.destroyed) socket.destroy()
      }

      const finish = () => {
        if (isResolved) return
        isResolved = true
        cleanup()
        resolve({ isOpen, banner })
      }

      socket.setTimeout(timeout)

      socket.on('connect', () => {
        isOpen = true
        // 连接成功后，不要立即销毁，等待数据
      })

      socket.on('data', (data) => {
        // SSH 服务端连接后会立即发送版本字符串
        const str = data.toString().trim()
        if (str.startsWith('SSH-')) {
          banner = str
          finish() // 拿到 Banner，任务完成
        }
      })

      socket.on('timeout', () => finish())
      socket.on('error', () => finish())
      socket.on('close', () => finish())

      // 兜底：如果连接成功但 500ms 内没收到数据，也算成功但不带 Banner
      setTimeout(() => {
        if (isOpen && !isResolved) finish()
      }, timeout + 200)

      socket.connect(port, ip)
    })
  }

  handleServiceUp(service) {
    const ipv4 = service.addresses.find((addr) => /^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(addr))
    if (ipv4) {
      const rawHostname = service.host || service.name
      const cleanHostname = rawHostname.replace(/\.local\.?$/, '')
      this.addDevice({
        hostname: cleanHostname,
        ip: ipv4,
        name: service.name,
        mac: service.txt?.mac || 'N/A',
        type: 'ssh-mdns'
      })
    }
  }

  addDevice(newInfo) {
    if (!this.isScanning) return

    const existing = this.devices.get(newInfo.ip)
    let shouldUpdate = false

    if (!existing) {
      shouldUpdate = true
    } else {
      const isExistingLowQuality = existing.type.includes('probe')
      const isNewHighQuality = !newInfo.type.includes('probe')
      if (isExistingLowQuality && isNewHighQuality) shouldUpdate = true
    }

    if (shouldUpdate) {
      this.devices.set(newInfo.ip, newInfo)
      this.broadcastUpdate()
    }
  }

  broadcastUpdate() {
    const list = Array.from(this.devices.values())
    BrowserWindow.getAllWindows().forEach((win) => {
      if (!win.isDestroyed()) {
        win.webContents.send('ssh-devices-update', list)
      }
    })
  }
}

export default new SshDiscovery()
