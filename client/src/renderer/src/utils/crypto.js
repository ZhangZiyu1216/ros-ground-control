import forge from 'node-forge'

/**
 * 格式化 RSA 公钥，确保其符合 PEM 标准
 * @param {string} key - 原始公钥字符串
 * @returns {string} 标准 PEM 格式公钥
 */
function formatPubKey(key) {
  if (!key) return ''

  // 1. 如果有些后端返回的是 JSON 对象里的字段，这里防错一下
  if (typeof key === 'object') {
    // 尝试寻找可能的字段，或者直接转字符串
    key = key.key || key.pubkey || key.public_key || String(key)
  }

  // 2. 清理现有的头尾和空白字符，只保留 Base64 内容
  let body = key
    .replace(/-----BEGIN PUBLIC KEY-----/g, '')
    .replace(/-----END PUBLIC KEY-----/g, '')
    .replace(/[\r\n\s]/g, '') // 去除所有换行和空格

  // 3. 按每 64 个字符插入一个换行符 (PEM 标准)
  const chunked = body.match(/.{1,64}/g).join('\n')

  // 4. 重新拼接标准的头尾
  return `-----BEGIN PUBLIC KEY-----\n${chunked}\n-----END PUBLIC KEY-----`
}

/**
 * 使用公钥加密密码
 * @param {string} password - 用户输入的 Sudo 密码
 * @param {string} rawKey - 从后端获取的原始公钥 (可能格式不标准)
 * @returns {string} Base64 编码的加密字符串
 */
export function encryptPassword(password, rawKey) {
  try {
    // 1. 标准化 Key
    const pemKey = formatPubKey(rawKey)

    // 2. 解析公钥
    const publicKey = forge.pki.publicKeyFromPem(pemKey)

    // 3. 加密 (RSA-OAEP-SHA256)
    const encrypted = publicKey.encrypt(password, 'RSA-OAEP', {
      md: forge.md.sha256.create(),
      mgf1: {
        md: forge.md.sha256.create()
      }
    })

    return forge.util.encode64(encrypted)
  } catch (e) {
    console.error('Encryption failed. Raw Key:', rawKey)
    throw new Error(`公钥解析失败: ${e.message}`)
  }
}
