import forge from 'node-forge'

/**
 * 使用公钥加密密码
 * @param {string} password - 用户输入的 Sudo 密码
 * @param {string} pemKey - 从 /sys/pubkey 获取的 PEM 格式公钥
 * @returns {string} Base64 编码的加密字符串
 */
export function encryptPassword(password, pemKey) {
  const publicKey = forge.pki.publicKeyFromPem(pemKey)
  const encrypted = publicKey.encrypt(password, 'RSA-OAEP', {
    md: forge.md.sha256.create(),
    mgf1: {
      md: forge.md.sha256.create()
    }
  })
  return forge.util.encode64(encrypted)
}
