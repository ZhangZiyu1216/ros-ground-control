package security

import (
	"crypto/rand"
	"crypto/rsa"
	"crypto/sha256"
	"crypto/x509"
	"encoding/base64"
	"encoding/pem"
	"fmt"
	"log"
	"sync"
)

var (
	privateKey *rsa.PrivateKey
	publicKey  []byte // PEM 格式的公钥缓存
	once       sync.Once
)

// InitKeys 生成 RSA 2048 密钥对
func InitKeys() {
	once.Do(func() {
		var err error
		// 生成私钥
		privateKey, err = rsa.GenerateKey(rand.Reader, 2048)
		if err != nil {
			log.Fatalf("Failed to generate RSA keys: %v", err)
		}

		// 提取公钥并转为 PKIX 格式 (适用于跨语言交互)
		pubASN1, err := x509.MarshalPKIXPublicKey(&privateKey.PublicKey)
		if err != nil {
			log.Fatalf("Failed to marshal public key: %v", err)
		}

		// 转为 PEM 格式 (Client 端容易读取)
		publicKey = pem.EncodeToMemory(&pem.Block{
			Type:  "PUBLIC KEY",
			Bytes: pubASN1,
		})

		log.Println("[Security] RSA KeyPair generated.")
	})
}

// GetPublicKeyPEM 获取 PEM 格式公钥
func GetPublicKeyPEM() string {
	if publicKey == nil {
		InitKeys()
	}
	return string(publicKey)
}

// DecryptPassword 解密客户端传来的 Base64 密文
func DecryptPassword(encryptedBase64 string) (string, error) {
	if privateKey == nil {
		return "", fmt.Errorf("security module not initialized")
	}

	// 1. Base64 解码
	ciphertext, err := base64.StdEncoding.DecodeString(encryptedBase64)
	if err != nil {
		return "", fmt.Errorf("base64 decode failed: %v", err)
	}

	// 2. RSA 解密 (使用 OAEP + SHA256，这是目前推荐的安全标准)
	hash := sha256.New()
	plaintext, err := rsa.DecryptOAEP(hash, rand.Reader, privateKey, ciphertext, nil)
	if err != nil {
		return "", fmt.Errorf("decryption failed: %v", err)
	}

	return string(plaintext), nil
}
