import requests
import json
import base64
import sys
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding

# ================= 用户配置 =================
# Agent 地址
BASE_URL = "http://127.0.0.1:8080/api/fs"
SYS_URL = "http://127.0.0.1:8080/api/sys"

# 【重要】请输入运行 Agent 机器的真实 Sudo 密码
SUDO_PASSWORD = "021216" 
# ===========================================

class Tester:
    def __init__(self):
        self.encrypted_password = None

    def log(self, msg, color="white"):
        colors = {
            "green": "\033[92m",
            "red": "\033[91m",
            "yellow": "\033[93m",
            "blue": "\033[94m",
            "reset": "\033[0m"
        }
        print(f"{colors.get(color, '')}{msg}{colors['reset']}")

    def setup_crypto(self):
        self.log("[INIT] 获取 RSA 公钥并加密密码...", "blue")
        try:
            # 1. 获取公钥
            resp = requests.get(f"{SYS_URL}/pubkey")
            if resp.status_code != 200:
                self.log(f"获取公钥失败: {resp.text}", "red")
                sys.exit(1)
            
            pem_key = resp.json()["public_key"].encode()
            public_key = serialization.load_pem_public_key(pem_key)

            # 2. RSA-OAEP-SHA256 加密
            ciphertext = public_key.encrypt(
                SUDO_PASSWORD.encode(),
                padding.OAEP(
                    mgf=padding.MGF1(algorithm=hashes.SHA256()),
                    algorithm=hashes.SHA256(),
                    label=None
                )
            )
            self.encrypted_password = base64.b64encode(ciphertext).decode()
            self.log("✅ 密码加密成功", "green")
        except Exception as e:
            self.log(f"加密过程出错: {e}", "red")
            sys.exit(1)

    def get_headers(self, use_sudo=False):
        h = {"Content-Type": "application/json"}
        if use_sudo:
            if not self.encrypted_password:
                self.setup_crypto()
            h["X-Sudo-Password"] = self.encrypted_password
        return h

    def test_binary_detection(self):
        self.log("\n[TEST] 二进制文件探测 (读取 /bin/ls)", "blue")
        # 这是一个绝对存在的二进制文件
        target = "/bin/ls"
        try:
            resp = requests.get(f"{BASE_URL}/read", params={"path": target})
            data = resp.json()

            if data.get("is_binary") is True:
                self.log(f"✅ 成功识别为二进制文件. MIME: {data.get('mime')}", "green")
                if "content" not in data:
                    self.log("✅ 未返回 Content (符合预期)", "green")
                else:
                    self.log("❌ 错误：二进制文件不应返回 Content", "red")
            else:
                self.log(f"❌ 识别失败，被误判为文本: {data}", "red")
        except Exception as e:
            self.log(f"❌ 请求异常: {e}", "red")

    def test_text_detection(self):
        self.log("\n[TEST] 文本文件探测 (新建 .launch 文件)", "blue")
        # 创建一个测试文件
        target = "./test_detect.launch"
        content = "<launch>\n  <!-- Test -->\n</launch>"
        
        # 1. 写入
        requests.post(f"{BASE_URL}/write", json={"path": target, "content": content})
        
        # 2. 读取
        try:
            resp = requests.get(f"{BASE_URL}/read", params={"path": target})
            data = resp.json()
            
            if data.get("is_binary") is False:
                self.log(f"✅ 成功识别为文本文件. MIME: {data.get('mime')}", "green")
                if data.get("content") == content:
                    self.log("✅ 内容读取正确", "green")
                else:
                    self.log("❌ 内容不匹配", "red")
            else:
                self.log("❌ 识别失败，被误判为二进制", "red")
        finally:
            # 清理
            requests.post(f"{BASE_URL}/delete/permanent", json={"path": target})

    def test_sudo_operations(self):
        self.log("\n[TEST] Sudo 提权读写测试 (/root 目录)", "blue")
        
        # 这个文件只有 root 能写，普通用户通常无权访问 /root 目录
        target = "/root/agent_sudo_test.txt"
        secret_content = "This is a top secret config."

        # 1. 尝试普通写入 (预期失败)
        self.log("1. 尝试普通权限写入 (预期失败)...", "yellow")
        resp = requests.post(f"{BASE_URL}/write", json={"path": target, "content": "Fail"})
        if resp.status_code != 200:
            self.log(f"✅ 写入被拒绝 (符合预期): {resp.json().get('error')}", "green")
        else:
            self.log("❌ 严重错误：普通用户居然写入了 /root 文件！", "red")

        # 2. 尝试 Sudo 写入
        self.log("2. 尝试 Sudo 权限写入...", "yellow")
        resp = requests.post(f"{BASE_URL}/write", 
                             json={"path": target, "content": secret_content},
                             headers=self.get_headers(use_sudo=True))
        if resp.status_code == 200:
            self.log("✅ Sudo 写入成功", "green")
        else:
            self.log(f"❌ Sudo 写入失败: {resp.text}", "red")
            return

        # 3. 尝试普通读取 (预期失败 - 假设 /root 目录普通用户无法进入)
        # 注意：如果你的系统允许普通用户 ls /root，这里可能会通过，
        # 但通常 /root 是 700 权限。
        self.log("3. 尝试普通权限读取 (预期失败)...", "yellow")
        resp = requests.get(f"{BASE_URL}/read", params={"path": target})
        if resp.status_code != 200:
             self.log(f"✅ 读取被拒绝 (符合预期): {resp.json().get('error')}", "green")
        else:
             self.log("⚠️ 警告：普通用户可以读取该文件 (可能是系统权限设置较宽)", "yellow")

        # 4. 尝试 Sudo 读取
        self.log("4. 尝试 Sudo 权限读取...", "yellow")
        resp = requests.get(f"{BASE_URL}/read", 
                            params={"path": target},
                            headers=self.get_headers(use_sudo=True))
        data = resp.json()
        if resp.status_code == 200 and data.get("content") == secret_content:
            self.log("✅ Sudo 读取成功且内容匹配", "green")
        else:
            self.log(f"❌ Sudo 读取失败: {resp.text}", "red")

        # 5. 清理 (Sudo 删除)
        self.log("5. 清理测试文件...", "yellow")
        resp = requests.post(f"{BASE_URL}/delete/permanent", 
                             json={"path": target},
                             headers=self.get_headers(use_sudo=True))
        if resp.status_code == 200:
            self.log("✅ 文件清理完成", "green")
        else:
            self.log(f"❌ 清理失败: {resp.text}", "red")

if __name__ == "__main__":
    t = Tester()
    # 先初始化加密
    t.setup_crypto()
    
    t.test_binary_detection()
    t.test_text_detection()
    t.test_sudo_operations()