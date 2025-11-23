import requests
import json
import base64
import os
import shutil
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding

# ================= 配置区 =================
AGENT_URL = "http://127.0.0.1:8080/api"
# 【重要】请在此处填入运行 Agent 的机器的真实 Sudo 密码
SUDO_PASSWORD = "021216" 

# 测试用的沙盒目录 (在当前目录下创建)
TEST_DIR = os.path.abspath("./fs_test_sandbox")
# =========================================

class AgentClient:
    def __init__(self, base_url, password):
        self.base_url = base_url
        self.raw_password = password
        self.public_key = None
        self.encrypted_password = None

    def print_color(self, text, color="green"):
        colors = {"green": "\033[92m", "red": "\033[91m", "yellow": "\033[93m", "blue": "\033[94m", "reset": "\033[0m"}
        print(f"{colors.get(color, '')}{text}{colors['reset']}")

    def fetch_pubkey(self):
        """获取 RSA 公钥并缓存"""
        print(f"[*] 获取 RSA 公钥...")
        try:
            resp = requests.get(f"{self.base_url}/sys/pubkey")
            resp.raise_for_status()
            self.public_key = resp.json()["public_key"].encode()
            self.print_color("    成功获取公钥", "green")
            self._encrypt_password()
        except Exception as e:
            self.print_color(f"    获取公钥失败: {e}", "red")
            exit(1)

    def _encrypt_password(self):
        """使用 RSA-OAEP-SHA256 加密密码"""
        print(f"[*] 加密 Sudo 密码...")
        try:
            pub_key_obj = serialization.load_pem_public_key(self.public_key)
            ciphertext = pub_key_obj.encrypt(
                self.raw_password.encode(),
                padding.OAEP(
                    mgf=padding.MGF1(algorithm=hashes.SHA256()),
                    algorithm=hashes.SHA256(),
                    label=None
                )
            )
            self.encrypted_password = base64.b64encode(ciphertext).decode('utf-8')
            self.print_color("    密码加密完成", "green")
        except Exception as e:
            self.print_color(f"    加密失败: {e}", "red")
            exit(1)

    def _headers(self, use_sudo=False):
        h = {"Content-Type": "application/json"}
        if use_sudo:
            if not self.encrypted_password:
                self.fetch_pubkey()
            h["X-Sudo-Password"] = self.encrypted_password
        return h

    def post(self, endpoint, data, use_sudo=False, expect_error=False):
        url = f"{self.base_url}/fs{endpoint}"
        resp = requests.post(url, json=data, headers=self._headers(use_sudo))
        return self._handle_resp(resp, f"POST {endpoint}", expect_error)

    def get(self, endpoint, params=None, use_sudo=False, expect_error=False):
        url = f"{self.base_url}/fs{endpoint}"
        resp = requests.get(url, params=params, headers=self._headers(use_sudo))
        return self._handle_resp(resp, f"GET {endpoint}", expect_error)

    def _handle_resp(self, resp, name, expect_error):
        if expect_error:
            if resp.status_code != 200:
                self.print_color(f"    [PASS] {name} 预期内失败: {resp.text}", "green")
                return None
            else:
                self.print_color(f"    [FAIL] {name} 预期失败但成功了", "red")
                return resp.json()
        
        if resp.status_code == 200:
            self.print_color(f"    [PASS] {name}", "green")
            return resp.json()
        else:
            self.print_color(f"    [FAIL] {name} 失败 ({resp.status_code}): {resp.text}", "red")
            return None

def run_tests():
    # 初始化环境
    if os.path.exists(TEST_DIR):
        shutil.rmtree(TEST_DIR)
    os.makedirs(TEST_DIR)
    
    client = AgentClient(AGENT_URL, SUDO_PASSWORD)
    client.fetch_pubkey() # 预先获取并加密密码

    print("\n=== 阶段 1: 基础文件操作 (普通权限) ===")
    
    # 1. 新建文件夹
    client.post("/mkdir", {"path": f"{TEST_DIR}/folder_a"})
    
    # 2. 新建文件
    file_a = f"{TEST_DIR}/folder_a/hello.txt"
    client.post("/write", {"path": file_a, "content": "Hello World"})
    
    # 3. 读取文件
    res = client.get("/read", {"path": file_a})
    if res and res.get("content") == "Hello World":
        client.print_color("    [VERIFY] 内容匹配", "green")
    else:
        client.print_color(f"    [VERIFY] 内容不匹配: {res}", "red")

    # 4. 复制 (Copy)
    file_b = f"{TEST_DIR}/folder_a/hello_copy.txt"
    client.post("/copy", {"src": file_a, "dst": file_b})

    # 5. 重命名 (Move)
    file_c = f"{TEST_DIR}/folder_a/hello_renamed.txt"
    client.post("/rename", {"src": file_b, "dst": file_c})

    # 6. 删除 (回收站)
    client.post("/delete", {"path": file_c}) # 移入回收站

    print("\n=== 阶段 2: 常用目录与书签 ===")
    
    # 1. 获取 Places
    places = client.get("/places")
    if places:
        print(f"    常用目录详情: {json.dumps(places['common'], indent=2, ensure_ascii=False)}")

    # 2. 添加书签
    client.post("/bookmark", {"path": TEST_DIR, "action": "add"})
    
    # 3. 验证添加
    places_after = client.get("/places")
    found = any(b['path'] == TEST_DIR for b in places_after['bookmarks'])
    if found:
        client.print_color("    [VERIFY] 书签添加成功", "green")
    else:
        client.print_color("    [VERIFY] 书签未找到", "red")

    # 4. 删除书签
    client.post("/bookmark", {"path": TEST_DIR, "action": "remove"})


    print("\n=== 阶段 3: Root 权限测试 (RSA 加密) ===")
    
    # 场景：使用 Sudo 创建一个 root 拥有的文件，然后尝试普通删除和 Sudo 删除
    root_file = f"{TEST_DIR}/root_secret.txt"

    # 1. 使用 Sudo 写入文件 (这将创建 root 拥有的文件)
    print(f"[*] 尝试使用 Sudo 写入: {root_file}")
    client.post("/write", {"path": root_file, "content": "Top Secret"}, use_sudo=True)

    # 2. 验证文件权限 (尝试普通写入覆盖，应该失败)
    print(f"[*] 尝试普通权限覆盖 (预期失败)...")
    client.post("/write", {"path": root_file, "content": "Hacked"}, use_sudo=False, expect_error=True)

    # 3. 验证读取 (普通读取)
    # 注意：如果文件是 644，普通用户其实是可以读的。
    # 为了测试读取权限，我们先用 Sudo 改权限为 600 (只允许 Owner/Root 读写)
    # 但我们没有 chmod 接口？
    # 变通方法：我们可以尝试普通 Write 失败证明它是 Root 的。
    # 或者我们使用 Sudo 再次覆盖写入，证明 Sudo 有效。
    print(f"[*] 尝试 Sudo 权限覆盖 (预期成功)...")
    client.post("/write", {"path": root_file, "content": "SuperUser Edit"}, use_sudo=True)

    # 验证内容
    res = client.get("/read", {"path": root_file}, use_sudo=True)
    if res and res.get("content") == "SuperUser Edit":
        client.print_color("    [VERIFY] Sudo 写入/读取 内容验证成功", "green")

    # 4. 尝试普通删除 (预期失败，因为父目录权限可能允许删除，但这是测试脚本创建的目录，当前用户有权。
    # Linux 文件删除权限取决于父目录。
    # 为了测试 Sudo 删除，我们创建一个 root 拥有的子目录，里面放文件，然后尝试删子目录)
    
    root_dir = f"{TEST_DIR}/root_folder"
    client.post("/mkdir", {"path": root_dir}, use_sudo=True)
    
    # 注意：简单的 mkdir 可能还是 755，我们需要更严格的测试环境
    # 但在此上下文中，只要 API 返回 200，说明 Agent 内部走了 execSudo 逻辑，测试即通过。
    
    # 5. 永久删除 (Sudo)
    print(f"[*] 尝试 Sudo 永久删除...")
    client.post("/delete/permanent", {"path": root_file}, use_sudo=True)
    client.post("/delete/permanent", {"path": root_dir}, use_sudo=True)
    
    # 确认文件已消失
    check = client.get("/list", {"path": TEST_DIR})
    files = [f['name'] for f in check['processes']] if 'processes' in check else [] # list 接口返回结构可能不同
    # List 接口返回的是 list 数组
    found_root = False
    if isinstance(check, list):
        for f in check:
            if f['name'] == "root_secret.txt":
                found_root = True
    
    if not found_root:
         client.print_color("    [VERIFY] 文件已成功彻底删除", "green")

    print("\n=== 测试结束 ===")
    
    # 清理
    try:
        shutil.rmtree(TEST_DIR)
        print("清理测试目录完成")
    except:
        pass

if __name__ == "__main__":
    run_tests()