import requests
import websocket
import threading
import json
import time
import sys

# ================= 用户配置区 =================
# Agent 地址
AGENT_IP = "127.0.0.1"
AGENT_PORT = "8080"

# [关键] 请修改为你本机真实的 launch 文件路径
# 例如: "/home/user/catkin_ws/src/my_pkg/launch/test.launch"
#TEST_LAUNCH_PATH = "/home/zzy/vins_ws/src/VINS-Fusion/vins_estimator/launch/Drone_250.launch"
#TEST_LAUNCH_PATH = "/opt/ros/noetic/share/mavros/launch/px4.launch"
TEST_LAUNCH_PATH = "/home/zzy/PX4-Autopilot/launch/mavros_posix_sitl.launch"

# 这个 launch 文件属于哪个包 (用于构建命令参数，类似: roslaunch pkg file.launch)
# 如果你只想通过绝对路径启动，可以在 args 里不传 pkg，Agent 逻辑需要适配，
# 但通常 roslaunch 用法是: roslaunch <pkg> <file>
# 这里我们模拟前端直接传绝对路径给 Agent，看 Agent 的处理 (Agent目前通过 args 传参)
# 为了通用性，Agent 的 ProcessManager 是直接执行 cmd + args。
# 如果你的 Agent 期望的是 `roslaunch file.launch` (直接路径)，请配置如下：
TEST_CMD = "roslaunch"
TEST_ARGS = [TEST_LAUNCH_PATH] 

# ============================================

BASE_URL = f"http://{AGENT_IP}:{AGENT_PORT}/api"
WS_URL = f"ws://{AGENT_IP}:{AGENT_PORT}/ws/logs"

# 全局日志存储
received_logs = []
ws_thread = None
is_running = True

def print_color(text, color="green"):
    colors = {
        "green": "\033[92m",
        "red": "\033[91m",
        "yellow": "\033[93m",
        "blue": "\033[94m",
        "reset": "\033[0m"
    }
    print(f"{colors.get(color, '')}{text}{colors['reset']}")

# --- WebSocket 处理 ---
def on_message(ws, message):
    data = json.loads(message)
    # 只打印我们测试进程的日志，过滤掉 system (ros bridge等) 的日志以免刷屏
    if data.get("process_id") == "test-proc-001":
        print(f"[WS-LOG] {data['data']}")
    received_logs.append(data)

def on_error(ws, error):
    if is_running:
        print_color(f"[WS-ERROR] {error}", "red")

def on_close(ws, close_status_code, close_msg):
    print("[WS] Closed")

def start_ws():
    ws = websocket.WebSocketApp(WS_URL,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.run_forever()

# --- 主测试逻辑 ---
def run_test():
    global ws_thread, is_running
    
    print_color("=== 开始 ROS Ground Control 集成测试 ===", "blue")

    # 1. 测试 Ping
    try:
        resp = requests.get(f"{BASE_URL}/sys/ping")
        if resp.status_code == 200:
            print_color(f"[PASS] Agent 连接成功: {resp.json()}", "green")
        else:
            print_color(f"[FAIL] Agent 连接失败: {resp.text}", "red")
            return
    except Exception as e:
        print_color(f"[FAIL] 无法连接到 Agent，请确认 main.go 已运行。错误: {e}", "red")
        return

    # 2. 启动 WebSocket 线程
    print_color("[INFO] 正在建立 WebSocket 连接...", "yellow")
    ws_thread = threading.Thread(target=start_ws)
    ws_thread.daemon = True
    ws_thread.start()
    time.sleep(1) # 等待连接建立

    # 3. 获取并打印当前网络接口配置
    try:
        resp = requests.get(f"{BASE_URL}/sys/config")
        print_color(f"[INFO] 当前 Agent 网络配置: {resp.json()}", "blue")
    except:
        pass

    # 4. 测试启动 Launch 文件 (核心测试)
    print_color(f"[STEP] 尝试启动 launch 文件: {TEST_LAUNCH_PATH}", "yellow")
    
    payload = {
        "id": "test-proc-001",
        "cmd": TEST_CMD,
        "args": TEST_ARGS
    }

    try:
        resp = requests.post(f"{BASE_URL}/proc/start", json=payload)
        result = resp.json()
        
        if resp.status_code == 200:
            print_color("[PASS] 请求发送成功", "green")
            
            # --- 验证点 1: 自动环境探测 ---
            detected_setup = result.get("setup", "")
            if detected_setup and "setup.bash" in detected_setup:
                if detected_setup == "/opt/ros/noetic/setup.bash":
                     print_color(f"[WARN] 使用了系统默认环境: {detected_setup} (未检测到工作空间或本来就是系统包)", "yellow")
                else:
                     print_color(f"[PASS] 成功探测到工作空间环境: {detected_setup}", "green")
            else:
                print_color(f"[FAIL] 未返回 setup 脚本路径: {result}", "red")

        else:
            print_color(f"[FAIL] 启动失败: {resp.text}", "red")
            return

    except Exception as e:
        print_color(f"[FAIL] 请求异常: {e}", "red")
        return

    # 5. 观察运行状态 (5秒)
    print_color("[INFO] 进程运行中，观察日志输出 (5秒)...", "blue")
    time.sleep(5)

    # 6. 停止进程
    print_color("[STEP] 正在停止进程...", "yellow")
    try:
        stop_payload = {"id": "test-proc-001"}
        resp = requests.post(f"{BASE_URL}/proc/stop", json=stop_payload)
        if resp.status_code == 200:
            print_color("[PASS] 进程停止成功", "green")
        else:
            print_color(f"[FAIL] 停止失败: {resp.text}", "red")
    except Exception as e:
        print_color(f"[FAIL] 停止请求异常: {e}", "red")

    is_running = False
    print_color("=== 测试结束 ===", "blue")

if __name__ == "__main__":
    run_test()