import websocket
import threading
import json
import time
import sys

# Agent 地址
WS_URL = "ws://127.0.0.1:8080/ws/terminal"

def on_message(ws, message):
    # message 是 bytes 类型 (BinaryMessage)
    # 直接解码打印，模拟终端显示
    try:
        text = message.decode('utf-8')
        # 使用 sys.stdout.write 以避免自动换行，保持原样输出
        sys.stdout.write(text)
        sys.stdout.flush()
    except:
        print(f"[Binary Data: {len(message)} bytes]")

def on_error(ws, error):
    print(f"\n[WS-ERROR] {error}")

def on_close(ws, status, msg):
    print("\n[WS] Closed")

def on_open(ws):
    print("[WS] Connected to Terminal")
    
    def run():
        time.sleep(1)
        
        # 1. 发送 Resize 指令
        resize_cmd = {
            "type": "resize",
            "cols": 100,
            "rows": 30
        }
        ws.send(json.dumps(resize_cmd))
        
        # 2. 发送 ls 命令 (模拟用户打字)
        # 注意：\r 是回车，shell 需要它来执行
        cmd_input = {
            "type": "input",
            "data": "ls -la --color=auto\r"
        }
        ws.send(json.dumps(cmd_input))

        time.sleep(2)

        # 3. 发送 uname -a
        ws.send(json.dumps({"type": "input", "data": "uname -a\r"}))
        
        time.sleep(1)
        
        # 4. 退出 shell
        # ws.send(json.dumps({"type": "input", "data": "exit\r"}))
        # time.sleep(1)
        # ws.close()

    threading.Thread(target=run).start()

if __name__ == "__main__":
    # enableTrace 可以看到底层帧信息
    # websocket.enableTrace(True)
    ws = websocket.WebSocketApp(WS_URL,
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.run_forever()