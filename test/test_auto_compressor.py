#!/usr/bin/env python3
import rospy
import time
import sys
from sensor_msgs.msg import Image

# ================= 配置区 =================
# 1. 仿真环境中真实存在的源话题 (Agent 可能已经为它启动了压缩，但这不影响测试)
# 请使用 'rostopic list' 确认这个名字
SOURCE_TOPIC = "/camera/infra1/image_raw"

# 2. 我们用来测试的“诱饵”话题
# Agent 应该会发现这个新话题，并自动为它启动压缩进程
TARGET_TOPIC = "/agent_test_bait/image_raw"
# =========================================

class AutoCompressorTest:
    def __init__(self):
        self.pub = None
        self.sub = None
        self.image_received = False

    def print_color(self, text, color="green"):
        colors = {"green": "\033[92m", "red": "\033[91m", "yellow": "\033[93m", "blue": "\033[94m", "reset": "\033[0m"}
        print(f"{colors.get(color, '')}{text}{colors['reset']}")

    def img_callback(self, data):
        self.image_received = True
        # 原封不动地转发到诱饵话题
        if self.pub:
            self.pub.publish(data)

    def run(self):
        rospy.init_node('test_auto_compressor_node', anonymous=True)

        self.print_color(f"=== 开始 Agent 自动压缩功能测试 ===", "blue")
        self.print_color(f"[*] 源话题: {SOURCE_TOPIC}")
        self.print_color(f"[*] 诱饵话题: {TARGET_TOPIC}")

        # 1. 设置转发器
        self.pub = rospy.Publisher(TARGET_TOPIC, Image, queue_size=10)
        self.sub = rospy.Subscriber(SOURCE_TOPIC, Image, self.img_callback)

        # 2. 等待源数据
        self.print_color("[*] 等待源话题数据...", "yellow")
        wait_start = time.time()
        while not self.image_received:
            if time.time() - wait_start > 10:
                self.print_color(f"[FAIL] 10秒内未收到 {SOURCE_TOPIC} 的数据。请检查仿真是否开启。", "red")
                sys.exit(1)
            time.sleep(0.1)
        
        self.print_color("[OK] 源数据流正常，开始转发到诱饵话题。", "green")
        self.print_color("[*] 正在等待 Agent 发现并启动压缩进程 (约需 5-10 秒)...", "yellow")

        # 3. 轮询检查 /compressed 话题是否出现
        # Agent 的轮询周期是 5秒，所以我们设置超时为 15秒
        timeout = 15
        check_start = time.time()
        expected_topic = TARGET_TOPIC + "/compressed"
        
        success = False
        while time.time() - check_start < timeout:
            try:
                # 获取当前所有话题
                topics = rospy.get_published_topics()
                # topics 是一个 list of [topic_name, topic_type]
                
                found = False
                for t_name, t_type in topics:
                    if t_name == expected_topic:
                        # 进一步确认类型是否正确
                        if t_type == "sensor_msgs/CompressedImage":
                            found = True
                            break
                
                if found:
                    success = True
                    break
                
                # 打印进度点
                sys.stdout.write(".")
                sys.stdout.flush()
                time.sleep(1)
                
            except Exception as e:
                self.print_color(f"\n[ERROR] 查询话题失败: {e}", "red")
                break

        print("") # 换行

        # 4. 输出结果
        if success:
            self.print_color(f"\n[PASS] 测试通过！", "green")
            self.print_color(f"    Agent 成功检测到了 '{TARGET_TOPIC}'")
            self.print_color(f"    并自动创建了 '{expected_topic}'")
        else:
            self.print_color(f"\n[FAIL] 测试失败。", "red")
            self.print_color(f"    超时 {timeout} 秒后仍未发现 '{expected_topic}'。")
            self.print_color(f"    请检查 Agent 日志是否有报错。")

if __name__ == "__main__":
    try:
        test = AutoCompressorTest()
        test.run()
    except rospy.ROSInterruptException:
        pass