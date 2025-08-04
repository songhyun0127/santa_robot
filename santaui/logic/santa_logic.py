# logic/santa_logic.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def handle_send_gift(gift_type: str):
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("gift_gui_publisher")
    pub = node.create_publisher(String, "/order_info", 10)

    msg = String()
    msg.data = "인형" if gift_type == "doll" else "레고"
    pub.publish(msg)

    print(f"📤 퍼블리시됨: {msg.data}")
    node.destroy_node()
    rclpy.shutdown()
