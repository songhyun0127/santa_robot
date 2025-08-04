from tkinter import messagebox
import rclpy
from std_msgs.msg import String

def handle_send_gift(card_type):
    # GUI 알림
    if card_type == "doll":
        messagebox.showinfo("포장 완료", "인형이 포장되었습니다!")
    elif card_type == "lego":
        messagebox.showinfo("포장 완료", "레고가 포장되었습니다!")
    else:
        messagebox.showwarning("오류", "알 수 없는 카드 타입입니다.")
        return

    # ROS2 퍼블리시
    try:
        rclpy.init(args=None)
        node = rclpy.create_node('gift_gui_publisher')
        pub = node.create_publisher(String, '/gift/command', 10)

        msg = String()
        msg.data = card_type
        pub.publish(msg)
        print(f"[GUI → ROS2] Published: {msg.data}")

        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[ERROR] ROS2 퍼블리시 실패: {e}")
