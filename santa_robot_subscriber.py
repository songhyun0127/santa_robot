import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class GiftRobotSubscriber(Node):
    def __init__(self):
        super().__init__('santa_robot_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/gift/command',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"📦 Received gift command: {command}")

        if command == "doll":
            # 여기서 m0609 제어 코드를 실행하거나 import
            self.run_m0609_doll_sequence()
        elif command == "lego":
            self.run_m0609_lego_sequence()

    def run_m0609_doll_sequence(self):
        # 예시: 로봇 제어 코드 파일 직접 실행
        subprocess.run(["python3", "m0609_doll_control.py"])

    def run_m0609_lego_sequence(self):
        subprocess.run(["python3", "m0609_lego_control.py"])

def main(args=None):
    rclpy.init(args=args)
    node = GiftRobotSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
