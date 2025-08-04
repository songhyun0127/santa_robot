import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(String, '/order_info', 10)
        self.get_logger().info("🟢 Order Publisher 노드 시작됨")

    def send_order(self, order_text: str):
        msg = String()
        msg.data = order_text
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 주문 전송: {order_text}")


def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()

    try:
        while rclpy.ok():
            order = input("📝 주문 입력 ('인형' 또는 '레고', 'exit'으로 종료): ").strip().lower()
            if order == 'exit':
                break
            if order not in ['인형', '레고']:
                print("⚠️ 올바른 주문이 아닙니다. 다시 입력하세요.")
                continue
            node.send_order(order)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
