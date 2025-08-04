import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from ui.widgets import GiftUI, GiftPublisherNode
import tkinter as tk

def main():
    rclpy.init()
    ros_node = GiftPublisherNode()
    root = tk.Tk()
    app = QApplication(sys.argv)
    window = GiftUI(ros_node, root)
    root.mainloop()

    try:
        sys.exit(app.exec_())
    finally:
        print("🛑 GUI 종료됨, ROS 노드 종료 처리 중…")
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()