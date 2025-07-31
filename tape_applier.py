import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from time import sleep

from DSR_ROBOT2 import movej, movec, set_tool, set_tcp
from DR_common2 import posj, posx

# 실제 로봇 제어 클래스
class RealRobot:
    def __init__(self):
        set_tool("Tool Weight_2FG")  # 실제 사용 툴 이름
        set_tcp("2FG_TCP")           # 실제 TCP 이름

    def movej(self, pos, vel=0.0, acc=0.0):
        target = posx(pos)
        movej(target, vel=vel, acc=acc)

    def movec(self, via, to, vel=0.0, acc=0.0):
        via_pos = posx(via)
        to_pos = posx(to)
        movec(via_pos, to_pos, vel=vel, acc=acc)

# 테이핑 로직 노드
class TapeApplier(Node):
    def __init__(self, robot):
        super().__init__('tape_applier_node')
        self.robot = robot

        # 테이프 경로 정의 (mm 단위)
        self.tape_start = [400.0, -200.0, 150.0, 180.0, 0.0, 90.0]  # X, Y, Z, Rx, Ry, Rz
        self.tape_end   = [550.0,  -50.0, 250.0, 180.0, 0.0, 90.0]

        # RViz 마커 퍼블리셔 생성 (1초마다 반복 퍼블리시)
        self.marker_pub = self.create_publisher(Marker, 'tape_path_marker', 10)
        self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # 선 굵기 (m 단위)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.a = 1.0

        # mm → m 변환
        p1 = Point(x=self.tape_start[0] / 1000.0, y=self.tape_start[1] / 1000.0, z=self.tape_start[2] / 1000.0)
        p2 = Point(x=self.tape_end[0] / 1000.0,   y=self.tape_end[1] / 1000.0,   z=self.tape_end[2] / 1000.0)
        marker.points.append(p1)
        marker.points.append(p2)

        self.marker_pub.publish(marker)

    def apply_tape(self):
        # 1. 접근 위치 설정 (시작점에서 10cm 위)
        approach = self.tape_start.copy()
        approach[2] += 100

        self.robot.movej(approach, vel=20, acc=20)
        self.get_logger().info("접근 위치로 이동 완료")

        # 2. 테이프 접촉 지점으로 이동
        self.robot.movej(self.tape_start, vel=10, acc=10)
        self.get_logger().info("테이프 접촉 위치 도달")

        # 3. 테이프 들어올리기 (5cm)
        lift = self.tape_start.copy()
        lift[2] += 50
        self.robot.movej(lift, vel=10, acc=10)
        self.get_logger().info("테이프 살짝 들어올림")

        # 4. 대각선 방향 movec
        self.robot.movec(lift, self.tape_end, vel=10, acc=10)
        self.get_logger().info("movec로 대각선 테이핑 완료")

        # 5. 끝점에서 고정 (선택)
        sleep(1)
        self.get_logger().info("테이핑 종료")

# 메인 함수
def main(args=None):
    rclpy.init(args=args)

    # 로봇 연결 및 노드 초기화
    robot = RealRobot()
    node = TapeApplier(robot)

    try:
        node.apply_tape()
        rclpy.spin(node)  # RViz 마커 유지
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
