'''
README

v2 변경점
basic_def.py 생성하여 함수 이동 및 몇몇 코드줄 삭제. 
test_position 위치 변경 / 조금 올렸습니다. 
불일치 시 home 대기 대신 test_position에서 대기로 코드 변경했습니다. 
v2의 set_accx, set_velx 안됩니다. 
다시 수정 전으로 고쳤습니다. 
--------------------------------------------------------------
업데이트시 주의사항 : 
1. WaypointManager : 현재 94번째 줄에 파일 경로 수정해주세요. (업데이트 할 때 마다 주의해주세요. )
전에 했던 프로젝트와 동일하게 json 읽고 위치를 변수에 저장해서 따로 저장했습니다. 
위치는 json 파일을 변경해주세요. 
group도 작동합니다만, 확인 부탁드립니다. 
--------------------------------------------------------------
개인 코멘트 : 
(2, on), (1, on)
(2, off), (1, off) 확인 했습니다. 
--------------------------------------------------------------
시나리오(move도 적어주면 좋음) :
1. 토픽 구독
2. 주문을 받으면
3. 박스 공정(cnt + 1)
4. 상품 강성 체크
5. 불일치 시 대기 장소로 이동 후 5-1로 / 일치 시 상승 및 5-2로
5-1. 토픽 재 구독
6-1. cnt가 1이라 박스 공정 무시 및 4~6-1 반복

5-2. 상품을 집고 박스에 넣고 그리퍼 off
6-2. 박스를 잡고 상자 위로
7-2. periodic 을 비동기로 하고, 힘 체크 후 박스 완성
8-2. 테이프가 감겨진 블럭 위치로
9-2. 잡고 특정 위치로 이동 및 그리퍼 off
시나리오 end
'''

import rclpy
import DR_init
import time
from rclpy.node import Node
from std_msgs.msg import String
import queue


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

order_queue = queue.Queue()

def order_callback(msg):
    order = msg.data.strip().lower()
    if order in ['인형', '레고']:
        print(f"📨 주문 수신: {order}")
        order_queue.put(order)
    else:
        print(f"⚠️ 알 수 없는 주문: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    node.create_subscription(
        String,
        '/dsr01/order_info',
        order_callback,
        10
    )

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            check_force_condition,
            set_tool, set_tcp,
            movej, movel, movec, movesx,
            get_tool_force,
            amove_periodic,
            DR_AXIS_Z,
            DR_BASE, DR_TOOL, DR_MVS_VEL_CONST,
        )
        from .nav_waypoint import WaypointManager
        from .basic_def import release, grip, force_end, grip_soft, release_soft, force_start

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 🧭 JSON 로드코드 간단 정리본
    wp = WaypointManager("/path/waypoint.json")
    home = wp.get_pos("home")
    box_pos = wp.get_pos("box_pos")
    test_position = wp.get_pos("test_position")
    test_position_2 = wp.get_pos("test_position_2")
    test_position_low = wp.get_pos("test_position_low")

    box_top = wp.get_group("box_top")
    box_close = wp.get_group("box_close")

    box_tape1 = wp.get_pos("box_tape1")
    box_tape2 = wp.get_pos("box_tape2")
    box_tape3 = wp.get_pos("box_tape3")
    box_tape4 = wp.get_pos("box_tape4")
    box_tape5 = wp.get_pos("box_tape5")

    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")
    grip_soft()
    cnt = 0
    print("👉 산타 로봇 작동 시작...")
    while rclpy.ok():
        # 초기 위치로 이동
        movej(home, vel=VELOCITY, acc=ACC)
        release(1)

        rclpy.spin_once(node, timeout_sec=0.1)
        if order_queue.empty():
            time.sleep(0.1)
            continue

        current_order = order_queue.get()
        print(f"🔧 주문 처리 시작: {current_order}")

        if cnt == 0:
            movel(box_pos, vel=VELOCITY, acc=ACC)
            force_start(100, -20)
            while not check_force_condition(DR_AXIS_Z, max=10):
                pass
            force_end()
            cnt = 1

        grip()
        # 초기 위치 이동
        print("👉 초기 자세 이동 중...")
        movej(home, vel=VELOCITY, acc=ACC)

        print("👉 실험 위치로 이동 중...")
        movel(test_position_low, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(test_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 힘 제어 진입
        force_start(500, -10)

        force_log = []
        start_time = time.time()

        while time.time() - start_time < 10.0:
            force = get_tool_force()
            fz = force[2]
            force_log.append(fz)
            time.sleep(0.05)

        # 제어 해제
        force_end()

        # 분류
        fz_abs_max = max(abs(f) for f in force_log)
        result_type = "레고" if fz_abs_max >= 10.0 else "인형"

        if result_type == current_order:
            print(f"✅ 주문 일치: {result_type} → 픽앤플레이스 수행")
            cnt = 0
            if fz_abs_max >= 10.0:
                print(f"🔵 결과: 단단한(hard) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                movel(test_position, vel=VELOCITY, acc=ACC)
                
                # release
                release(2)
                down = test_position.copy()
                down[2] -= 30
                movel(down, vel=VELOCITY, acc=ACC)

                # grip
                grip(2)

                # begin_blend(radius=30)  
                movel(test_position_low, vel=VELOCITY, acc=ACC)
                movel(box_pos, vel=VELOCITY, acc=ACC)
                down_th = box_pos.copy()
                down_th[2] -= 100
                movel(down_th, vel=VELOCITY, acc=ACC)
                # end_blend()

                # release
                release(2)

            else:
                print(f"🟢 결과: 부드러운(soft) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                movel(test_position, vel=VELOCITY, acc=ACC)

                # release
                release_soft(2)

                down = test_position.copy()
                down[2] -= 35
                movel(down, vel=VELOCITY, acc=ACC)

                # grip
                grip_soft(2)

                movel(test_position_low, vel=VELOCITY, acc=ACC)
                movel(box_pos, vel=VELOCITY, acc=ACC)
                down_th = box_pos.copy()
                down_th[2] -= 100
                movel(down_th, vel=VELOCITY, acc=ACC)

                # release
                release_soft(2)
                
            movesx(box_top, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)

            # grip
            grip(3)

            movesx(box_close, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)
            force_start(100, -15)

            amove_periodic(amp=[0,0,0,0,0,1.5], period=1.0, atime=0.2, repeat=15, ref=DR_TOOL)
            while not check_force_condition(DR_AXIS_Z, max=10):
                time.sleep(0.1)
                pass

            force_end()
            release(0.5)

            movel(box_tape1,vel=100, acc=100)
            time.sleep(0.1)
            movel(box_tape2,vel=100, acc=100)

            # grip
            grip(1)

            movec(box_tape3, box_tape4, vel=VELOCITY, acc=ACC, radius=50)
            time.sleep(0.1)

            # release
            release(1)
            movel(box_tape5, vel=VELOCITY, acc=ACC)

            print("———————————————")
            print('🏠 주문 대기중..')

        else:
            print(f"❌ 주문 불일치: 주문={current_order}, 감지={result_type}")
            print("⚠️ 사용자에게 알림: 잘못된 물체입니다.")
            movel(test_position, vel=VELOCITY, acc=ACC)
            print("⚠️ 일치하는 물건을 놔주세요. ")

        ############################################################################
        grip_soft()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
