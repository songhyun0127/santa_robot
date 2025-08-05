'''
코드 간단 정리본

WaypointManager : 현재 116번째 줄에 파일 경로 수정해주세요. (업데이트 할 때 마다 주의해주세요. )
전에 했던 프로젝트와 동일하게 json 읽고 위치를 변수에 저장해서 따로 저장했습니다. 
위치는 json 파일을 변경해주세요. 
group도 작동합니다만, 확인 부탁드립니다. 

release(), grip(), force_end()로 코드 간단화 진행 했습니다. 
release(tm), grip(tm)은 time.sleep(tm)합니다. 

화요일 오전 중, 최적화는 노드 관리와 force_start(args 조절)로 진행해보겠습니다. 
release()와 grip()에 wait_digital_input()은 안되는지 테스트 부탁드립니다. 
현재 위치의 주석 제외 300줄 안으로 정리하였습니다. 

궁금한점으로 초기에 VELOCITY, ACC = 60, 60과
set_velx = 60, set_accx= 60의 차이점에 대해 설명을 듣는 시간이 필요할 것 같습니다. 
'''

import rclpy
import DR_init
import time
from rclpy.node import Node
from std_msgs.msg import String
import queue
from .nav_waypoint import WaypointManager

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

order_queue = queue.Queue()

def order_callback(msg):
    order = msg.data.strip().lower()
    if order in ['인형', '레고']:
        print(f"주문 수신: {order}")
        order_queue.put(order)
    else:
        print(f"알 수 없는 주문: {msg.data}")

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
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            movec,
            movesx,
            get_tool_force,
            get_digital_input,
            set_digital_output,
            amove_periodic,
            moveb,
            
            
            DR_FC_MOD_REL, 
            DR_AXIS_Z,
            DR_BASE, DR_TOOL, DR_MVS_VEL_CONST,
            DR_MV_MOD_ABS,
            DR_LINE, DR_CIRCLE,
        )
        from DR_common2 import posx, posj, posb

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    # def wait_digital_input(sig_num):
    #     while not get_digital_input(sig_num):
    #         time.sleep(0.5)
    #         print(f"Wait for digital input: {sig_num}")
    #         pass

    def release(tm=0.0):
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        time.sleep(tm)
        # wait_digital_input(2)

    def grip(tm=0.0):
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        time.sleep(tm)
        # wait_digital_input(1)

    def release_soft(tm=0.0):
        set_digital_output(2, ON)
        set_digital_output(1, ON)
        time.sleep(tm)
        # wait_digital_input(2)

    def grip_soft(tm=0.0):
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        time.sleep(tm)
        # wait_digital_input(1)

    def force_end():
        time.sleep(0.1)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()
        print("force_control_end")
        print("===================================")

    def place(box_pos):
        down_th = box_pos.copy()
        down_th[2] -= 100
        seg11 = posb(DR_LINE, test_position_low, radius=20)
        seg12 = posb(DR_LINE, box_pos, radius=20)
        seg13 = posb(DR_LINE, down_th, radius=20)
        b_list = [seg11, seg12, seg13]
        moveb(b_list, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # 🧭 JSON 로드
    wp = WaypointManager("/home/rokey/ros2_ws/src/DoosanBootcam3rdCo1/dsr_rokey/rokey/rokey/project/waypoint.json")
    home = wp.get_pos("home")
    box_pos = wp.get_pos("box_pos")
    test_position = wp.get_pos("test_position")
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

    set_digital_output(2, OFF)
    set_digital_output(1, OFF)
    cnt = 0
    print("===================================")
    print("산타 로봇 작동 시작...")
    while rclpy.ok():
        # 초기 위치로 이동
        ###########################################################
        movej(home, vel=VELOCITY, acc=ACC)
        release(1)

        ###########################################################
        rclpy.spin_once(node, timeout_sec=0.1)
        if order_queue.empty():
            time.sleep(0.1)
            continue

        current_order = order_queue.get()
        print(f"주문 처리 시작: {current_order}")
        print("===================================")

        ###########################################################
        if cnt == 0:
            movel(box_pos, vel=VELOCITY, acc=ACC)
            print("force_control_start")

            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.1)
            while not check_force_condition(DR_AXIS_Z, max=10):
                pass

            force_end()
            cnt = 1

        ###########################################################
        # 초기 위치 이동
        print("초기 자세 이동 중...")
        movej(home, vel=VELOCITY, acc=ACC)

        grip()

        print("실험 위치로 이동 중...")
        movel(test_position_low, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(test_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 힘 제어 진입
        print("순응 제어 시작")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 힘 측정
        print("힘 측정 중...")
        log_duration = 10.0  # 측정 시간 (초)
        log_interval = 0.05
        force_log = []
        start_time = time.time()

        while time.time() - start_time < log_duration:
            force = get_tool_force()
            fz = force[2]
            force_log.append(fz)
            time.sleep(log_interval)

        # 제어 해제
        force_end()

        # 분류
        fz_abs_max = max(abs(f) for f in force_log)
        result_type = "레고" if fz_abs_max >= 10.0 else "인형"

        if result_type == current_order:
            cnt = 0
            if fz_abs_max >= 10.0:
                print(f"결과: 단단한(hard) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                print(f"주문 일치: {result_type} → 픽앤플레이스 수행")
                movel(test_position, vel=VELOCITY, acc= ACC)
                
                # release
                release(2)

                down = test_position.copy()
                down[2] -= 30
                movel(down, vel=VELOCITY, acc= ACC)

                # grip
                grip(2)

                place(box_pos)

                # release
                release(2)

            else:
                print(f"결과: 부드러운(soft) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                print(f"주문 일치: {result_type} → 픽앤플레이스 수행")
                movel(test_position, vel=VELOCITY, acc= ACC)
                
                # release
                release_soft(2)

                down = test_position.copy()
                down[2] -= 35
                movel(down, vel=VELOCITY, acc= ACC)

                # grip
                grip_soft(2)

                place(box_pos)

                # release
                release_soft(2)

            print("플레이스 작업 완료")
            print("===================================")
            print("박스 커버 결합 시작...")
            
            movesx(box_top, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)

            # grip
            grip_soft(3)

            movesx(box_close, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)

            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.1)
            amove_periodic(amp=[0,0,0,0,0,1.5], period=1.0, atime=0.2, repeat=15, ref=DR_TOOL)
            while not check_force_condition(DR_AXIS_Z, max=10):
                time.sleep(0.1)
                pass

            print("박스 커버 결합 완료")
            
            force_end()
            release(0.5)

            print("테이핑 작업 시작...")
            movel(box_tape1,vel=100, acc=100)
            time.sleep(0.1)
            movel(box_tape2,vel=100, acc=100)

            # grip
            grip(1)

            movec(box_tape3, box_tape4, vel=VELOCITY, acc=ACC, radius=50)
            time.sleep(0.1)

            # release
            release(1)
            movel(box_tape5, VELOCITY, ACC)
            print("테이핑 작업 완료")

            print("===================================")
            print('주문 대기중..')

        else:
            print(f"주문 불일치: 주문={current_order}, 감지={result_type}")
            print("잘못된 물체입니다.")
            movej(home, vel=VELOCITY, acc=ACC)
            print("홈으로 복귀 후 대기 중...")

        ############################################################################
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)

    rclpy.shutdown()

if __name__ == "__main__":
    main()