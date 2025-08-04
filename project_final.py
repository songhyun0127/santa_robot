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

OFF, ON = 0, 1

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
            set_digital_output,
            amove_periodic,

            DR_FC_MOD_REL, 
            DR_AXIS_Z,
            DR_BASE, DR_TOOL, DR_MVS_VEL_CONST,
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    # 초기 위치
    home = [0, 0, 90, 0, 90, 0]

    box_pos = posx([409.55, 78.39, 135.12, 147.32, 179.96, 146.68])

    test_position = posx([699.86, 74.92, 43.33, 24.5, -179.52, 26.39])
    test_position_low = posx([699.86, 74.92, 100, 24.5, -179.52, 26.39])

    box_top1 = posx([546.56, 72.74, 166.93, 0.51, 180, 3.46])
    box_top2 = posx([546.56, 72.74, 18.64, 5.94, -180, 8.89])
    box_top3 = posx([404.39, 75.94, 166.93, 174, -180, 177.01])
    box_top4 = posx([400.54, 75.94, 105.7, 9.08, -180, 12.03])

    box_top = [box_pos, box_top1, box_top2]
    box_close = [box_top1, box_top3]

    box_tape1 = posx([396.16, -46.80, 166.49, 170.34, 180, 170.49])
    box_tape2 = posx([396.16, -46.80, 6.28, 173.34, -180, 173.59])
    box_tape3 = posx([397.28, 115.59, 208.18, 28.22, 180, 28.36])
    box_tape4 = posx([396.47, 178.96, 42.6, 23.41, 180, 22.55])
    box_tape5 = posx([396.47, 178.96, 110, 23.41, 180, 22.55])


    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")

    set_digital_output(2, OFF)
    set_digital_output(1, OFF)
    cnt = 0
    print("👉 산타 로봇 작동 시작...")

    while rclpy.ok():
        # 초기 위치로 이동
        ###########################################################
        
        movej(home, vel=VELOCITY, acc=ACC)
        time.sleep(1)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

        ###########################################################
        rclpy.spin_once(node, timeout_sec=0.1)
        if order_queue.empty():
            time.sleep(0.1)
            continue

        current_order = order_queue.get()
        print(f"🔧 주문 처리 시작: {current_order}")

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

            time.sleep(0.1)
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()
            print("force_control_end")
            cnt = 1

        ###########################################################

        # 초기 위치 이동
        print("👉 초기 자세 이동 중...")
        movej(home, vel=VELOCITY, acc=ACC)

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        print("👉 실험 위치로 이동 중...")
        movel(test_position_low, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(test_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 힘 제어 진입
        print("🧠 순응 제어 시작")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        print("⬇️ Z축 하중 -10N 적용 중")
        set_desired_force(
            fd=[0, 0, -10, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )

        # 힘 측정
        print("📡 힘 측정 중...")
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
        print("🧹 힘 해제 중...")
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()

        # 분류
        fz_abs_max = max(abs(f) for f in force_log)
        result_type = "레고" if fz_abs_max >= 10.0 else "인형"

        if result_type == current_order:
            print(f"✅ 주문 일치: {result_type} → 픽앤플레이스 수행")
            cnt = 0
            if fz_abs_max >= 10.0:
                print(f"🔵 결과: 단단한(hard) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                movel(test_position, vel=VELOCITY, acc= ACC)
                
                # release
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                time.sleep(2.0)
                

                down = test_position.copy()
                down[2] -= 30
                movel(down, vel=VELOCITY, acc= ACC)

                # grip
                set_digital_output(1, ON)
                set_digital_output(2, OFF)
                
                time.sleep(2.0)

                # begin_blend(radius=30)  
                movel(test_position_low, vel=VELOCITY, acc= ACC)
                movel(box_pos, vel= VELOCITY, acc = ACC)
                down_th = box_pos.copy()
                down_th[2] -= 100
                movel(down_th, vel=VELOCITY, acc= ACC)
                # end_blend()

                # release
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                
                time.sleep(2.0)


            else:
                print(f"🟢 결과: 부드러운(soft) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
                
                movel(test_position, vel=VELOCITY, acc= ACC)
                
                # release
                set_digital_output(2, ON)
                set_digital_output(1, ON)
                time.sleep(2.0)

                down = test_position.copy()
                down[2] -= 35
                movel(down, vel=VELOCITY, acc= ACC)

                # grip
                set_digital_output(2, OFF)
                set_digital_output(1, OFF)
                time.sleep(2.0)

                movel(test_position_low, vel=VELOCITY, acc= ACC)
                movel(box_pos, vel= VELOCITY, acc = ACC)
                down_th = box_pos.copy()
                down_th[2] -= 100
                movel(down_th, vel=VELOCITY, acc= ACC)

                # release
                set_digital_output(2, ON)
                set_digital_output(1, ON)
                
                time.sleep(2.0)

            movesx(box_top, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)

            # grip
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            
            time.sleep(3)

            movesx(box_close, vel=VELOCITY, acc=ACC, time=10, vel_opt=DR_MVS_VEL_CONST)

            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.1)
            amove_periodic(amp=[0,0,0,0,0,1.5], period=1.0, atime=0.2, repeat=15, ref=DR_TOOL)
            while not check_force_condition(DR_AXIS_Z, max=10):
                time.sleep(0.1)
                pass

            time.sleep(0.1)
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()

            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            time.sleep(0.5)

            movel(box_tape1,vel=100, acc=100)
            time.sleep(0.1)
            movel(box_tape2,vel=100, acc=100)

            # grip
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            time.sleep(1)

            movec(box_tape3, box_tape4, vel=VELOCITY, acc=ACC, radius=50)
            time.sleep(0.1)

            # release
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            time.sleep(1)

            movel(box_tape5, VELOCITY, ACC)

            print("———————————————")
            print('🏠 주문 대기중..')

        else:
            print(f"❌ 주문 불일치: 주문={current_order}, 감지={result_type}")
            print("⚠️ 사용자에게 알림: 잘못된 물체입니다.")
            movej(home, vel=VELOCITY, acc=ACC)
            print("🏠 홈으로 복귀 후 대기 중...")

        ############################################################################

    rclpy.shutdown()



if __name__ == "__main__":
    main()