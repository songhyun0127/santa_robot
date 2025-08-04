import rclpy
import DR_init
import time
from std_msgs.msg import String

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

gift_command = None  # 전역 선언

def gift_callback(msg):
    global gift_command
    gift_command = msg.data
    print(f"[ROS2 수신] 포장 명령: {gift_command}")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

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


    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")

    set_digital_output(2, OFF)
    set_digital_output(1, OFF)
    
    sub = node.create_subscription(String, '/gift/command', gift_callback, 10)

    while rclpy.ok():
        # 초기 위치로 이동
        movej(home, vel=VELOCITY, acc=ACC)
        time.sleep(1)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

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

        movej(home, vel=VELOCITY, acc=ACC)

        print("🕓 GUI 명령 수신 대기 중...")
        gift_command = None
        while rclpy.ok() and gift_command is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if gift_command == "doll":
            print("인형 포장 루틴 실행")
        elif gift_command == "lego":
            print("레고 포장 루틴 실행")
        else:
            print("⚠️ 알 수 없는 명령, 다음 명령 대기")
            continue

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        # 초기 위치 이동
        print("👉 초기 자세 이동 중...")
        movej(home, vel=VELOCITY, acc=ACC)

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
            print(f"   ▶ Fz: {fz:.2f} N")
            time.sleep(log_interval)

        # 제어 해제
        print("🧹 힘 해제 중...")
        release_force()
        time.sleep(0.3)
        release_compliance_ctrl()

        # 분류
        fz_abs_max = max(abs(f) for f in force_log)
        if fz_abs_max >= 10.0:
            print(f"🔵 결과: 단단한(hard) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
            movel(test_position, vel=VELOCITY, acc= ACC)
            
            # release
            set_digital_output(2, ON)
            set_digital_output(1, OFF)

            down = test_position.copy()
            down[2] -= 30
            movel(down, vel=VELOCITY, acc= ACC)

            # grip
            set_digital_output(2, OFF)
            set_digital_output(1, ON)

            movel(test_position_low, vel=VELOCITY, acc= ACC)
            movel(box_pos, vel= VELOCITY, acc = ACC)
            down_th = box_pos.copy()
            down_th[2] -= 100
            movel(down_th, vel=VELOCITY, acc= ACC)

            # release
            set_digital_output(2, ON)
            set_digital_output(1, OFF)


        else:
            print(f"🟢 결과: 부드러운(soft) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
            
            movel(test_position, vel=VELOCITY, acc= ACC)
            
            # release
            set_digital_output(2, ON)
            set_digital_output(1, ON)
            time.sleep(1)

            down = test_position.copy()
            down[2] -= 35
            movel(down, vel=VELOCITY, acc= ACC)

            # grip
            set_digital_output(2, OFF)
            set_digital_output(1, OFF)
            time.sleep(1)

            movel(test_position_low, vel=VELOCITY, acc= ACC)
            movel(box_pos, vel= VELOCITY, acc = ACC)
            down_th = box_pos.copy()
            down_th[2] -= 100
            movel(down_th, vel=VELOCITY, acc= ACC)

            # release
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            time.sleep(1)

        movesx(box_top, vel=VELOCITY, acc=ACC, time=5, vel_opt=DR_MVS_VEL_CONST)

        # grip
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        time.sleep(3)

        movesx(box_close, vel=VELOCITY, acc=ACC, time=5, vel_opt=DR_MVS_VEL_CONST)

        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        amove_periodic(amp=[0,0,0,0,0,1.5], period=1.0, atime=0.2, repeat=15, ref=DR_TOOL)
        while not check_force_condition(DR_AXIS_Z, max=10):
            print(check_force_condition(DR_AXIS_Z, max=10))
            
            time.sleep(0.1)
            print('periodic end')
            pass

        time.sleep(0.1)
        release_force()
        time.sleep(0.1)
        release_compliance_ctrl()

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        print("———————————————")

    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
