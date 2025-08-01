import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

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
            get_tool_force,
            set_digital_output,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    # 초기 위치
    home = [0, 0, 90, 0, 90, 0]

    box_pos = posj([9.669, 7.604, 91.275, -0.061, 81.164, 9.676])

    test_position = posx([699.86, 74.92, 43.33, 24.5, -179.52, 26.39])
    test_position_low = posx([699.86, 74.92, 100.0, 24.5, -179.52, 26.39])

    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")

    set_digital_output(2, OFF)
    set_digital_output(1, OFF)

    while rclpy.ok():
        # 초기 위치로 이동
        movej(home, vel=VELOCITY, acc=ACC)
        time.sleep(5)
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

        movej(box_pos, vel=VELOCITY, acc=ACC)
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

        proceed = input("▶️ 실험을 시작할까요? (y/n): ").strip().lower()
        if proceed != 'y':
            break

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
        log_duration = 3.0  # 측정 시간 (초)
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
        if fz_abs_max >= 8.0:
            print(f"🔵 결과: 단단한(hard) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")
        else:
            print(f"🟢 결과: 부드러운(soft) 물체로 판단됨 (|Fz| 최대 = {fz_abs_max:.2f}N)")

        print("———————————————")

    rclpy.shutdown()



if __name__ == "__main__":
    main()
