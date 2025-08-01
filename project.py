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

    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")

    while rclpy.ok():
        # 초기 위치로 이동
        movej(home, vel=VELOCITY, acc=ACC)
        time.sleep(5)
        movej(box_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
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

    rclpy.shutdown()



if __name__ == "__main__":
    main()
