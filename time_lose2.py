# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,
            wait,
            stop,
            DR_FC_MOD_REL, DR_AXIS_Z,
            check_force_condition,
            get_current_posx,
            set_digital_output,
            DR_SSTOP, DR_TOOL, DR_BASE,
            move_periodic,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([442.11, -48.08, 80.0, 68.65, -179.66, 70.85])
    pos2 = posx([545.52, -42.09, 80.0, 72.86, -179.63, 74.87])
    pos3 = posx([501.89, -132.39, 80.0, 55.03, -179.39, 57.12])

    pos4 = posx([495.37, -73.28, 80.0, 48.94, -179.31, 50.87])

    pos5 = posx([403.3, 48.4, 80.0, 47.85, -179.1, 49.59])
    pos6 = posx([297.85, 40.64, 80.0, 47.81, -178.91, 49.41])
    pos7 = posx([342.46, 136.35, 80.0, 49.78, -178.73, 51.25])

    pos8 = posx([347.24, 77.71, 80.0, 49.81, -178.53, 51.29])

    move = [pos1, pos2, pos3, pos4]
    goal = [pos5, pos6, pos7, pos8]

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    set_digital_output(1, OFF)
    set_digital_output(2, OFF)  
    set_digital_output(2, ON) 
    set_digital_output(2, OFF) 

    while rclpy.ok():

        print("movej1")
        movej(JReady, vel=VELOCITY, acc=ACC)
    
        for a in range(len(move)):
            movej(JReady, vel=VELOCITY, acc=ACC)
            print(f"move {a}")
            movel(move[a], vel=VELOCITY, acc=ACC)

            down = move[a].copy()
            down[2] -= 40.0
            movel(down, vel=VELOCITY, acc=ACC)

            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.1)

            #grip
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            time.sleep(0.5)
            release_compliance_ctrl()

            movel(move[a], vel=VELOCITY, acc=ACC)

            movej(JReady, vel=VELOCITY, acc=ACC)

            print(f"goal {a}")
            movel(goal[a], vel=VELOCITY, acc=ACC)

            if a==3:
                task_compliance_ctrl([500, 500, 500, 100, 100, 100])
                time.sleep(0.1)
                set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                time.sleep(0.1)

                down_goal = goal[a].copy()
                down_goal[2] -= 40.0
                movel(down_goal, vel=VELOCITY, acc=ACC)

                if check_force_condition(DR_AXIS_Z, max = 15):
                    stop(DR_SSTOP)

                force_condition = check_force_condition(DR_AXIS_Z, max = 15)

                while(force_condition):
                    move_periodic(amp=[0,0,0,0,0,15], period=1, atime=0.2, repeat=1, ref=DR_TOOL)
                    if check_force_condition(DR_AXIS_Z, min=5, ref=DR_BASE):
                        break
                
                # relase
                    set_digital_output(2, ON)
                    set_digital_output(1, OFF)

                    time.sleep(0.5)
                    release_force()
                    time.sleep(0.1)
                    release_compliance_ctrl()

                    movel(goal[a], vel=VELOCITY, acc=ACC)


            else:

                down_goal = goal[a].copy()
                down_goal[2] -= 40.0
                movel(down_goal, vel=VELOCITY, acc=ACC)

                task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
                time.sleep(0.1)

                # relase
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                time.sleep(0.5)
                release_compliance_ctrl()

                movel(goal[a], vel=VELOCITY, acc=ACC)



    rclpy.shutdown()
if __name__ == "__main__":
    main()