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
            DR_FC_MOD_REL, DR_AXIS_Z,
            check_force_condition,
            get_current_posx,
            set_digital_output,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([249.1, 102.42, 92.44, 56.33, 179.87, 58.38])
    pos2 = posx([248.98, 49.82, 92.44, 143.28, -179.59, 145.72])
    pos3 = posx([250.42, -3.83, 92.44, 129.63, -179.46, 131.96])
    pos4 = posx([299.78, -1.23, 92.44, 131.51, -179.47, 133.91])
    pos5 = posx([300.73, 50.75, 92.44, 124.34, -179.59, 126.57])
    pos6 = posx([300.33, 100.57, 92.44, 103.34, -179.64, 105.46])
    pos7 = posx([351.16, 101.07, 92.44, 88.22, -179.55, 90.52])
    pos8 = posx([351.30, 48.78, 92.44, 75.23, -179.35, 77.68])
    pos9 = posx([350.67, -0.99, 92.44, 66.6, -179.18, 69.19])

    pos10 = posx([398.87, 99.98, 92.44, 74.55, -179.2, 77.02])
    pos11 = posx([398.27, 50.8, 92.44, 65.22, -179.01, 67.66])
    pos12 = posx([398.86, 0.51, 92.44, 56.98, -178.79, 59.57])

    pos13 = posx([449.28, -0.8, 92.442, 69.42, -178.88, 71.97])
    pos14 = posx([449.97, 51.87, 92.44, 71.49, 178.84, 74.15])
    pos15 = posx([449.5, 103.15, 92.44, 81.17, -178.88, 83.69])

    pos16 = posx([500.5, 101.4, 92.44, 80.73, -178.8, 83.41])
    pos17 = posx([501.7, 50.03, 92.44, 72.59, -178.31, 75.39])
    pos18 = posx([501.33, -0.52, 92.44, 70.57, -178.08, 73.54])

    move = [pos1, pos2, pos3,pos4, pos5, pos6, pos7, pos8, pos9]
    goal = [[pos10, pos11, pos12], [pos15, pos14, pos13], [pos16, pos17, pos18]]

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    set_digital_output(1, ON)
    set_digital_output(2, OFF)    
    set_digital_output(1, OFF)


    while rclpy.ok():

        print("movej1")
        movej(JReady, vel=VELOCITY, acc=ACC)
        count_low, count_mid, count_high = 0, 0, 0

        for a in range(len(move)):
            movej(JReady, vel=VELOCITY, acc=ACC)
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            time.sleep(0.5)
            print(f"move {a}")
            movel(move[a], vel=VELOCITY, acc=ACC)

            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.1)


            while not check_force_condition(DR_AXIS_Z, max = 15):
                pass
            
            point = get_current_posx()[0][2]
            print(point)
            z = move[a][2] - point
            print(f'z = {z}')

            time.sleep(0.1)
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()

            movel(move[a], vel=VELOCITY, acc=ACC)
            # relase
            set_digital_output(2, ON)
            set_digital_output(1, OFF)

            down = move[a].copy()
            down[2] -= (z + 15)
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

            if z >= 45:
                print('LOW')
                movel(goal[2][count_high], vel=VELOCITY, acc=ACC)

                #task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])

                down_goal = goal[2][count_high].copy()
                down_goal[2] -= (z + 15)
                print(down_goal)
                movel(down_goal, vel=VELOCITY, acc=ACC)
                
                #release_compliance_ctrl()

                # relase
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                time.sleep(0.5)

                count_high += 1

            elif z >= 30:
                print('middle')
                movel(goal[1][count_mid], vel=VELOCITY, acc=ACC)

                # task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])

                down_goal = goal[1][count_mid].copy()
                down_goal[2] -= (z + 15)
                print(down_goal)
                movel(down_goal, vel=VELOCITY, acc=ACC)

                # release_compliance_ctrl()
                
                # relase
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                time.sleep(0.5)

                count_mid += 1
                
            else:
                print('HIGH')
                movel(goal[0][count_low], vel=VELOCITY, acc=ACC)
            
                

                down_goal = goal[0][count_low].copy()
                down_goal[2] -= (z + 15)
                print(down_goal)
                movel(down_goal, vel=VELOCITY, acc=ACC)

                #task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
                #release_compliance_ctrl()
                
                # relase
                set_digital_output(2, ON)
                set_digital_output(1, OFF)
                time.sleep(0.5)

                count_low += 1
            
            

            
        
    rclpy.shutdown()
if __name__ == "__main__":
    main()

# move 3
# 53.12271499633789
# z = 39.31728500366211 중간거
# move 4
# 45.048587799072266
# z = 47.39141220092773 낮은거
# move 5
# 65.02189636230469
# z = 27.41810363769531 높은거


