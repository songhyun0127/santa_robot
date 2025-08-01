'''
v2 : force_based_movement 성능 파악 및 최적화 진행
'''

# pick and place in 1 method. from pos1 to pos2 @20241104
import time
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # set_digital_output,
            get_current_posx,
            get_current_posj,
            # get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            # DR_BASE,
        )

        from DR_common2 import posj, posx
    
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # def wait_digital_input(sig_num):
    #     while not get_digital_input(sig_num):
    #         time.sleep(0.5)
    #         print(f"Wait for digital input: {sig_num}")
    #         pass

    # def release():
    #     print("set for digital output 0 1 for release")
    #     set_digital_output(2, ON)
    #     set_digital_output(1, OFF)
    #     wait_digital_input(2)

    # def grip():
    #     print("set for digital output 1 0 for grip")
    #     set_digital_output(1, ON)
    #     set_digital_output(2, OFF)
    #     wait_digital_input(1)

    def force_based_movement(max_force=10):
        """
        주어진 조건에 맞게 이동하면서 힘이 가해지면 멈추는 함수.
        :param position_obj: 초기 위치 (목표 위치)
        :param pose_type: pose / posx
        :param VELOCITY: 이동 속도
        :param ACC: 가속도
        :param max_force: 최대 허용 힘 (기본값은 5)
        """
        # 순응 제어 시작
        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.5)
        # 힘 설정
        print("Starting set_desired_force")
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("Moving and checking force condition...")
        # 힘이 감지될 때까지 이동
        while not check_force_condition(DR_AXIS_Z, max=max_force):
            pass
            
            
        # 힘 해제
        print("Starting release_force")
        release_force()
        time.sleep(0.5)
        # 순응 제어 해제
        print("Starting release_compliance_ctrl")
        release_compliance_ctrl()
        print("Movement complete, force released.")

    def modify_position_value(position_obj, index,  delta, pose_type):
        """
        :param position_obj: posx 또는 posj 객체
        :param index: 바꾸고자 하는 인덱스 번호
        :param delta: 조정할 상대적인 변화량 (양수/음수)
        :param pose_type: 'posx' 또는 'posj' 객체를 구분하는 타입 문자열

        posx일때는 고정값으로 이동 / posj일때는 joint를 더하기?
        """
        # posx/posj 객체를 리스트로 변환하여 수정
        position_list = list(position_obj)
        
        # posx 객체인지 확인(나중엔 type(position_obj) 로 검증 가능)
        if pose_type == "posx":
            position_list[index] = delta
            # 수정된 posx 객체 반환
            modified_position = posx(position_list)
        
        # posj 객체인지 확인(나중엔 type(position_obj) 로 검증 가능)
        elif pose_type == "posj":
            position_list[index] = position_list[index] + delta
            # 수정된 posj 객체 반환
            modified_position = posj(position_list)
        
        else:
            raise TypeError("Unsupported pose_type: pose_type must be 'posx' or 'pose'")
        
        return modified_position

    

    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")
    # 박스 위치
    box_pos = posj([9.669, 7.604, 91.275, -0.061, 81.164, 9.676])


    while rclpy.ok():
        # 시작
        print('move start')
        Ready_j = posj([0, 0, 90, 0, 90, 0])
        
        # 홈으로 이동
        movej(Ready_j, vel=VELOCITY, acc=ACC)
        
        movej(box_pos, vel=VELOCITY, acc=ACC)

        # 6번째 조인트를 45도 회전 조절하고 이동
        modify_Ready_j = modify_position_value(box_pos, 5, 45, 'posj')
        movej(modify_Ready_j, vel=VELOCITY, acc=ACC)
        
        # 현재 posx 좌표의 2번째인덱스(높이) 조절
        Ready_x= posx(get_current_posx()[0])
        modify_Ready_x = modify_position_value(Ready_x, 2, 20, 'posx')
        movel(modify_Ready_x, vel=VELOCITY, acc=ACC)

        # 힘이 느껴질때까지 이동 후 닿았다면 posx 좌표 얻어오기
        force_based_movement(max_force=5)
        touch_ground_x = posx(get_current_posx()[0])
        
        # 2번째인덱스(높이) 가 10 이상이라면 안닿은것
        if touch_ground_x[2] > 10:
            print("doesnt touch ground")
        
        # 2번째인덱스(높이) 가 10 이하라면 땅에 닿은것
        else :
            print("touch ground")

            # 2번째인덱스 (높이) 를 20으로 올리고, 이동 및 조인트[5]를 (전에 45로 돌렸으므로) -90도 회전
            modify_tg_x = modify_position_value(touch_ground_x, 2, 20, 'posx')
            movel(modify_tg_x, vel=VELOCITY, acc=ACC)
            touch_ground_j = posj(get_current_posj())
            modify_stg_j = modify_position_value(touch_ground_j, 5, -90, 'posj')
            movej(modify_stg_j, vel=VELOCITY, acc=ACC)

            # 이후 다시 현재 posx 좌표의 2번째인덱스(높이) 조절
            go_ground = posx(get_current_posx()[0])
            modify_go_ground = modify_position_value(go_ground, 2, 20, 'posx')

            # 힘이 느껴질때까지 이동 후 닿았다면 posx 좌표 얻어오기
            force_based_movement(max_force=5)
            second_tg_x = posx(get_current_posx()[0])

            # 2번째인덱스(높이) 가 10 이상이라면 안닿은것
            if second_tg_x[2] > 10:
                print("doesnt touch ground")

            # 2번째인덱스(높이) 가 10 이하라면 땅에 닿은것
            else :
                print("touch ground")

        # 팔 올리기
        finish_x= posx(get_current_posx()[0])
        modify_finish_x = modify_position_value(finish_x, 2, 90, 'posx')
        movel(modify_finish_x, vel=VELOCITY, acc=ACC)
        # 마무리로 [0, 0, 90, 0, 90, 0] 보낸 후 종료        
        movej(Ready_j, vel=VELOCITY, acc=ACC)
        # wait(0.5)
        print('finish')
    rclpy.shutdown()

if __name__ == "__main__":
    main()



'''
issue_solution

1. posj, posx 가 튜플인지 몰랐음
그래서 값을 변환하고 movej, movel 했을 때 이동하지 않음
근데 그러면 어떻게 j로 회전은 잘 한건지 모르겠음
x만 튜플인지 확인 해봐야 알 듯
------
posx 는 [0] 으로 가져와야 했는데
posj 는 조인트가 고정이라 sol이 없음 그래서 [0] 이 없어야 했다. 


2. get_current_posj 가 0, 0, 0, 0, 0, 0 이 나옴
------
위 이슈와 같은 솔루션으로 해결



'''