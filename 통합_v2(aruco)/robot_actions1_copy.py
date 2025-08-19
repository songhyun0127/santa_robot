import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 120, 120

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT2 import set_digital_output, wait
ON, OFF = 1, 0

try:
    from DSR_ROBOT2 import movej, movel, move_periodic, DR_TOOL
    from DR_common2 import posx
except ImportError as e:
    print(f"❌ 로봇 제어 모듈 import 실패: {e}")
        

JReady = [0, 0, 90, 0, 90, 0]
pos1 = posx([498.017, 136.581, 262.157, 155.001, 179.977, 154.733])
# 후라이팬 관련 좌표
pos2 = posx([625.034, -108.247, 175.739, 166.652, -177.763, -99.833])  # 후라이팬 잡고 대기
pos3 = posx([612.206, -107.86, 22.175, 169.891, 179.714, -96.326])     # 후라이팬 잡는 위치
pos4 = posx([344.663, -3.943, 31.173, 170.539, -173.775, -96.126])     # 후라이팬 화구 위치
pos5 = posx([344.663, -3.943, 175.739, 170.539, -173.775, -96.126])    # 후라이팬 화구 대기 위치
# 삼겹살 관련 좌표
pos6 = posx([467.885, 188.921, 203.858, 172.293, 180.0, -94.339])      # 삼겹살 대기 위치
pos7 = posx([467.885, 188.921, 69.000, 173.651, 180.0, -92.981])       # 삼겹살 잡는 위치
pos8 = posx([469.839, -10.413, 203.858, 163.304, 180.0, -103.329])     # 삼겹살 후라이팬 대기 위치
pos9 = posx([469.839, -10.413, 23.834, 163.304, 180.0, -103.329])      # 삼겹살 후라이팬 투입 위치
pos80 = posx([410.55, -37.26, 198.51, 0.33, 180.0, 90.76])             # 삼겹살 detect 위치

pos18 = posx([622.781, 54.926, 216.089, 15.285, 179.803, 105.45])
pos19 = posx([627.042, 53.44, 36.186, 6.584, 179.856, 96.859])

# 좌표 정의
pos10 = posx([504.799, 141.384, 261.723, 90.286, 157.489, 90.01])       # 디텍션 위치
pos11 = posx([525.348, 288.057, 216.677, 25.359, 179.999, 28.327])      # 대기 위치
pos12 = posx([525.348, 288.057, 35.429, 25.358, 179.999, 28.326])       # 잡기 위치
pos13 = posx([469.867, 176.119, 216.677, 25.36, 179.999, 28.328])       # 뿌리기 대기 위치
pos14 = posx([471.895, 91.792, 122.659, 89.093, 114.77, 92.488])        # 뿌리기 동작 위치

# 후라이팬 완료 좌표 정의
pos16 = posx([326.472, -80.0, 175.739, 83.834, -180.0, -131.293])  # 구이 완료 대기 위치
pos17 = posx([326.472, -80.0, 41.182, 83.834, -180.0, -131.293])   # 구이 완료 위치
pos20 = posx([375.488, -14.195, 176.000, 39.942, 179.943, 130.029]) # 냄비 화구 대기 위치
pos21 = posx([375.488, -14.195, 42.148, 39.942, 179.943, 130.029]) # 냄비 화구 위치

# 김치 좌표
pos28 = posx([635.23, 177.51, 154.192, 13.056, 179.732, 12.545])  # 김치 잡기 대기 위치
pos29 = posx([635.23, 177.51, 65.207, 13.056, 179.732, 12.545])   # 김치 잡기 위치

# 김치 놓기 좌표
pos32 = posx([594.643, -198.146, 110.601, 161.517, -179.407, 160.862])  # 놓기 대기 위치
pos33 = posx([594.643, -198.146, 20.607, 161.517, -179.407, 160.862])    # 놓기 위치

# 상추
pos30 = posx([298.786, 182.991, 259.604, 89.309, 179.992, 89.11])       # 상추 집기 대기 위치
pos31 = posx([309.183, 183.184, 60.149, 38.854, 179.94, 38.644])       # 상추 집기 위치

# 상추 놓기 좌표
pos33_1 = posx([594.643, -198.146, 30.607, 161.517, -179.407, 160.862])    # 놓기 위치

# 좌표 정의
pos22 = posx([274.726, -239.755, 334.488, 90.46, -120.972, 90.27])      # 전자온도계 대기 위치
pos23 = posx([274.467, -249.064, 240.055, 90.428, -120.972, 90.255])      # 전자온도계 잡는 위치
pos23_1 = posx([274.649, -239.778, 440.955, 90.454, -120.972, 90.27])      # 전자온도계 뽑는 위치

pos24 = posx([500.823, -192.265, 198.297, 75.334, -162.511, 68.404])    # 온도계 grip, 뒤집기 1
pos25 = posx([521.823, -172.265, 340.955, 75.334, -162.511, 68.404])    # 온도계 grip, 뒤집기 1 대기 
pos35 = posx([500.823, -76.805, 198.297, 75.334, -162.511, 68.404])     # 온도계 grip, 뒤집기 2
pos36 = posx([521.823, -172.265, 188.297, 75.334, -162.511, 68.404])

# 냄비 완료 좌표 정의
pos16_1 = posx([343.816, -120.318, 175.739, 173.56, 179.939, -50.601]) # 수육 완료 대기 위치
pos17_1 = posx([336.376, -118.237, 40.853, 173.56, 179.939, -50.601])  # 수육 완료 위치


# 그립 및 릴리즈 함수
def grip():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1.0)

def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(1.0)

# 이동 함수 (movej, movel 공통)
def move_to_position(position):
    movel(position, vel=VELOCITY, acc=ACC)

def move_home():
    print("로봇을 HOME 위치로 이동합니다...")
    movej(JReady, vel=60, acc=60)

# 예시 함수들
def move_to_fridge():
    
    print("🚀 로봇을 냉장고 위치로 이동합니다...")
    move_home()
    move_to_position(pos1)


    print("🍳 후라이팬 및 삼겹살 이동 시작...")

def move_frypan():
    # 후라이팬 이동 및 배치
    print("🍳 후라이팬 위치로 이동 중...")
    move_to_position(pos2)
    move_to_position(pos3)
    grip()
    move_to_position(pos2)
    move_to_position(pos5)
    move_to_position(pos4)
    release()
    move_to_position(pos5)
    print("✅ 후라이팬 이동 완료")

def move_pork():
    # 삼겹살 이동 및 투입
    print("🥩 삼겹살 위치로 이동 중...")
    move_to_position(pos6)
    move_to_position(pos7)
    grip()
    move_to_position(pos6)
    move_to_position(pos8)
    move_to_position(pos9)
    release()
    move_to_position(pos8)
    move_to_position(pos80)
    print("✅ 삼겹살 이동 완료")

def move_boiled_pork():
    print("🍲 냄비 위치로 이동 중...")
    move_home()
    move_to_position(pos18)
    move_to_position(pos19)
    grip()
    move_to_position(pos18)
    move_to_position(pos20)
    move_to_position(pos21)
    release()
    move_to_position(pos20)

def move_thermometer():
    print("🌡️ 온도계 측정 시작...")
    move_to_position(pos22)
    move_to_position(pos23)
    grip()
    move_to_position(pos23_1)
    move_to_position(pos25)
    move_to_position(pos36)
    print("✅ 온도 측정 완료")

def back_thermometer():
    move_to_position(pos25)     # 온도계 grip, 뒤집기 1 대기
    move_to_position(pos23_1)     # 전자온도계 대기 위치
    move_to_position(pos23)     # 전자온도계 위치
    release()
    move_to_position(pos22)     # 전자온도계 대기 위치

def move_spice_detection():
    move_to_position(pos10)

def move_pepper_1():
    print("🧂 [DEBUG] move_pepper() 함수가 호출되었습니다.")
    # 조미료 잡기
    move_to_position(pos11)
    move_to_position(pos12)
    grip()
    move_to_position(pos11)

def move_pepper_2():
    print("🧂 [DEBUG] move_pepper() 함수가 호출되었습니다.")
    # 흔들기 동작 1 (이동 중 흔들기)
    move_periodic(amp=[25, 0, 10, 0, 0, 0], period=1.0, atime=0.01, repeat=2, ref=DR_TOOL)
    # 뿌리기 위치로 이동
    move_to_position(pos13)
    move_to_position(pos14)
    # 흔들기 동작 2 (실제 뿌리기)
    move_periodic(amp=[0, 30, 0, 0, 0, 0], period=1.0, atime=0.02, repeat=2, ref=DR_TOOL)
    # 복귀 및 조미료 내려놓기
    move_to_position(pos13)
    move_to_position(pos11)
    move_to_position(pos12)
    release()
    move_to_position(pos11)
    print("✅ 조미료 뿌리기 완료")

def complete_boiled_pork():
    print("✅ 수육 조리 완료 모션 시작...")
    move_home()
    move_to_position(pos20)
    move_to_position(pos21)
    grip()
    move_to_position(pos20)
    move_to_position(pos16_1)
    move_to_position(pos17_1)
    release()
    move_to_position(pos16_1)
    move_home()
    print("🎉 수육 조리 완료 위치로 이동 완료")

def complete_pork():
    print("✅ 삽겹살 조리 완료 모션 시작...")
    print("✅ 삼겹살 조리 완료 모션 시작...")
    move_to_position(pos5)      # 후라이팬 화구 대기 위치
    move_to_position(pos4)      # 후라이팬 화구 위치
    grip()
    move_to_position(pos5)      # 후라이팬 화구 대기 위치
    move_to_position(pos16)     # 조리완료 대기위치
    move_to_position(pos17)     # 조리완료 위치
    release()
    move_to_position(pos16)     # 조리완료 대기위치
    move_home()
    print("🎉 삼겹살 조리 완료 위치로 이동 완료")

def move_kimchi():
    print("🥬 김치 이동 시작...")
    # 김치 잡기
    move_to_position(pos28)
    move_to_position(pos29)
    grip()
    move_to_position(pos28)
    # 김치 놓기
    move_to_position(pos32)
    move_to_position(pos33)
    release()
    move_to_position(pos32)

    print("✅ 김치 이동 완료")

def move_lettuce():
    print("🥬 상추 이동 시작...")
    # 상추 모션
    move_to_position(pos30)     # 상추 잡기 대기 위치
    move_to_position(pos31)     # 상추 잡기 위치
    grip()
    move_to_position(pos30)     # 상추 잡기 대기 위치
    # 상추 놓기
    move_to_position(pos32)
    move_to_position(pos33_1)
    release()
    move_to_position(pos32)

    print("✅ 김치 이동 완료")

def flip_pork_1():
    print("🥩 [DEBUG] flip_pork_1() 함수가 호출되었습니다.")
    print("🥩 삼겹살 뒤집기 대기 모션 시작")
    # 뒤집기 동작
    move_to_position(pos22)     # 대기 위치로 이동
    move_to_position(pos23)     # 온도계 잡는 위치로 이동
    grip()                                  # 온도계 잡기
    move_to_position(pos23_1)     # 대기 위치로 이동
    move_to_position(pos25)     # 뒤집기 대기 위치로 이동
    print("🥩 삼겹살 뒤집기 대기 모션 완료")

def flip_pork_2():
    print("🥩 [DEBUG] flip_pork_2() 함수가 호출되었습니다.")
    print("🥩 삼겹살 뒤집기 모션 시작")
    # 뒤집기 동작
    move_to_position(pos24)     # 온도계 grip, 뒤집기 1
    move_to_position(pos35)     # 온도계 grip, 뒤집기 2
    move_to_position(pos24)     # 온도계 grip, 뒤집기 1
    # 웨이트 를 주고 일정 조건 이벤트 발생시 음성 안내 후 회수
    move_to_position(pos25)     # 온도계 grip, 뒤집기 1 대기
    move_to_position(pos23_1)     # 전자온도계 대기 위치
    move_to_position(pos23)     # 전자온도계 위치
    release()
    move_to_position(pos22)     # 전자온도계 대기 위치
    print("🥩 삼겹살 뒤집기 모션 완료")


def pork_config_pos():
    print("삼겹살 디텍션 위치 이동 시작...")
    move_to_position(pos8)