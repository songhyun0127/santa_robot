import speech_recognition as sr
from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import os
import time
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import os
import time
import rclpy
import DR_init
import cv2.aruco as aruco

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 120, 120
ON, OFF = 1, 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class VoiceAssistant(Node):
    def __init__(self):
        super().__init__('voice_assistant_node')
        self.recognizer = sr.Recognizer()
        self.microphone = self.get_usb_microphone()

    def get_usb_microphone(self):
        for index, name in enumerate(sr.Microphone.list_microphone_names()):
            if "USB Audio" in name or "C270" in name:
                print(f"🎧 USB 마이크 선택됨: {name} (index {index})")
                return sr.Microphone(device_index=index)
        print("⚠️ USB 마이크를 찾지 못했습니다. 기본 마이크를 사용합니다.")
        return sr.Microphone()

    def listen(self):
        with self.microphone as source:
            print("\n🎙️ 말씀해주세요...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            user_input = self.recognizer.recognize_google(audio, language="ko-KR")
            print(f"📝 인식된 문장: {user_input}")
            return user_input
        except sr.UnknownValueError:
            print("😕 음성을 이해하지 못했어요.")
            return None
        except sr.RequestError:
            print("⚠️ 음성 인식 서비스에 문제가 있어요.")
            return None
        
    def pork_detection(self, pork_name):
        from robot_actions1_copy import pork_config_pos
        pork_config_pos()
        model_path = '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/rokey/rokey/cooking/pork_best.pt'
        model = YOLO(model_path)

        box_color = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (255, 255, 255), (125, 125, 125)]

        # 카메라 열기 (0: 기본 내장 카메라)
        cap = cv2.VideoCapture(4)

        if not cap.isOpened():
            print("카메라를 열 수 없습니다.")
            exit()

        print("실시간 예측 시작 (종료: Q 키 누르기)")

        # 1초 동안 slice_Pork-60 확률을 80% 이상 체크
        start_time = time.time()  # 시작 시간
        pork_count = 0  # slice_Pork-60이 감지된 프레임 수
        frame_count = 0  # 처리한 프레임 수
        pork_threshold = 0.7  # 80%

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 프레임 예측 (stream=False: 프레임 1개씩 예측)
            results = model.predict(source=frame, conf=0.6, verbose=False)

            # 예측 결과 가져오기
            result = results[0]
            boxes = result.boxes  # bounding box 정보
            classes = result.names  # 클래스 이름들

            # 1초가 경과되었는지 체크
            current_time = time.time()
            if current_time - start_time >= 2:  # 2초 경과
                # 1초 동안 slice_Pork-60이 80% 이상인 프레임이 일정 비율 이상이면 종료 처리
                if pork_count / frame_count >= pork_threshold:
                    # 해당 조건에 맞으면 예측 종료
                    cap.release()
                    cv2.destroyAllWindows()
                    print("예측 종료")
                    break

                # 카운터 리셋 및 타이머 초기화
                pork_count = 0
                frame_count = 0
                start_time = time.time()  # 타이머 초기화

            # 각 프레임에서 slice_Pork-60 체크
            for box in boxes:
                cls_id = int(box.cls[0])  # 클래스 ID
                conf = float(box.conf[0]) * 100  # 신뢰도 (0~1 → 0~100%)
                label = f"{classes[cls_id]} {conf:.1f}%"

                # Bounding box 좌표
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 정수 변환

                # 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color[cls_id], 2)
                # 클래스 및 확률 표시
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color[cls_id], 2)

                # "slice_Pork-60"을 찾고 80% 이상의 확률일 경우 카운팅
                if classes[cls_id] == pork_name and conf >= pork_threshold * 100:
                    pork_count += 1

            frame_count += 1  # 처리한 프레임 수 증가

            # 화면에 출력
            cv2.imshow("YOLO Predict", frame)

            # Q 키 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 종료 처리
        cap.release()
        cv2.destroyAllWindows()
        print("예측 종료")

    def respond(self, user_input):
        from robot_actions1_copy import move_spice_detection, move_home, back_thermometer, flip_pork_1, flip_pork_2, complete_pork, pork_config_pos, move_to_fridge, move_frypan, move_pork, move_boiled_pork, complete_boiled_pork, move_thermometer, move_pepper_1, move_pepper_2, move_kimchi, move_lettuce
        from temp_detector import process_lcd_and_detect_digits
        
        if user_input and "냉장고" in user_input:
            self.speak("냉장고를 열어드리겠습니다.")
            move_to_fridge()
            model_path = '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/rokey/rokey/cooking/fridge_best.pt'
            model = YOLO(model_path)

            box_color = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), 
             (0, 255, 255), (255, 0, 255), (255, 255, 255), (125, 125, 125)]

            cap = cv2.VideoCapture(4)

            if not cap.isOpened():
                print("카메라를 열 수 없습니다.")
                exit()

            print("실시간 예측 시작 (종료: Q 키 누르기)")

            # 5초 동안 스캔하여 각 슬롯에서 가장 많이 등장한 물건을 기록
            detected_items_over_time = {"slot1": [], "slot2": [], "slot3": []}
            start_time = time.time()

            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                results = model.predict(source=frame, conf=0.6, verbose=False)
                result = results[0]
                boxes = result.boxes
                classes = result.names

                detected_items = []  # (x_center, label) 저장용 리스트

                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0]) * 100
                    label = f"{classes[cls_id]}"

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x_center = (x1 + x2) // 2  # 중심 x좌표

                    detected_items.append((x_center, label))

                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color[cls_id % len(box_color)], 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color[cls_id % len(box_color)], 2)

                # x좌표 기준으로 왼→오 정렬
                detected_items.sort(key=lambda x: x[0])

                # 냉장고 슬롯 초기화 (3칸)
                fridge_slots = ["", "", ""]

                for idx, (_, label) in enumerate(detected_items[:3]):  # 최대 3개까지만
                    fridge_slots[idx] = label

                # 각 슬롯에서 가장 많이 나타난 물건을 기록
                detected_items_over_time["slot1"].append(fridge_slots[0])
                detected_items_over_time["slot2"].append(fridge_slots[1])
                detected_items_over_time["slot3"].append(fridge_slots[2])

                # 슬롯 상태 출력
                print(f"냉장고 리스트: {fridge_slots}")

                # 5초가 지나면 중단하고, 각 슬롯의 최빈값을 반환
                if time.time() - start_time > 5:
                    break
                
                cv2.imshow("YOLO Predict", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
            print("예측 종료")

            # 각 슬롯에서 가장 많이 디텍팅된 물건을 선택
            final_slots = []
            for slot in ["slot1", "slot2", "slot3"]:
                items = detected_items_over_time[slot]
                
                # 빈 문자열을 "없음"으로 처리
                items = [item if item != "" else "없음" for item in items]
                
                # 각 아이템의 빈도 세기
                item_counts = {}
                for item in items:
                    if item in item_counts:
                        item_counts[item] += 1
                    else:
                        item_counts[item] = 1
                
                # 가장 많이 등장한 아이템 선택 (빈도수가 가장 큰 항목)
                most_common_item = max(item_counts, key=item_counts.get)
                final_slots.append(most_common_item)

            # 최종 디텍된 물건 리스트 출력
            print(f"최종 냉장고 슬롯 상태: {final_slots}")
            for i in range(len(final_slots)):
                if final_slots[i] == 'lettuce':
                    final_slots[i] = '상추'
                elif final_slots[i] == 'pork':
                    final_slots[i] = '삼겹살'
                elif final_slots[i] == 'kimchi':
                    final_slots[i] = '김치'


            self.speak(f"냉장고 위치로 이동했습니다. {', '.join(final_slots)}가 있네요.")  # 디텍션된 클래스명 넣기
            self.speak("어떤 재료를 사용하시겠어요?") 

            while True:
                next_input = self.listen()
                ##########################
                next_input = '삼겹살'
                ##########################
                if next_input:  # 입력이 인식되었을 경우
                    return self.respond(next_input)
                else:
                    self.speak("죄송해요, 잘 못 들었어요. 어떤 재료를 사용하시겠어요?")

        elif user_input and "삼겹살" in user_input:
            self.speak("구이를 원하시나요, 수육을 원하시나요, 아니면 고추장삼겹살볶음을 원하시나요?")
            
            while True:
                user_input_2 = self.listen()
                ##########################
                user_input_2 = '구이'
                ##########################
                if user_input_2:
                    user_input_2 = user_input_2.strip().lower()
                    print(f"🧪 두 번째 인식된 문장: '{user_input_2}'")
                    
                    if "구이" in user_input_2:
                        self.speak("삼겹살 구이를 준비하겠습니다.")
                        move_frypan()
                        move_pork()

                        self.pork_detection('blg_Pork-before')
                        self.speak("고기를 뒤집을게요.")

                        # 뒤집기
                        flip_pork_1()
                        flip_pork_2()

                        self.pork_detection('slice_Pork-60')
                        # 반쯤 익었을 때, 조미료
                        move_spice_detection()
                        self.speak("조미료가 필요하신가요?")
                        
                        # 1. 아르코 마커로 사용 가능한 조미료를 먼저 감지합니다.
                        available_seasonings = self.detect_available_seasonings(duration=10)

                        # 2. 감지된 조미료가 있을 경우에만 사용자에게 질문합니다.
                        if not available_seasonings:
                            self.speak("사용 가능한 조미료가 감지되지 않았습니다. 바로 다음 조리를 시작할게요.")
                        else:
                            # 사용자가 알아들을 수 있도록 ", "로 연결된 문자열을 만듭니다.
                            seasonings_text = ", ".join(available_seasonings)
                            self.speak(f"현재 {seasonings_text}을(를) 사용할 수 있습니다. 어떤 것을 사용하시겠어요?")

                            # 3. 사용자의 답변을 듣고 '사용 가능한' 조미료인지 확인합니다.

                            while True:
                                user_choice = self.listen()
                                ##########################
                                user_choice = '소금'
                                ##########################
                                if user_choice:
                                    user_choice = user_choice.strip().lower()
                                    
                                    chosen_seasoning = None
                                    # 사용자가 말한 조미료가 감지된 목록에 있는지 확인합니다.
                                    for seasoning in available_seasonings:
                                        if seasoning in user_choice:
                                            chosen_seasoning = seasoning
                                            break
                                    
                                    if chosen_seasoning:
                                        # 감지된 조미료에 대해서만 로봇 동작을 수행합니다.
                                        if chosen_seasoning == "소금":
                                            self.speak(f"{chosen_seasoning}을(를) 가져다 드리겠습니다.")
                                            move_pepper_1()
                                            self.speak("확인을 위해 소리를 들려드릴게요!")
                                            move_pepper_2()

                                        elif chosen_seasoning == "후추":
                                            self.speak(f"{chosen_seasoning}을(를) 가져다 드리겠습니다.")

                                        elif chosen_seasoning == "고춧가루":
                                            self.speak(f"{chosen_seasoning}을(를) 가져다 드리겠습니다.")
                                            # move_red_pepper() # 고춧가루 관련 동작 함수 (필요시 추가)

                                        elif chosen_seasoning == "설탕":
                                            self.speak(f"{chosen_seasoning}을(를) 가져다 드리겠습니다.")
                                            # move_sugar() # 설탕 관련 동작 함수 (필요시 추가)
                                        
                                        break # 조미료 선택이 완료되었으므로 반복문을 탈출합니다.
                                    else:
                                        self.speak(f"죄송하지만 {user_choice}은(는) 사용할 수 없는 조미료입니다. 다시 말씀해주세요.")
                                else:
                                    self.speak("죄송합니다. 다시 말씀해 주세요.")        

                        # 고기 익은거 확인
                        self.pork_detection('blg_Pork-after')

                        complete_pork()
                        self.speak("조리가 완료되었습니다. 뜨거우니 조심하세요!")
                        self.speak("상추와 김치를 가져다 드릴게요.")
                        move_kimchi()
                        move_lettuce()
                        self.speak("맛있게 드세요!")
                        move_home()
                        return
                    
                    elif "수육" in user_input_2:
                        self.speak("삼겹살 수육을 준비하겠습니다.")                                    
                        move_boiled_pork()
                        move_pork()
                        self.speak("온도계를 가져오겠습니다.")
                        move_thermometer()
                        time.sleep(1)
                        cap = cv2.VideoCapture(4)  # 카메라 번호 확인 필요 (보통 0, 1, 2, ...)

                        if not cap.isOpened():
                            print("카메라를 열 수 없습니다.")
                            return

                        print("실시간 예측 시작 (종료: Q 키 누르기)")
                        flag = 0

                        while True:
                            ret, frame = cap.read()
                            if not ret:
                                print("프레임을 읽을 수 없습니다.")
                                break

                            # LCD 영역 검출 및 7세그먼트 인식
                            predicted_digits, debug_img = process_lcd_and_detect_digits(frame)

                            if predicted_digits and '?' not in predicted_digits:
                                print("인식된 숫자:", predicted_digits)
                                if int(predicted_digits) == 888:
                                    continue
                                elif flag == 0 and int(predicted_digits) > 200 and int(predicted_digits) < 270:
                                    print('물이 끓고 있습니다.')
                                    if flag == 0:
                                        self.speak("물이 뜨겁습니다. 조심하세요!")
                                        flag = 1
                                elif flag == 1 and int(predicted_digits) > 270 and int(predicted_digits) < 320:
                                    print('물이 매우 뜨겁습니다.')
                                    if flag == 1:
                                        self.speak("물이 매우 뜨겁습니다. 조심하세요!")
                                        flag = 2
                                elif flag == 2 and int(predicted_digits) > 320:
                                    print('조리가 완료되었습니다.')
                                    if flag == 2:
                                        self.speak("조리가 완료되었습니다. 뜨거우니 조심하세요!")
                                        flag = 3
                                    break

                            # 디버그 이미지 표시
                            cv2.imshow("Debug Image", debug_img)

                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                        cap.release()
                        cv2.destroyAllWindows()

                        back_thermometer()
                        complete_boiled_pork()
                        self.speak("조리가 완료되었습니다. 뜨거우니 조심하세요!")
                        self.speak("상추와 김치를 가져다 드릴게요.")
                        move_kimchi()
                        move_lettuce()
                        self.speak("맛잇게 드세요!")
                        move_home()
                        return 
                    
                    elif "볶음" in user_input_2 or "고추장" in user_input_2:
                        self.speak("고추장삼겹살볶음을 준비하겠습니다.")
                        return "고추장삼겹살볶음을 준비하겠습니다."
                    
                    else:
                        self.speak(f"{user_input_2}이라고 말씀하셨군요. 아직 학습되지 않은 방식입니다. 다시 말씀해 주세요.")
                else:
                    self.speak("죄송합니다. 다시 말씀해 주세요.")

    def detect_available_seasonings(self, duration=10):
        """지정된 시간 동안 아르코 마커를 감지해 사용 가능한 조미료 목록을 반환합니다."""
        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            self.speak("카메라에 문제가 있어 조미료를 확인할 수 없습니다.")
            return set()

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()

        id_to_spice = {
            0: "소금",
            1: "설탕",
            2: "후추",
            3: "고춧가루"
        }

        detected_spices = set()
        start_time = time.time()
        
        self.speak("사용 가능한 조미료를 확인하겠습니다.")

        while time.time() - start_time < duration:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
                for marker_id in ids.flatten():
                    if marker_id in id_to_spice:
                        detected_spices.add(id_to_spice[marker_id])
            
            cv2.imshow("Detecting Seasonings...", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        
        return detected_spices
    
    def handle_side_dish_selection(self):
        user_input_3 = self.listen()
        if user_input_3:
            user_input_3 = user_input_3.strip().lower()
            print(f"🥬 반찬 선택 인식: '{user_input_3}'")
            if "상추" in user_input_3:
                self.simulate_event("lettuce")
            elif "김치" in user_input_3:
                self.simulate_event("kimchi")
            else:
                self.speak(f"{user_input_3}이라고 말씀하셨군요. 아직 학습되지 않은 반찬입니다.")
        else:
            self.speak("죄송합니다. 다시 말씀해 주세요.")

    def speak(self, text):
        tts = gTTS(text=text, lang="ko")
        filename = "response.mp3"
        tts.save(filename)
        sound = AudioSegment.from_file(filename)
        faster_sound = sound._spawn(sound.raw_data, overrides={
            "frame_rate": int(sound.frame_rate * 1.0)
        }).set_frame_rate(sound.frame_rate)
        play(faster_sound)
        os.remove(filename)

def main():
    rclpy.init()
    aux_node = rclpy.create_node('voice_ass_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = aux_node

    from DSR_ROBOT2 import (
        movej, movel, move_periodic, DR_TOOL, set_digital_output, wait
    )
    from DR_common2 import posx
    from robot_actions1_copy import move_home, back_thermometer, flip_pork_1, flip_pork_2, complete_pork, pork_config_pos, move_to_fridge, move_frypan, move_pork, move_boiled_pork, complete_boiled_pork, move_thermometer, move_pepper_1, move_pepper_2, move_kimchi, move_lettuce
    from temp_detector import process_lcd_and_detect_digits

    assistant = VoiceAssistant()
    print("🟢 음성 비서가 대기 중입니다. '감지'라고 말씀하시면 시작됩니다. 종료하려면 Ctrl+C를 누르세요.")

    try:
        while True:
            user_input = assistant.listen()
            #####################333
            user_input = '감지'
            ##########################
            if user_input:
                user_input = user_input.strip().lower()

                if "종료" in user_input:
                    assistant.speak("음성 비서를 종료합니다. 안녕히 계세요!")
                    break

                if "감지" in user_input:
                    assistant.speak("네, 부르셨어요?")
                    next_command = assistant.listen()
                    #####################333
                    next_command = '냉장고'
                    ##########################
                    if next_command:
                        assistant.respond(next_command)
    except KeyboardInterrupt:
        print("\n👋 음성 비서를 종료합니다. 안녕히 계세요!")

if __name__ == "__main__":
    main()