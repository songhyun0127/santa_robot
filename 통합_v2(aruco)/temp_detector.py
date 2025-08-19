import cv2
import numpy as np

# 7세그먼트 상태 매핑 (7개 세그먼트 -> 숫자)
SEGMENT_MAP = {
    (1,1,1,1,1,1,0): '0',
    (0,1,1,0,0,0,0): '1',
    (1,1,0,1,1,0,1): '2',
    (1,1,1,1,0,0,1): '3',
    (0,1,1,0,0,1,1): '4',
    (1,0,1,1,0,1,1): '5',
    (1,0,1,1,1,1,1): '6',
    (1,1,1,0,0,0,0): '7',
    (1,1,1,1,1,1,1): '8',
    (1,1,1,1,0,1,1): '9',
}

points_list_fixed = [
    [(150, 268), (154, 163), (161, 54), (243, 54), (250, 165), (242, 270)],
    [(306, 263), (305, 162), (314, 50), (393, 52), (395, 157), (391, 264)],
    [(468, 261), (466, 160), (468, 46), (556, 50), (555, 155), (556, 261)],
]

def get_segment_rects(points, padding=10):
    # 세그먼트 좌표 생성 함수
    left_bottom = points[0]
    left_middle = points[1]
    left_top = points[2]
    right_top = points[3]
    right_middle = points[4]
    right_bottom = points[5]

    segs = {
        'a': (left_top[0]-padding, left_top[1]-padding, right_top[0]+padding, right_top[1]+padding),
        'b': (right_top[0]-padding, right_top[1]-padding, right_middle[0]+padding, right_middle[1]+padding),
        'c': (right_middle[0]-padding, right_middle[1]-padding, right_bottom[0]+padding, right_bottom[1]+padding),
        'd': (left_bottom[0]-padding, left_bottom[1]-padding, right_bottom[0]+padding, right_bottom[1]+padding),
        'e': (left_bottom[0]-padding, left_bottom[1]-padding, left_middle[0]+padding, left_middle[1]+padding),
        'f': (left_middle[0]-padding, left_middle[1]-padding, left_top[0]+padding, left_top[1]+padding),
        'g': (left_middle[0]-padding, left_middle[1]-padding, right_middle[0]+padding, right_middle[1]+padding),
    }
    return segs

def crop_and_resize(img, crop_rect, scale=4):
    x1, y1, x2, y2 = crop_rect
    roi = img[y1:y2, x1:x2]
    resized = cv2.resize(roi, (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    return resized

def get_segments_status(binary_img, seg_rects):
    # 7세그먼트의 ON/OFF 상태 판별 함수
    status = []
    for seg_name, (x1, y1, x2, y2) in seg_rects.items():
        x1, x2 = sorted([int(x1), int(x2)])
        y1, y2 = sorted([int(y1), int(y2)])

        # 최소 크기 확보
        if x2 - x1 < 1:
            x2 = x1 + 1
        if y2 - y1 < 1:
            y2 = y1 + 1

        # 이미지 범위 클램프
        x1, y1 = max(x1, 0), max(y1, 0)
        x2, y2 = min(x2, binary_img.shape[1] - 1), min(y2, binary_img.shape[0] - 1)

        seg_region = binary_img[y1:y2, x1:x2]
        mean_val = cv2.mean(seg_region)[0]

        # 임계값 100 이상이면 ON으로 판단
        status.append(1 if mean_val > 100 else 0)
    return tuple(status)

def find_and_warp_lcd(image):
    # LCD 영역 검출 및 원근 변환 함수
    warped_width = 700
    warped_height = 300
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    lcd_contour = None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * perimeter, True)
        
        if len(approx) == 4 and area > 500:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            if 1.5 < aspect_ratio < 3.0:
                if area > max_area:
                    max_area = area
                    lcd_contour = approx

    if lcd_contour is not None:
        pts = lcd_contour.reshape(4, 2)
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)] # Top-right
        rect[3] = pts[np.argmax(diff)] # Bottom-left
        
        dst_pts = np.array([
            [0, 0],
            [warped_width - 1, 0],
            [warped_width - 1, warped_height - 1],
            [0, warped_height - 1]], dtype="float32")

        M = cv2.getPerspectiveTransform(rect, dst_pts)
        warped = cv2.warpPerspective(image, M, (warped_width, warped_height))
        
        return warped, rect.astype(int)
    else:
        return None, None

def process_lcd_and_detect_digits(frame):
    # 0. 이미지 회전 추가: 반시계 방향으로 90도 회전
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # 1. LCD 영역 검출 및 보정
    warped_lcd, lcd_corners = find_and_warp_lcd(rotated_frame)

    if warped_lcd is not None:
        debug_img = warped_lcd.copy()
        sharpen_kernel = np.array([[0, -1, 0],
                                   [-1, 5, -1],
                                   [0, -1, 0]])
        sharpened = cv2.filter2D(warped_lcd, -1, sharpen_kernel)
        gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        predicted_digits = ""
        for points in points_list_fixed:
            seg_rects = get_segment_rects(points)
            status = get_segments_status(binary, seg_rects)
            digit = SEGMENT_MAP.get(status, '?')
            predicted_digits += digit

            # 디버그용 사각형 그리기
            for seg_name, (x1, y1, x2, y2) in seg_rects.items():
                x1, x2 = sorted([int(x1), int(x2)])
                y1, y2 = sorted([int(y1), int(y2)])
                color = (0, 255, 0) if status[list(seg_rects.keys()).index(seg_name)] else (0, 0, 255)
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 1)

        return predicted_digits, debug_img
    else:
        return None, frame