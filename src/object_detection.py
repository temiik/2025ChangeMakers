import cv2
import numpy as np
import socket
import time

# --- Обновлённые LAB пороги ---
COLOR_THRESHOLDS = {
    "green": [
        ([50, 20, 20], [255, 110, 140])
    ],
    "red": [
        ( [0, 157, 0] , [255, 255, 255])
    ]
}

MIN_AREA = 2000
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CENTER_TOLERANCE = 200
MARGIN = 100

# Область интереса (ROI): центральная часть кадра
ROI = (0, 160, FRAME_WIDTH, 560)

# Верхняя область для раннего обнаружения
LOOKAHEAD_ROI = (0, 60, FRAME_WIDTH, 200)

# ROIs
FRONT_ROI = (400, 600, 0, FRAME_WIDTH)  # Верхняя часть для проверки стены
LOWER_ROI = (600, 720, 0, FRAME_WIDTH)  # Нижняя часть для проверки близости к стене

# Обновлённые маски для других объектов
BLACK_LOWER = np.array([0, 0, 0], dtype=np.uint8)
BLACK_UPPER = np.array([50, 128, 128], dtype=np.uint8)

already_detected = {
    "red": False,
    "green": False
}

cooldown = {
    "red": 0,
    "green": 0
}

COOLDOWN_TIME = 4  # секунд
kernel = np.ones((5, 5), np.uint8)
BACKUP_AREA = 3500  # Порог на "врезался"

def detect_largest_object_lab(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lab_roi = lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    largest = None

    for color in COLOR_THRESHOLDS:
        for lower, upper in COLOR_THRESHOLDS[color]:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(lab_roi, lower, upper)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)

            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                center_x = ROI[0] + x + w // 2
                center_y = ROI[1] + y + h // 2

                if largest is None or area > largest["area"]:
                    largest = {
                        "color": color,
                        "center": (center_x, center_y),
                        "area": area
                    }

    return largest

def detect_lookahead_column(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lookahead_roi = lab[LOOKAHEAD_ROI[1]:LOOKAHEAD_ROI[2], LOOKAHEAD_ROI[0]:LOOKAHEAD_ROI[2]]
    for color in COLOR_THRESHOLDS:
        for lower, upper in COLOR_THRESHOLDS[color]:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(lookahead_roi, lower, upper)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 600:
                    return color
    return None

def wall_too_close(lab):
    # Проверяем стену в верхней части
    front_roi = lab[FRONT_ROI[0]:FRONT_ROI[1], FRONT_ROI[2]:FRONT_ROI[3]]
    front_mask = cv2.inRange(front_roi, BLACK_LOWER, BLACK_UPPER)
    wall_area = np.sum(front_mask > 0)
    
    # Проверяем стену в нижней части
    lower_roi = lab[LOWER_ROI[0]:LOWER_ROI[1], LOWER_ROI[2]:LOWER_ROI[3]]
    lower_mask = cv2.inRange(lower_roi, BLACK_LOWER, BLACK_UPPER)
    lower_area = np.sum(lower_mask > 0)
    
    print("Upper wall area:", wall_area)
    print("Lower wall area:", lower_area)
    
    # Если в нижней части много черного - значит мы слишком близко к стене
    return lower_area > 20000  # Порог для нижней части стены

def main():
    global already_detected, cooldown

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    EV3_IP = '10.42.0.3'  # Укажи IP твоего EV3
    PORT = 12345

    try:
        print("[INFO] Connecting to EV3 at {}:{}...".format(EV3_IP, PORT))
        client.connect((EV3_IP, PORT))
        print("[INFO] Connected.")
        print("[INFO] Waiting for start signal from EV3...")
        sock_file = client.makefile("r")
        while True:
            line = sock_file.readline()
            if not line:
                continue
            if line.strip() == "start":
                print("[INFO] Received start signal from EV3. Starting detection.")
                break

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Failed to grab frame.")
                continue

            lookahead_color = detect_lookahead_column(frame)
            if lookahead_color:
                msg = "prepare_{}".format(lookahead_color)
            else:
                obj = detect_largest_object_lab(frame)
                now = time.time()
                msg = "none"

                if obj and obj["area"] >= MIN_AREA:
                    color = obj["color"]
                    cx, cy = obj["center"]

                    if (color == "red" and cx < MARGIN) or (color == "green" and cx > FRAME_WIDTH - MARGIN):
                        already_detected[color] = False
                        msg = "none"
                    elif not already_detected[color] or now > cooldown[color]:
                        msg = "{},{},{}".format(color, cx, cy)
                        already_detected[color] = True
                        cooldown[color] = now + COOLDOWN_TIME
                else:
                    for color in already_detected:
                        already_detected[color] = False

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            if wall_too_close(lab):
                print("[WARN] Wall too close in lower part! Sending backup command")
                msg = "backup"
            else:
                msg = msg if "prepare" in msg or "," in msg else "none"

            try:
                client.sendall((msg + "\n").encode())
                print("[SEND]", msg)
            except Exception as e:
                print("[ERROR] Lost connection: {}".format(str(e)))
                break

            time.sleep(0.1)

    except Exception as e:
        print("[ERROR]", str(e))

    finally:
        cap.release()
        client.close()
        print("[INFO] Finished")

if __name__ == "__main__":
    main()