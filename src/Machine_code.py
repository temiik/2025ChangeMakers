#!/usr/bin/env python3
import socket
from ev3dev2.motor import MediumMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import UltrasonicSensor, TouchSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from time import sleep, time

# Моторы и сенсоры
drive_motor = MediumMotor(OUTPUT_B)
steering_motor = MediumMotor(OUTPUT_C)
ultrasonic_left = UltrasonicSensor(INPUT_1)
ultrasonic_right = UltrasonicSensor(INPUT_2)
touch_sensor = TouchSensor(INPUT_3)
color_sensor = ColorSensor(INPUT_4)

# Константы
SPEED = 400
cnt = 0
latest_color = None
latest_x = None
last_seen_color = None
last_seen_time = 0
msg = ""
TURN_COOLDOWN = 4
GYRO_TURN_ANGLE = 45
cnt = 0
def center_steering():
    steering_motor.run_to_abs_pos(position_sp=0, speed_sp=800)

def move_forward():
    drive_motor.run_forever(speed_sp=-SPEED)

def stop():
    drive_motor.stop(stop_action='brake')
    steering_motor.stop()

def adjust_to_center():
    left = ultrasonic_left.distance_centimeters
    right = ultrasonic_right.distance_centimeters
    tolerance = 3
    if abs(left - right) <= tolerance:
        center_steering()
    elif left < right:
        steering_motor.run_to_abs_pos(position_sp=-50, speed_sp=1000)
    else:
        steering_motor.run_to_abs_pos(position_sp=40, speed_sp=1000)

    print("Left distance:", ultrasonic_left.distance_centimeters)
    print("Right distance:", ultrasonic_right.distance_centimeters)

def turnl():
    steering_motor.run_to_abs_pos(position_sp=-50, speed_sp=1000)
    drive_motor.run_forever(speed_sp=1000)
    sleep(0.8)
    
    
def turn_around_obstacle(target_color):
    global msg

    print("Starting turn around obstacle (PD control)")
    
    # Проверяем расстояния до поворота
    left_before = ultrasonic_left.distance_centimeters
    right_before = ultrasonic_right.distance_centimeters
    print("Distances before turn - Left:", left_before, "Right:", right_before)
    
    # Определяем направление поворота на основе расстояний
    turn_direction = 1 if left_before > right_before else -1
    print("Turning direction:", "right" if turn_direction > 0 else "left")
    
    center_steering()
    drive_motor.run_forever(speed_sp=700)
    sleep(0.4)

    # PD controller constants (tune as needed)
    target_x = 640  # Center of image (adjust if needed)
    cKp = 0.15
    cKd = 0.08
    straightConst = 0
    prevError = 0

    # Start moving forward slowly while turning
    drive_motor.run_forever(speed_sp=-150)

    timeout = time() + 5  # Safety timeout
    while time() < timeout:
        if msg == "none" or not msg.startswith(target_color):
            print("Obstacle no longer seen - finishing turn")
            break
        try:
            _, x_str, y_str = msg.split(',')
            current_x = int(x_str)
            cy = int(y_str)
            print("DEBUG: cy =", cy)  # Для отладки
            
            # Проверяем, не слишком ли близко столбец
            if cy > 350:  # Порог уменьшен!
                print("Column] too close! Backing up.")
                stop()
                # Отходим назад
                drive_motor.run_forever(speed_sp=800)
                sleep(0.8)
                stop()
                sleep(0.3)
                left_after = ultrasonic_left.distance_centimeters
                right_after = ultrasonic_right.distance_centimeters
                print("Distances after backup - Left:", left_after, "Right:", right_after)
                if left_after < 20 or right_after < 20:
                    print("Still too close, backing up more")
                    drive_motor.run_forever(speed_sp=800)
                    sleep(0.5)
                    stop()
                break
            
            error = target_x - current_x
            angle = int(straightConst + error * cKp + (error - prevError) * cKd)
            prevError = error
            angle = angle * turn_direction
            angle = max(-70, min(70, angle))
            steering_motor.run_to_abs_pos(position_sp=angle, speed_sp=1000)
        except Exception as e:
            print("[WARN] PD control error:", e)
        sleep(0.05)

    stop()
    center_steering()
    sleep(0.7)

    # После прохождения столба
    steering_motor.run_to_abs_pos(position_sp=0, speed_sp=1000)
    drive_motor.run_forever(speed_sp=-1000)
    sleep(0.7)
    stop()
    center_steering()

    # Финальная проверка и корректировка
    left_dist = ultrasonic_left.distance_centimeters
    right_dist = ultrasonic_right.distance_centimeters
    if abs(left_dist - right_dist) > 5:
        if left_dist < right_dist:
            steering_motor.run_to_abs_pos(position_sp=20, speed_sp=800)
        else:
            steering_motor.run_to_abs_pos(position_sp=-20, speed_sp=800)
    drive_motor.run_timed(time_sp=600, speed_sp=-600)
    sleep(0.6)
    stop()
    print("Turn completed (PD control)")

def get_smoothed_distance(sensor, window=5):
    readings = [sensor.distance_centimeters for _ in range(window)]
    return sum(readings) / window

def main():
    global latest_color, latest_x, last_seen_color, last_seen_time, cnt, msg
    lasttime = -100

    HOST = '0.0.0.0'
    PORT = 12345
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print("Waiting for connection from Raspberry Pi...")
    client_socket, _ = server_socket.accept()
    print("Connected.")

    print("Waiting for touch sensor press to start...")
    while not touch_sensor.is_pressed:
        sleep(0.05)
    print("Touch sensor pressed! Sending start signal to Pi.")
    client_socket.sendall(b"start\n")

    center_steering()
    drive_motor.run_forever(speed_sp=SPEED)
    sleep(0.35)

    sock_file = client_socket.makefile("r")
    drive_motor.run_forever(speed_sp=0)
    sleep(1)

    drive_motor.run_forever(speed_sp=50)

    last1time = -100
    last2time = -100
    
    try:
        for line in sock_file:
            msg = line.strip()
            print("Received:", msg)

            now = time()
            can_turn = now - last_seen_time > 3

            if msg == "none":
                adjust_to_center()
                move_forward()
            
            elif msg == "backup" and now - last2time > 2.2:
                print("[INFO] Wall too close - backing up")
                center_steering()
                last2time = now
                drive_motor.run_forever(speed_sp=SPEED)  # Двигаемся назад
                sleep(1.0)  # Едем назад 1 секунду
                stop()
                sleep(0.5)  # Небольшая пауза
                move_forward()  # Возвращаемся к нормальному движению
            else:
                parts = msg.split(',')
                if len(parts) == 3:
                    color, x_str, y_str = parts
                    try:
                        x = int(x_str)
                        y = int(y_str)
                        latest_color = color
                        latest_x = x

                        if can_turn and color != last_seen_color:
                            turn_around_obstacle(color)
                            last_seen_color = color
                            last_seen_time = now
                    except ValueError:
                        print("[WARN] Invalid message:", msg)

            if(cnt == 12):
                stop()

            sleep(0.05)

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        stop()
        client_socket.close()
        server_socket.close()

if __name__ == '__main__':
    main()