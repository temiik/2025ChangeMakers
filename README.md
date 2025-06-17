This repository documents our journey in building an autonomous vehicle for the WRO Future Engineers 2024-25 challenge, using a hybrid approach with LEGO EV3 and Raspberry Pi for robust, real-time vision and control.

# 🛠️ Team Changemakers — WRO Future Engineers 2024-25 Documentation

[![Youtube](https://img.shields.io/badge/Youtube-%23FF0000.svg?style=for-the-badge&logo=Youtube&logoColor=white)](https://www.youtube.com/@2025ChangeMakers)
[![Instagram](https://img.shields.io/badge/Instagram-%23E4405F.svg?style=for-the-badge&logo=Instagram&logoColor=white)](https://instagram.com/nerdvana_romania/)
[![Facebook](https://img.shields.io/badge/Facebook-%231877F2.svg?style=for-the-badge&logo=Facebook&logoColor=white)](https://www.facebook.com/nerdvanaro/)

**Team Members:**  
- Yesken Kairat, 16 — kairatyesken@gmail.com  
- Tolendi Temirlan, 16  
- Yusuf Mukhambetkaliev, 16  

**Location:** Kazakhstan, Kokshetau
**Contact:** kairatyesken@gmail.com

**Mission:**  
> We are Team Changemakers — passionate about robotics, innovation, and real-world engineering. Our goal is to build robust, intelligent autonomous vehicles that can adapt to any challenge.

---

## Table of Contents
- [Introduction](#introduction)
- [System Architecture](#system-architecture)
- [Hardware Overview](#hardware-overview)
  - [Chassis & Steering](#chassis--steering)
  - [Sensors](#sensors)
  - [Computing Units](#computing-units)
- [Software Overview](#software-overview)
  - [EV3dev & Raspberry Pi Setup](#ev3dev--raspberry-pi-setup)
  - [Communication (SSH)](#communication-ssh)
  - [Vision & Detection (OpenCV)](#vision--detection-opencv)
  - [Control Logic](#control-logic)
- [Algorithmic Details](#algorithmic-details)
  - [ROI (Region of Interest) Explanation](#roi-region-of-interest-explanation)
  - [Ultrasonic Centering](#ultrasonic-centering)
  - [Wall Avoidance with Camera](#wall-avoidance-with-camera)
- [Why We Switched: Arduino, EV3, and Pi](#why-we-switched-arduino-ev3-and-pi)
- [Potential Improvements](#potential-improvements)
- [Team & Contact](#team--contact)

---

## Introduction

Our robot is designed to autonomously navigate a WRO FE field, detect colored pillars and lines, avoid walls, and park precisely. We leverage the strengths of both the LEGO EV3 (for reliable motor and sensor control) and the Raspberry Pi (for advanced computer vision), combining them via a simple and robust SSH-based communication protocol.

---

## System Architecture

- **Front-wheel Ackerman steering** for realistic car-like movement.
- **Rear-wheel drive** for propulsion.
- **Raspberry Pi** handles all camera-based object and wall detection using OpenCV.
- **EV3 Brick** (running ev3dev) manages motors, ultrasonic sensors, and executes movement commands.
- **SSH socket communication** links the Pi and EV3, ensuring low-latency, reliable command transfer.

---

## Hardware Overview

### Chassis & Steering

- **Chassis:** Custom made by lego, adapted for WRO FE size constraints.
- **Steering:** Front-wheel Ackerman steering, providing realistic turning dynamics and improved maneuverability compared to differential drive.
- **Drive:** Rear-wheel drive, controlled via EV3 motors.

### ⚡ Power Management

- **EV3 Power:** Powered by standard LEGO EV3 battery (2200mAh Li-ion)
  - Powers both EV3 motors (drive and steering)
  - Powers ultrasonic sensors and color sensor
  - Provides stable voltage for reliable sensor readings
- **Raspberry Pi Power:** USB power bank (10000mAh)
  - Dedicated power source for stable computer vision
  - Powers the CSI camera module
  - Ensures clean signal processing

### 🔌 Connection Scheme

#### EV3 Configuration

| Port Type | Port Number | Component | Purpose |
|-----------|-------------|-----------|----------|
| Input | 1 | Ultrasonic Sensor (Left) | Wall distance detection |
| Input | 2 | Ultrasonic Sensor (Right) | Wall distance detection |
| Input | 3 | Color Sensor | Turn detection |
| Input | 4 | Touch Sensor | Program start button |
| Output | A | Drive Motor | Main propulsion |
| Output | B | Steering Motor | Direction control |

#### Raspberry Pi Configuration

| Port | Connected To | Purpose |
|------|-------------|----------|
| CSI Port | Camera Module | Vision input |
| USB-A | EV3 Brick | SSH communication |
| USB-C | Power Bank | Power input (5V) |
| GPIO | Fan | Cooling (optional) |

#### Power Distribution

| Component | Power Source | Voltage | Current Draw |
|-----------|--------------|---------|--------------|
| EV3 Brick | EV3 Battery | 7.2V | 2200mAh |
| Motors | EV3 Battery | 7.2V | ~500mA each |
| Sensors | EV3 Battery | 5V | ~50mA each |
| Raspberry Pi | Power Bank | 5V | ~2.5A max |
| Camera | Raspberry Pi | 3.3V | ~250mA |

#### Communication Flow
```
Raspberry Pi (Vision Processing)
         ↓ 
    [USB Cable]
         ↓ 
   EV3 (Control)
         ↓
[Motors & Sensors]
```

### Sensors

- **Ultrasonic Sensors (EV3):** Mounted on both sides for precise centering between walls.
- **Color Sensor (EV3):** For counting turns
- **Button (EV3):** Starting the program
- **Camera (Raspberry Pi):** Wide-angle CSI camera for robust color and object detection.

### Computing Units

- **Raspberry Pi 4:** Runs Python 3, OpenCV, and all vision algorithms.
- **LEGO EV3 Brick:** Runs ev3dev (Debian-based OS), controls motors and sensors, receives commands from Pi with python

---

## Software Overview

### EV3dev & Raspberry Pi Setup

- **EV3dev** installed on the EV3 brick for full Linux compatibility and Python support. It gives opportunity to control all motors and sensors with **python 3.5**
- **Raspberry Pi** runs Raspbian OS with OpenCV and all necessary Python libraries for columns and wall detection
- **SSH keys** set up for passwordless communication between Pi and EV3.

### Communication (SSH)

- **Socket-based protocol:** The Pi opens a TCP socket to the EV3, sending simple string commands (e.g., `red,640,300`, `backup`, `prepare_green`).

**Example: Socket connection on Raspberry Pi**
```python
import socket
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
EV3_IP = '10.42.0.3'  # EV3 IP address
PORT = 12345
client.connect((EV3_IP, PORT))
```

**Example: Socket server on EV3**
```python
import socket
HOST = '0.0.0.0'
PORT = 12345
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for connection from Raspberry Pi...")
client_socket, _ = server_socket.accept()
print("Connected.")
```

- **EV3 listens** for commands, parses them, and executes corresponding motor/sensor actions.

### Vision & Detection (OpenCV)

- **LAB color space** is used for robust color detection, minimizing the effect of lighting changes.
- **Gaussian blur, erosion, and dilation** are applied to clean up the image and improve contour detection.
- **Contours** are extracted for pillars, lines, and walls.
- **ROI (Region of Interest):** Only relevant parts of the image are processed for each task, improving speed and reliability.

**Example: ROI extraction and mask creation (Raspberry Pi)**
```python
    import cv2
import numpy as np
# Assume frame is a BGR image from the camera
lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
ROI = (0, 160, 1280, 560)  # x1, y1, x2, y2
lab_roi = lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
lower = np.array([50, 20, 20], dtype=np.uint8)
upper = np.array([255, 110, 140], dtype=np.uint8)
mask = cv2.inRange(lab_roi, lower, upper)
```

**Example: Sending a command from Pi to EV3**
```python
msg = "red,640,300"  # Example: detected red pillar at (640, 300)
client.sendall((msg + "\n").encode())
```

**Example: Receiving and handling a command on EV3**
```python
for line in sock_file:
    msg = line.strip()
    if msg == "backup":
        # Back up if wall is too close
        drive_motor.run_forever(speed_sp=400)
        sleep(1.0)
        stop()
    elif ',' in msg:
        color, x, y = msg.split(',')
        # Handle pillar avoidance logic
```

### 🔧 Physical Setup and Calibration

#### Camera Mounting
- Camera mounted at optimal height (15cm) for pillar detection
- Tilted slightly downward (15°) to better see nearby obstacles
- Wide-angle lens provides ~160° field of view

#### Sensor Positioning
- Ultrasonic sensors mounted parallel to ground
- Positioned at equal distances from center
- Height optimized to avoid floor reflections

#### Motor Configuration
- Drive motor geared for balance of speed and torque
- Steering motor calibrated for maximum 50° turn angle
- PID parameters tuned for smooth control

### Control Logic

- **Pillar avoidance:** The Pi detects red/green pillars and sends their coordinates to the EV3, which executes a PD-controlled avoidance maneuver.
- **Wall centering:** The EV3 uses ultrasonic sensors to keep the robot centered between walls.
- **Wall avoidance (camera):** The Pi detects black regions (walls) in the camera image and commands the EV3 to back up if too close.
- **Line detection:** (If used) The Pi detects colored lines for turn/parking logic.

---

## Algorithmic Details

### ROI (Region of Interest) Explanation

- **Why ROI?** Processing only the relevant part of the image (e.g., lower half for wall detection, center for pillars) reduces noise and increases speed.
- **How?** Each detection task (pillar, wall, line) uses its own ROI, defined as a rectangle in image coordinates.

### Ultrasonic Centering

- **How it works:** The EV3 reads both left and right ultrasonic sensors. If the difference is within a small tolerance, the robot is centered. Otherwise, the steering is adjusted to correct the position.

**Example: Ultrasonic centering on EV3**
```python
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
    print("Left distance:", left)
    print("Right distance:", right)
```

### Wall Avoidance with Camera

- **How it works:** The Pi analyzes the lower part of the image for large black regions (walls). If detected, it sends a `backup` command to the EV3.

**Example: Wall detection and backup command (Raspberry Pi)**
```python
BLACK_LOWER = np.array([0, 0, 0], dtype=np.uint8)
BLACK_UPPER = np.array([50, 128, 128], dtype=np.uint8)
lower_roi = lab[600:720, 0:1280]
    lower_mask = cv2.inRange(lower_roi, BLACK_LOWER, BLACK_UPPER)
    lower_area = np.sum(lower_mask > 0)
if lower_area > 20000:
    client.sendall(b"backup\n")
```

---

## Why We Switched: Arduino, EV3, and Pi

- **Previous Solution:** We initially used Arduino for motor and sensor control, but found it limiting for real-time vision and complex logic.
- **Why EV3?** The EV3 offers robust, reliable motor and sensor control, and is easy to program with Python via ev3dev.
- **Why Raspberry Pi?** The Pi is powerful enough for real-time image processing with OpenCV, and can easily communicate with the EV3.
- **Why not just one?** Combining both allows us to leverage the strengths of each platform: Pi for vision, EV3 for control.
- **Communication:** SSH and sockets provide a simple, reliable way to link the two systems, with minimal latency and easy debugging.

---

## Potential Improvements

- **Integrate IMU/Gyro** for more precise turns and drift correction.
- **Optimize vision pipeline** for even faster frame rates.
- **Add stuck detection** using image similarity or motor feedback.
- **Experiment with different camera positions** for improved detection of low or high obstacles.
- **Add TPU** such as coral USB from google to increase amount of FPS
---

**Acknowledgements:**  
Thanks to the WRO community and open-source contributors for inspiration and code samples.
