# 📦 Hardware Documentation and Setup Guide

This autonomous robot consists of two processing units — a Raspberry Pi and a LEGO EV3 Brick. The chassis is based on a modified LEGO EV3 Ackermann steering platform. The robot is equipped with multiple sensors and motors and is controlled by a computer vision algorithm for obstacle avoidance and lap counting.

---

## 📑 `schemes/` — Connection Diagrams

This directory contains all connection and wiring schemes for the LEGO EV3 and Raspberry Pi system.


## 📡 Hardware Connection Table:

| Device                        | Type             | Port / Connection     | Function                                |
|:-----------------------------|:----------------|:---------------------|:----------------------------------------|
| **Medium Motor (drive)**      | Motor            | EV3 OUTPUT B          | Robot forward/reverse movement          |
| **Medium Motor (steering)**   | Motor            | EV3 OUTPUT C          | Controls wheel steering angle           |
| **Ultrasonic Sensor (left)**  | Digital Sensor   | EV3 INPUT 1           | Measures distance to left obstacles     |
| **Ultrasonic Sensor (right)** | Digital Sensor   | EV3 INPUT 2           | Measures distance to right obstacles    |
| **Touch Sensor**              | Digital Sensor   | EV3 INPUT 3           | Start signal / emergency stop           |
| **Color Sensor**              | Color Sensor     | EV3 INPUT 4           | Lap counting based on markers           |
| **Raspberry Pi 4/5**          | Single Board PC  | USB-A to Mini-USB EV3 | Image capture and color detection       |
| **Webcam**                    | USB Camera       | Raspberry Pi          | Real-time track and obstacle streaming  |
| **PowerBank**                 | Battery          | Raspberry Pi 5V       | Raspberry Pi power supply               |
| **EV3 battery**               | Battery          | EV3 Brick             | Motors and sensors power supply         |


---

## 📄 USB Communication:

Raspberry Pi connects to LEGO EV3 via USB cable:
- **EV3 Brick**: Mini-USB side port
- **Raspberry Pi**: USB-A port

TCP Socket Protocol, port `12345`.

---

## 📌 Connection Rules:

- Always connect motors and sensors while the EV3 Brick is powered off.
- Ensure cables do not obstruct the camera’s field of view.
- Verify all connector tightness before running the robot.

---

## 📄 System Operation:

- Raspberry Pi captures and processes video frames.
- It identifies pole position and color.
- Sends driving commands to EV3 Brick over TCP.
- EV3 handles turning or forward movement.
- Ultrasonic sensors handle post-obstacle alignment.

---

## 📦 `schemes/` Directory Contents:

Contains all wiring and connection schemes:
- EV3 Brick port wiring.
- Raspberry Pi to EV3 communication link.
- Sensor and camera placement photo layout.

Each diagram is labeled for clarity.
