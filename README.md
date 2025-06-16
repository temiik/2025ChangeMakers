# 🛠️ Team [Your Team Name] — WRO Future Engineers 2024-25 Documentation

> This repository documents our journey in building an autonomous vehicle for the WRO Future Engineers 2024-25 challenge, using a hybrid approach with LEGO EV3 and Raspberry Pi for robust, real-time vision and control.

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

- **Chassis:** Custom or RC-based, adapted for WRO FE size constraints.
- **Steering:** Front-wheel Ackerman steering, providing realistic turning dynamics and improved maneuverability compared to differential drive.
- **Drive:** Rear-wheel drive, controlled via EV3 motors.

### Sensors

- **Ultrasonic Sensors (EV3):** Mounted on both sides for precise centering between walls.
- **Camera (Raspberry Pi):** Wide-angle CSI camera for robust color and object detection.

### Computing Units

- **Raspberry Pi 4:** Runs Python 3, OpenCV, and all vision algorithms.
- **LEGO EV3 Brick:** Runs ev3dev (Debian-based OS), controls motors and sensors, receives commands from Pi.

---

## Software Overview

### EV3dev & Raspberry Pi Setup

- **EV3dev** installed on the EV3 brick for full Linux compatibility and Python support.
- **Raspberry Pi** runs Raspbian OS with OpenCV and all necessary Python libraries.
- **SSH keys** set up for passwordless communication between Pi and EV3.

### Communication (SSH)

- **Socket-based protocol:** The Pi opens a TCP socket to the EV3, sending simple string commands (e.g., `red,640,300`, `backup`, `prepare_green`).
- **EV3 listens** for commands, parses them, and executes corresponding motor/sensor actions.

### Vision & Detection (OpenCV)

- **LAB color space** is used for robust color detection, minimizing the effect of lighting changes.
- **Gaussian blur, erosion, and dilation** are applied to clean up the image and improve contour detection.
- **Contours** are extracted for pillars, lines, and walls.
- **ROI (Region of Interest):** Only relevant parts of the image are processed for each task, improving speed and reliability.

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
- **Why?** This method is robust to visual noise and ensures the robot doesn't drift toward a wall.

### Wall Avoidance with Camera

- **How it works:** The Pi analyzes the lower part of the image for large black regions (walls). If detected, it sends a `backup` command to the EV3.
- **Why?** This allows the robot to react to obstacles that the ultrasonics might miss, especially at sharp angles or when approaching corners.

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

---

## Team & Contact

<h1> Our team </h1>
<img src="https://github.com/temiik/2025ChangeMakers/blob/main/t-photos/funny.jpeg?raw=true">
Yesken Kairat 16 kairatyesken@gmail.com
Tolendi Temirlan 16
Yusuf Mukhambetkaliev 16


**Acknowledgements:**  
Thanks to the WRO community and open-source contributors for inspiration and code samples.
