# Future Engineering — Regional Stage WRO 2025
## Introduction
### Welcome to the documentation of our project for the VRO 2025 Regional Stage in the Future Engineering category. Our team has developed an autonomous robotic vehicle, which integrates various hardware components to perform a range of tasks. This vehicle is built using the Raspberry Pi 4B and other high-performance electronic components to ensure precise movement, sensing, and power efficiency.

Components
- Raspberry Pi 4B 8GB: The core of the system, providing computational power and managing the overall control of the robot.

- Raspberry Pi Camera Module V2: Used for visual processing and object recognition.

- L298N Motor Driver: Responsible for controlling the motors, allowing for bidirectional movement.

- Motor 12B: Drives the wheels of the robot, enabling motion.

- Servo MG996R: Used for controlling specific movements, such as turning or adjusting angles.

- Arduino Uno: Acts as a secondary controller to interface with certain components and manage additional sensors.

- Laser Ranger Tracker GY-530 VL53LDK: A distance sensor used for precise obstacle detection and navigation.

- LiPo Battery 5000mAh 3S 100C: Powers the entire system, providing the necessary energy for operation.

- Power Reduction Regulator DC-DC YH11060D 200W 12A: Ensures stable and efficient power distribution to all components.

This project aims to combine hardware and software for a fully autonomous robotic system capable of navigating its environment, detecting obstacles, and performing various tasks with high efficiency. The vehicle is designed to be scalable and adaptable to various challenges, ensuring robust performance in diverse environments.

This repository contains the full documentation, source code, and setup instructions for building and operating the robotic vehicle. We hope this project demonstrates the potential of integrating cutting-edge technology to create intelligent systems capable of solving real-world problems


<h1> Our team </h1>
<img src="https://github.com/temiik/2025ChangeMakers/blob/main/t-photos/funny.jpeg?raw=true">
Yesken Kairat 16 kairatyesken@gmail.com
Tolendi Temirlan 16
Yusuf Mukhambetkaliev 16

<h1> Engineering solution </h1>

<img src="https://github.com/temiik/2025ChangeMakers/blob/main/v-photos/front.jpeg?raw=true">

🔌 Power Supply System
Our system uses a well-thought-out power distribution setup to ensure stable operation of all components during both autonomous and testing phases:

The Raspberry Pi 4B (8GB) is powered via a power bank connected through the USB-C port. This provides a stable 5V supply with sufficient current for the Pi and the camera module.

The Arduino Uno, VL53L0X laser distance sensors, MG996R servo motor, and L298N motor driver are powered by a 3S 5000mAh 100C LiPo battery. This delivers a nominal voltage of 11.1V, suitable for running motors and power-hungry components.

To safely and efficiently regulate voltage, we use a DC-DC buck converter (YH11060D, 200W, 12A) to step down the 11.1V from the LiPo battery to 5V or 9V, depending on the requirements of each component (e.g., Arduino or servo motors).

The L298N motor driver receives direct power from the LiPo battery through its VIN and GND pins, while its logic level (5V) can be supplied from the onboard regulator or from the DC-DC converter output.

This power configuration allows for efficient load distribution and ensures reliable performance of all hardware in real-world field conditions.

<img src="blob:https://web.whatsapp.com/bb2e8a98-d533-4a8a-b707-593eac917fbc">

We made big progress since first version of our vehicle, because first one was using differential system of moving, that is prohibited.
Also we use raspberry pi 4B instead of esp32 cam, that gives more power for object detection.

## 🧠 Object Recognition Using Raspberry Pi

To recognize red and green columns, we implemented a computer vision system using the **Raspberry Pi 4B** and the **Raspberry Pi Camera Module v2**. The Raspberry Pi captures real-time video frames and processes them using Python and OpenCV.

The recognition logic is based on color detection in the **HSV** (Hue, Saturation, Value) color space. This approach allows for more reliable detection under varying lighting conditions.

### Detection Process:
1. Capture a frame from the camera.
2. Convert the image to HSV color space.
3. Apply color filters to detect **green** and **red** objects.
4. Use contours to identify the shape and size of the columns.
5. Send a specific code (`1` for green, `2` for red, `0` if none) to the Arduino Uno via serial communication.

### Python Code Example:
```python
import cv2
import serial
import time

# Setup serial connection to Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges
    green_lower = (40, 70, 70)
    green_upper = (80, 255, 255)
    red_lower1 = (0, 70, 50)
    red_upper1 = (10, 255, 255)
    red_lower2 = (170, 70, 50)
    red_upper2 = (180, 255, 255)

    # Create masks
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = red_mask1 | red_mask2

    # Detect objects and send signal to Arduino
    if cv2.countNonZero(green_mask) > 500:
        arduino.write(b'1\n')
    elif cv2.countNonZero(red_mask) > 500:
        arduino.write(b'2\n')
    else:
        arduino.write(b'0\n')
###
## Table of content
## [Models](https://github.com/temiik/2025ChangeMakers/tree/main/models)
## [Schemes](https://github.com/temiik/2025ChangeMakers/tree/main/schemes)
## [Hardware](https://github.com/temiik/2025ChangeMakers/tree/main/other/All%20components)
## [Software](https://github.com/temiik/2025ChangeMakers/tree/main/src)
## [Team photos](https://github.com/temiik/2025ChangeMakers/tree/main/t-photos)
## [Videos](https://github.com/temiik/2025ChangeMakers/tree/main/video)
## [Vehicle Photos](https://github.com/temiik/2025ChangeMakers/tree/main/v-photos)









## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
