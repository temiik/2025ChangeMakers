# 📦 Software Documentation and Code Overview

This folder contains all the source code for our autonomous car project built on LEGO EV3 and Raspberry Pi for the WRO Future Engineers 2025 category. The robot integrates two computing platforms: a LEGO EV3 Brick and a Raspberry Pi 4/5, connected via a USB cable. Raspberry Pi is responsible for real-time image processing and color-based object detection, while EV3 handles direct motor and sensor control, driving logic, lap counting, and obstacle avoidance maneuvers.

---

## 📐 Project Architecture

The control system of the robot is split into two independent Python programs:

- **On Raspberry Pi:**
  - Captures video from a webcam.
  - Performs color detection using either **LAB** or **HSV** color spaces.
  - Sends object detection results via TCP to the EV3 brick.

- **On EV3 Brick:**
  - Receives TCP messages from Raspberry Pi.
  - Executes driving, steering, and obstacle avoidance logic.
  - Monitors ultrasonic sensors, a touch sensor, and a color sensor.
  - Performs lap counting and obstacle navigation based on camera detections.

---
---

## 📜 Description of Each Script

### Raspberry Pi Programs (src/raspberry/)

- **camera_stream.py**
  - Captures live video stream from USB webcam.
  - Prepares image frames for color detection.

- **object_detect_lab.py**
  - Converts captured frames to LAB color space.
  - Applies inRange mask and detects largest color object based on calibrated thresholds.
  - Sends object position and color info to EV3 via TCP in real time.

- **object_detect_hsv.py**
  - Same functionality as object_detect_lab.py but using HSV color space.

- **tcp_client.py**
  - Opens a TCP socket connection to EV3 Brick.
  - Sends formatted detection data such as `color,x_center,y_center` or `none` to EV3.

- **thresholds_picker.py**
  - OpenCV window with trackbars to calibrate lower and upper color thresholds for either LAB or HSV.
  - Allows visual tuning of detection masks using images or webcam feed.
  - Saves final values to JSON config files for robot use.

---

### EV3 Programs (src/ev3/)

- **ev3_driver.py**
  - Listens for incoming TCP messages from Raspberry Pi.
  - Parses color and position data.
  - Controls drive and steering motors.
  - Executes turns or continues forward movement.
  - Reads sensor values (ultrasonic, color, touch) for lap counting and collision prevention.

- **turn_logic.py**
  - Contains `turn_left()` and `turn_right()` functions.
  - Moves steering motor to preset angle, drives forward, then realigns to center.
  - Monitors for object disappearance (via detection result "none") before completing the maneuver.

- **sensor_utils.py**
  - Reads ultrasonic distance from left and right sensors.
  - Checks color sensor for lap counting.
  - Monitors touch sensor for starting or aborting runs.

---

## 📡 Communication Overview

| From                | To               | Protocol | Purpose                         |
|:-------------------|:----------------|:----------|:---------------------------------|
| Raspberry Pi (Python)| EV3 Brick (Python) | TCP Socket | Sends detected color and object position |

- Port: **12345**
- Message format:  
  `color,x_center,y_center`  
  or  
  `none` (if no object detected)

Example:  
`green,320,240` or `none`

---

## 🎨 Color Detection Modes

The robot can detect color-coded traffic signs (red or green columns) using either:

- **LAB Color Space**
  - More resistant to lighting changes.
  - Tuned via `thresholds_picker.py` and saved as `lab_thresholds.json`.

- **HSV Color Space**
  - Easier to visualize and adjust.
  - Tuned via `thresholds_picker.py` and saved as `hsv_thresholds.json`.

Switching between them is as simple as running either `object_detect_lab.py` or `object_detect_hsv.py`.

---

## 📸 Lap Counting and Obstacle Avoidance

- **Color Sensor (INPUT 4)**
  - Detects colored marks on the track.
  - Increments lap counter on each pass.
  - Terminates round after 12 laps.

- **Ultrasonic Sensors (INPUT 1 & INPUT 2)**
  - Detect obstacles on left and right sides.
  - Used to recenter robot after avoiding an obstacle.

- **Touch Sensor (INPUT 3)**
  - Starts run.
  - Can be used to abort maneuver or reset system.

---

## 📄 How It Works (Summary)

1. Raspberry Pi reads camera feed and performs object detection.
2. Sends result via TCP to EV3.
3. EV3 receives color and position.
4. If obstacle is detected:
   - Turns left or right based on color.
   - Continues until object disappears (detection result = `none`).
   - Recenters via ultrasonic sensors.
5. Continues lap counting using color sensor.
6. Stops after completing 12 laps.
