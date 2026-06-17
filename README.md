# 🤖 Elder Helper Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-34A853?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python)
![YOLOv8](https://img.shields.io/badge/YOLO-v8_NCNN-yellow)
<img width="896" height="1195" alt="Elder_Helper_Mobile_Robot" src="https://github.com/user-attachments/assets/b21f8125-fcee-4b3b-901b-b21aee3f6c0b" />


**Elder Helper** is a semi-autonomous, vision-guided robotic assistant designed to provide an intuitive, contactless interface for elderly support in indoor environments. By recognizing natural hand gestures, the robot navigates between rooms and interacts with its environment—replacing the need for joysticks, apps, or remote controls.

---

## 📖 Project Overview

The system operates on a continuous 3-step cycle:
1. 👁️ **See:** A fixed-buffer USB webcam captures user hand gestures (0-5 fingers).
2. 🧠 **Think:** An onboard Raspberry Pi runs a dual-threaded YOLOv8 (NCNN) AI pipeline to classify the gesture asynchronously, preventing camera lag.
3. ⚙️ **Act:** The ROS 2 Jazzy high-level navigation node translates the gesture into a target waypoint and dispatches it to an ESP32 microcontroller via a `micro_ros_agent` Wi-Fi bridge for physical execution.

---

## 🛠️ System Architecture & Node Topology

The software stack is built on a distributed **ROS 2 Jazzy** publish/subscribe model.

| Node | Primary Function | Core Subscriptions | Core Publications |
| :--- | :--- | :--- | :--- |
| `vision_node` (`v_n.py`) | Asynchronous YOLO inference & camera streaming. | *None* | `/vision/finger_count`, `/vision/raw_stream/compressed` |
| `high_level_nav` (`nav_n.py`) | State machine & waypoint dispatch. | `/vision/finger_count`, `/robot_reached` | `/robot_target` (Int32) |
| `sys_mon` (`sys_mon.py`) | Real-time Raspberry Pi telemetry tracking. | *None* | `/pi_stats` |
| `ext_mechCTRL` (`ext_mechCTRL.py`) | GPIO edge-detection & servo/LED hardware PWM. | `/extra_man_flag` | *Physical Hardware Action* |
| `micro_ros_agent` | UDP Wi-Fi bridge to the ESP32 (Port 8888). | `/robot_target` | `/robot_reached`, `/robot_odom` |

---

## 🤚 Gesture-to-Command Mapping

| Gesture (Fingers) | System Command | Action |
| :---: | :--- | :--- |
| **0 (Fist)** | `EMERGENCY STOP` | Halts all motor function immediately. |
| **1** | `SELECT MODE_1` | Navigate to Room 1 (e.g., Bathroom). |
| **2** | `SELECT MODE_2` | Navigate to Room 2 (e.g., Bedroom). |
| **3** | `SELECT MODE_3` | Navigate to Room 3 (e.g., Garden). |
| **4** | `SELECT MODE_4` | Auxiliary/Custom Mode. |
| **5 (Open Hand)** | `START MISSION` | Commences autonomous operation. |

*(Note: The vision node utilizes a smoothing tracker requiring multiple consecutive identical frames to prevent flickering and false positives).*

---

## ⚙️ Hardware Integration

* **Main Compute (Brain):** Raspberry Pi 4/5 (Ubuntu 24.04 / ROS 2 Jazzy).
* **Low-Level MCU (Motor Control):** ESP32 linked via UDP.
* **Sensors & Actuators:**
  * Standard USB Webcam (640x480 resolution).
  * Digital Light Sensor (GPIO 4).
  * 10-LED WS2812B NeoPixel Strip (GPIO 12 / DMA).
  * Hardware PWM Servo Motor (GPIO 18).

---

## 🚀 Installation & Setup

### Prerequisites
* Ubuntu 24.04 with **ROS 2 Jazzy** installed.
* Python virtual environment recommended for YOLO dependencies.

### 1. Install System Dependencies
```bash
sudo apt update
sudo apt install python3-pip libgpiod2
pip3 install ultralytics opencv-python inference-sdk psutil rpi_ws281x

