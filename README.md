# TAMIR: Training Assistive Mobile Intelligent Robot

## Overview

**TAMIR** is an intelligent, mobile robotic system designed to monitor and correct unwanted pet behavior autonomously. It integrates computer vision, SLAM-based navigation, Bluetooth feedback, and geofencing using AprilTags to deliver real-time corrective audio cues. The system is powered by a modular ROS 2 backend and a modern React frontend for visualization.

---

## Key Features

### 🔍 Behavior Detection
- Uses YOLOv8 to detect pet behavior (e.g., entering restricted zones).
- Detects AprilTags for real-time geofence awareness.

### 🧠 Edge AI Integration
- Efficient inference using Raspberry Pi 5 and Hailo AI HAT.
- Real-time behavior classification and localization.

### 🗺️ Navigation & SLAM
- ROSbot 2R with 2D LiDAR + Astra depth camera.
- Nav2 stack for SLAM-based autonomous navigation.

### 📶 Bluetooth Feedback
- Bluetooth-based corrective cues through waterproof collar speakers.
- Audio playback triggered via ROS2 services.

### 📡 ROS 2 Modular Architecture
- Nodes include:
  - `TamirInterface`: Central node for service orchestration.
  - `Bluetooth_Node`: Scans and connects to speakers.
  - `OptimizedTrackerNode` & `YoloVisualizer`: YOLO inference and 3D spatial mapping.
  - `CameraLocalizer`: Publishes static TFs for localization.

---

## Project Structure

```
TAMIR/
├── tamir/               # ROS 2 package with nodes & services
│   ├── tamir_interface/
│   ├── nodes/
│   └── msg/
└── assets/              # Audio files, videos, AprilTag config
```

---

## Installation

### 🤖 Backend (ROS 2)

Ensure ROS 2 Foxy/Humble is installed and sourced.

```bash
cd ~/ros2_ws/src
git clone https://github.com/asarogers/TAMIR
cd ..
colcon build --packages-select tamir
source install/setup.bash
```

### 🔊 Audio Dependencies

Install ffmpeg for audio playback:

```bash
sudo apt install ffmpeg mpg321
```

---

## Running the System

**1. Start ROS Core:**

```bash
ros2 launch tamir bringup.launch.py
```

**2. Start the React Frontend:**

```bash
cd frontend
npm start
```

---

## Usage

- **Behavior Detection:** Pets are detected via YOLO. If a pet enters a restricted zone, corrective audio is played.
- **Visual Monitoring:** Frontend dashboard displays system status and demo videos.
- **Manual Triggering:** ROS services like `/correctiveSignal`, `/scan_for_devices`, and `/pair_bluetooth` can be triggered manually.

---

## Demos

- Teleoperation & SLAM Mapping
- Behavior Detection via Camera Feed
- Geofence Intrusion and Real-time Correction

---

## Authors

- Asa Rogers  

---

## License


