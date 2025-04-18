# TAMIR: Training Assistive Mobile Intelligent Robot

## Overview

**TAMIR** is an intelligent ROS 2-based robotic system built to autonomously detect and correct pet behavior. By combining computer vision, SLAM navigation, geofencing with AprilTags, and Bluetooth-based corrective feedback, TAMIR delivers real-time behavior-aware robotic supervision in home environments.

---

## Key Features

- 🧠 **YOLOv8 Detection:** Real-time pet behavior classification.
- 🗺️ **SLAM & Navigation:** Powered by ROSbot 2R with 2D LiDAR + Astra depth camera.
- 🔊 **Corrective Feedback:** Bluetooth speaker collars provide audio cues.
- 🧭 **AprilTag Geofencing:** Defines restricted zones spatially.
- 🛠️ **Modular ROS 2 Architecture:** Includes YOLO tracker, Bluetooth handler, interface nodes, and TF broadcasters.

---

## Project Structure

```
TAMIR/
├── tamir/               # ROS 2 package with nodes & services
│   ├── tamir_interface/
│   ├── nodes/
│   └── msg/
└── assets/              # Audio, tag configs, videos
```

---

## Setup Instructions

### 🔗 Networking via Husarnet

1. Create account at [Husarnet](https://app.husarnet.com)
2. Add new element, get a `JOINCODE`
3. On both laptop & ROSbot:

```bash
export JOINCODE="your_join_code_here"
sudo husarnet join $JOINCODE rosbot2r   # On ROSbot
sudo husarnet join $JOINCODE my-laptop  # On Laptop
```

4. To verify ROSbot connectivity:
```bash
fping -a -g 172.20.10.0/24
ssh husarion@<ROSBOT_IP>
```

---

## Docker & Firmware Setup

### ⚙️ Flash ROSbot Firmware

```bash
docker compose pull

docker stop rosbot microros || true && docker run \
--rm -it --privileged \
husarion/rosbot:humble-0.6.1-20230712 \
flash-firmware.py /root/firmware.bin

docker compose up -d rosbot microros
```

---

## Running the System

### 🗺️ Start Mapping (SLAM)

```bash
just start-rosbot        # On robot
just start-pc            # On PC with RViz
```

### 🕹️ Teleoperation

```bash
just run-teleop
```

---

## Camera & Perception

### 📷 Setup Telepresence Camera

On both PC and ROSbot:

```bash
git clone https://github.com/husarion/rosbot-telepresence.git
cd rosbot-telepresence
just sync                # PC
just flash-firmware      # ROSbot
just start-pc            # PC
just start-rosbot        # ROSbot
```

### 🚀 View Camera Feed in RViz

```bash
docker compose up -d microros astra
rviz2                     # Then select: /camera/color/image_raw
```

---

## SLAM & Navigation Setup

### 🔧 Install SLAM Toolbox

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

### 🧭 Docker Compose Example (SLAM & Nav2)

```yaml
# compose.tamir.yaml (abridged)
services:
  mapping:
    image: husarion/slam-toolbox:humble
    command: >
      ros2 launch slam_toolbox online_sync_launch.py
        slam_params_file:=/slam_params.yaml
  navigation:
    image: husarion/navigation2:humble
    command: >
      ros2 launch nav2_bringup navigation_launch.py
        params_file:=/nav2_params.yaml
```

```bash
docker compose -f compose.tamir.yaml up -d mapping navigation
```

---

## Development Tools

### 🐳 Running Docker GUI Containers

```bash
xhost +local:docker

docker run -it --net=host \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  humble_tamir:latest bash
```

---

## ROS Topics & Services

- `/behavior_msg`: Publishes pet behavior events.
- `/correctiveSignal`: Triggers Bluetooth sound feedback.
- `/scan_for_devices`, `/pair_bluetooth`: Bluetooth services.

---

## Authors

- **Asa Rogers**

---

## License

Licensed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0)

