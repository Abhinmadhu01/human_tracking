# Software Setup & Execution

Guide for deploying the ROS 2 Humble autonomous tracking stack on Raspberry Pi 5.

## Architecture

The system utilizes a hybrid OS architecture:
* **Host OS**: Raspberry Pi OS (64-bit Bookworm). The Sony IMX500 hardware drivers and Picamera2 hardware firmware run natively on the host to avoid virtualization overhead for camera silicon access.
* **Container Environment**: Docker container running Ubuntu 22.04 LTS, ROS 2 Humble, and MAVROS.

```text
Raspberry Pi 5 (Host - Raspberry Pi OS 64-bit)
  ├── IMX500 Driver & Picamera2 Silicon Access
  └── Docker Engine
        └── Container: Ubuntu 22.04 LTS
              ├── ROS 2 Humble
              ├── MAVROS (/dev/ttyACM0 to Cube)
              ├── bytetrack_tracker node
              ├── human_tracker_pro node
              └── gstream_human_detection.py
```

## 1. Build the Workspace

In the ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select detection_model human_tracking_kf
source install/setup.bash
```

## 2. Launch the System

Start the full tracker and flight controller:

```bash
ros2 launch human_tracking_kf bytetrack_launch.py
```

To override the default detection model or altitude:

```bash
ros2 launch human_tracking_kf bytetrack_launch.py \
    target_altitude:=5.0 \
    custom_model_path:=src/detection_model/model/imx500_network_nanodet_plus_416x416_pp.rpk
```

## 3. Verify Topics

Confirm nodes and communication:

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic hz /detections
ros2 topic hz /tracked_detections
ros2 topic echo /mavros/state --once
```

---

<div align="center">

| [⬅️ Previous: **06 - SITL Simulation**](../simulation.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **08 - ROS 2 Nodes & Interfaces**](nodes.md) |
| :--- | :---: | ---: |

</div>
