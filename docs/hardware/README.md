# Hardware Setup & Wiring

Complete guide for building the vehicle avionics, companion computer, power regulation, and sensor harness.

## Bill of Materials

| Component | Part Name / Specification | Purpose |
| :--- | :--- | :--- |
| **Flight Controller** | CubePilot Cube Orange+ | Dual IMU, running ArduCopter firmware |
| **Companion Computer** | Raspberry Pi 5 (8 GB) | Runs Docker, ROS 2 Humble, Picamera2, and tracking stack |
| **Vision Sensor** | Sony IMX500 AI Camera (Raspberry Pi AI Camera) | On-sensor neural network acceleration |
| **Camera Mount** | Custom 3D Printed Mount (45° Tilt) | Rigid mechanical fixture angled forward-downward |
| **Power Converter** | 5 V / 5 A High-Efficiency Buck BEC | Clean, isolated power for Raspberry Pi 5 |
| **Power Module** | Cube Power Module | Regulated power and battery telemetry for Cube |
| **Radio Control** | RadioMaster TX16S + ELRS/CRSF Receiver | Primary manual control and mode override |
| **Telemetry Link** | TP-Link High-Gain USB Wi-Fi Adapter | High-bandwidth ground link for ROS topics and QGC video |

### Key Hardware Components

| Cube Orange+ | IMX500 AI Camera | RadioMaster TX16S Transmitter |
| :---: | :---: | :---: |
| ![Cube Orange+ packaging box](images/hardware_cubepilot_orange_plus_packaging_box_v2.jpg) | ![Sony IMX500 camera module](images/hardware_rpi_ai_camera_imx500_module_v1.jpg) | ![RadioMaster TX16S](images/hardware_radiomaster_tx16s_rc_transmitter_gimbal_switches_v1.jpg) |

## Interconnect Architecture

<p align="center">
  <img src="connection_diagram/companion_computer_connection_diagram.png" width="850" alt="Companion Computer Avionics Connection Diagram" />
  <br>
  <em>Companion computer and flight controller avionics interconnect diagram showing power, USB data, and CSI interfaces.</em>
</p>

```text
                  LiPo Battery (4S 5000 mAh)
                 +------------+------------+
                 |                         |
         Cube Power Module        5V / 5A Buck Converter
                 |                         | (USB-C 5.1V)
                 v                         v
            Cube Orange+ <====USB====> Raspberry Pi 5 <====CSI====> Sony IMX500 Camera
                 |                         |                         (45° Forward-Down)
                 +-> GPS / Compass         +-> TP-Link Wi-Fi Adapter
                 +-> RC Receiver           +-> Active Cooler
                 +-> ESCs & Motors
```

## Power Warning for Raspberry Pi 5

The Raspberry Pi 5 requires a steady **5 V / 5 A (25 W)** power source. Powering the Pi 5 from the flight controller's internal rail or an undersized 3A BEC will trigger CPU undervoltage throttling (`0x50000`), USB brownouts, or sudden companion computer reboots in flight.

| 5V / 5A BEC Regulator | Multimeter Verification Test |
| :---: | :---: |
| ![BEC voltage regulator](images/hardware_bec_voltage_regulator_usb_c_module_v2.jpg) | ![Multimeter 5V test](images/hardware_multimeter_5v_bec_voltage_test_v1.jpg) |
| *High-amperage buck converter with clean USB-C output* | *Always measure BEC output voltage before connecting the Pi* |

## Camera Connection (CSI Port CAM0)

The Sony IMX500 AI camera connects to the Raspberry Pi 5 using a 22-to-15 pin ribbon cable on **CAM0**.

| Step 1: Insert Ribbon Cable | Step 2: Lock CAM0 Port |
| :---: | :---: |
| ![CSI ribbon cable insertion](images/hardware_rpi5_csi_ribbon_cable_insertion_v2.jpg) | ![Camera connected to CAM0](images/hardware_rpi5_csi_port_cam0_connected_v2.jpg) |
| *Lift retaining latch and slide ribbon cable in squarely* | *Firmly seat ribbon cable and press latch down to lock* |

## Fury Drone Human Tracking Setup (Multi-Angle Views)

The Sony IMX500 camera is rigidly mounted to the front undercarriage of the Fury quadcopter at a fixed 45° downward angle:

| Front View | Side View | Bottom Angled View | Top Angled View |
| :---: | :---: | :---: | :---: |
| ![Front view](images/hardware_fury_drone_human_tracking_front_view.jpg) | ![Side view](images/hardware_fury_drone_human_tracking_side_view.jpg) | ![Bottom angled view](images/hardware_fury_drone_human_tracking_bottom_angled_view.jpg) | ![Top angled view](images/hardware_fury_drone_human_tracking_top_angled_view.jpg) |
| *Front elevation showing 45° lens pitch* | *Side profile showing standoff from frame* | *Undercarriage chassis mount* | *Top view showing active cooler clearance* |

## Drone Evolution & Prototype Platforms

The human tracking stack was validated across both prototype and production airframes:

| ICFOSS V1 Tracking Drone (Top View) | ICFOSS V1 Tracking Drone (Side View) | ICFOSS V2 Prototype Setup |
| :---: | :---: | :---: |
| ![Fully assembled V1 drone top view](images/fully_assembled_v1_drone_top_angled_view.jpg) | ![Fully assembled V1 drone side view](images/fully_assembled_v1_drone_side_angled_view.jpg) | ![Drone V2 setup front view](images/drone_v2_setup_front_angled_view.jpg) |
| *ICFOSS V1 tracking drone with damper plate* | *Side view with battery and Cube Orange+* | *ICFOSS V2 bench prototype setup* |

## Bench Setup & Interconnect

| Bench Hardware Setup | Full Interconnect with Wi-Fi Link |
| :---: | :---: |
| ![Bench setup with Pi 5, IMX500 and BEC](images/hardware_bench_setup_rpi5_imx500_bec_complete_v1.jpg) | ![Complete bench interconnect with Wi-Fi](images/hardware_bench_interconnect_complete_with_wifi_v1.jpg) |
| *Pi 5, IMX500 AI camera, and dedicated BEC power* | *Full avionics harness with Cube Orange+ and Wi-Fi dongle* |

---

<div align="center">

| [⬅️ Previous: **01 - Safety Guide**](../../SAFETY.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **03 - Camera Holder Mount**](camera-holder/README.md) |
| :--- | :---: | ---: |

</div>
