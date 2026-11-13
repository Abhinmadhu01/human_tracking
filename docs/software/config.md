# Master Configuration Reference

All tunable parameters are configured in [`src/human_tracking_kf/config/tracker_params.yaml`](../../src/human_tracking_kf/config/tracker_params.yaml).

## Camera & AI Model Parameters

| Parameter | Default | Unit | Description |
| :--- | :---: | :---: | :--- |
| `model_path` | `imx500_network_ssd_...rpk` | string | Filename of `.rpk` model inside `src/detection_model/model/` |
| `resolution` | `"640x480"` | string | Camera resolution width x height |
| `fps` | `30` | Hz | Camera acquisition frame rate |
| `conf_threshold` | `0.40` | float | Minimum confidence score to accept detection |
| `camera_focal_length_px` | `460.64` | px | Calibrated camera focal length for distance calculation |

## Pursuit & Target Setpoints

| Parameter | Default | Unit | Description |
| :--- | :---: | :---: | :--- |
| `target_class` | `"person"` | string | Vision detection class filter |
| `target_altitude` | `8.0` | m | Desired hover flight altitude AGL |
| `target_distance_m` | `1.0` | m | Desired horizontal standoff distance to target |
| `distance_tolerance_m` | `0.2` | m | Dead-band threshold for distance regulation |
| `person_height_m` | `1.70` | m | Standard human height reference for pinhole math |
| `fwd_sign` | `1.0` | sign | Camera forward orientation sign (+1 = forward, -1 = reverse) |

## Geofence & Safety Limits

| Parameter | Default | Unit | Description |
| :--- | :---: | :---: | :--- |
| `max_range_m` | `50.0` | m | Maximum allowable distance from takeoff home position |
| `min_altitude` | `2.0` | m | Hard altitude floor (vehicle will never descend below) |
| `max_altitude` | `50.0` | m | Hard altitude ceiling |
| `detection_heartbeat_timeout` | `10.0` | s | Trigger failsafe if no detection message received |
| `failsafe_action` | `"hover"` | string | Action on failsafe: `"hover"`, `"rtl"`, or `"land"` |

## PID Controller Gains

| Loop | $k_p$ | $k_i$ | $k_d$ | Clamp Limit |
| :--- | :---: | :---: | :---: | :---: |
| **Strafe (Lateral)** | `0.01` | `0.0` | `0.0001` | $\pm 0.6	ext{ m/s}$ |
| **Distance (Longitudinal)** | `0.15` | `0.0` | `0.002` | $\pm 0.6	ext{ m/s}$ |
| **Yaw (Heading)** | `0.003` | `0.0002` | `0.0001` | $\pm 0.5	ext{ rad/s}$ |
| **Altitude** | `0.5` | `0.0` | `0.0` | $\pm 0.5	ext{ m/s}$ |

---

<div align="center">

| [⬅️ Previous: **08 - ROS 2 Nodes**](nodes.md) | [📖 **Documentation Roadmap**](../../README.md#documentation-roadmap) | [➡️ Next: **10 - Video Streaming**](../gstreamer-streaming.md) |
| :--- | :---: | ---: |

</div>
