# Known Issues & Optimization Log

Technical observations, edge cases, and future enhancements identified during flight testing.

## 1. Direct Sun Glare & Low-Light Contrast

* **Observation**: In extreme low-light conditions (dusk/night), or when the target moves directly into strong low-angle sunlight, confidence scores for the 320x320 MobileNetV2 SSD model drop below `0.35`.
* **Mitigation**: The `camera_ai_start.py` node caches valid detections across dropped inference cycles. For low-light operations, deploy the `imx500_network_nanodet_plus_416x416_pp.rpk` model.

## 2. Multi-Person Proximity & Crossing

* **Observation**: When a second person crosses directly in front of the active target, bounding boxes may momentarily merge.
* **Mitigation**: ByteTrack uses two-stage matching with motion prediction from the Kalman filter, allowing the system to maintain the target ID across brief crossings ($< 1.0	ext{ s}$).

## 3. Acceleration Limits & Wind Compensation

* **Observation**: In strong crosswinds ($> 15	ext{ km/h}$), the velocity smoother acceleration clamp ($a_{max} = 0.5	ext{ m/s}^2$) may cause a slight tracking lag during rapid lateral sprints.
* **Mitigation**: Increase `max_ay` to `0.8	ext{ m/s}^2` and adjust `kp_strafe` for outdoor windy environments.

---

<div align="center">

| [⬅️ Previous: **11 - Troubleshooting**](troubleshooting.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [🏠 **Return to Project Home**](../README.md) |
| :--- | :---: | ---: |

</div>
