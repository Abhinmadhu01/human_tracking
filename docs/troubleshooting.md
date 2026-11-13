# Troubleshooting & Flight Diagnostics

Diagnostic matrix for resolving common issues during bench testing, detection, and flight control.

| Symptom | Probable Cause | Corrective Action |
| :--- | :--- | :--- |
| **Drone does not respond in GUIDED mode** | Flight controller not armed or flight mode guard blocking | Confirm `/mavros/state` shows `mode: GUIDED` and `armed: true`. Check `allowed_modes` in `tracker_params.yaml`. |
| **Detection bounding boxes flicker or drop** | Missing inference frames from camera silicon | Ensure `camera_ai_start.py` anti-flicker caching is active. Lower `conf_threshold` from `0.50` to `0.35`. |
| **Drone oscillates or hunts laterally** | Lateral strafe gain $k_p$ too aggressive | Lower `kp_strafe` in `tracker_params.yaml` (e.g., from `0.01` to `0.007`) and raise damping `kd_strafe`. |
| **Drone moves backward when person advances** | Inverted forward sign or wrong image axis | Verify `fwd_sign: 1.0` in `tracker_params.yaml`. Bench test with props off to confirm commanded velocity signs. |
| **Companion Pi reboots during flight** | Undervoltage / insufficient BEC power | Measure BEC rail with multimeter under full load. Replace with dedicated 5V / 5A buck converter. |
| **No video appears in QGroundControl** | Mismatched IP or UDP port blocked by firewall | Verify laptop IP with `ipconfig` / `ifconfig`. Confirm Windows Defender allows UDP traffic on port 5000. |

---

<div align="center">

| [⬅️ Previous: **10 - Video Streaming**](gstreamer-streaming.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [➡️ Next: **12 - Known Issues & Limitations**](known-issues.md) |
| :--- | :---: | ---: |

</div>
