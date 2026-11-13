# GStreamer Live Video Streaming to QGroundControl

Guide for transmitting ultra-low-latency H.264 video with real-time detection bounding box overlays to a ground station laptop.

## Pipeline Architecture

The `gstream_human_detection.py` node subscribes to `/camera/image_raw` and `/detections`, overlays the bounding boxes and track IDs, and encodes the stream to H.264 using hardware-tuned parameters:

```text
Raspberry Pi 5 (Drone)
  [cv2.VideoWriter]
       |
  appsrc ! videoconvert ! format=I420 ! x264enc (tune=zerolatency, ultrafast)
       |
  rtph264pay ! udpsink (host=laptop_ip, port=5000)
       |
       v (UDP Network Stream)
Ground Station Laptop (QGroundControl on Port 5000)
```

## Running the Video Streamer

Run the streaming script specifying your ground station laptop IP:

```bash
python3 gstream_human_detection.py --ros-args -p laptop_ip:=192.168.1.100 -p port:=5000
```

Or set the environment variable:
```bash
export GSTREAM_IP=192.168.1.100
python3 gstream_human_detection.py
```

## Configuring QGroundControl (QGC)

1. Launch **QGroundControl** on your ground station laptop.
2. Click the **QGC Logo** (top-left) $
ightarrow$ **Application Settings**.
3. Select **General** from the left navigation.
4. Scroll to the **Video** settings section:
   * **Video Source**: Select `UDP h.264 Video Stream`.
   * **UDP Port**: Enter `5000`.
5. Return to the main flight screen: the live video feed with overlaid bounding boxes will automatically appear in the bottom-left PIP window or as the primary background.

---

<div align="center">

| [⬅️ Previous: **09 - Configuration**](software/config.md) | [📖 **Documentation Roadmap**](../README.md#documentation-roadmap) | [➡️ Next: **11 - Troubleshooting**](troubleshooting.md) |
| :--- | :---: | ---: |

</div>
