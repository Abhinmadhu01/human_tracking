#!/usr/bin/env python3
"""
GStreamer video streamer with detection bounding box overlay.
Subscribes to /camera/image_raw and /detections, draws bounding boxes
on each frame, and streams the annotated video via UDP/H.264 to a laptop.

"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import threading


class GStreamerStreamer(Node):
    def __init__(self):
        super().__init__('gst_streamer')

        # Set your laptop IP via GSTREAM_IP env variable or ROS param:
        #   export GSTREAM_IP=192.168.x.x
        #   or: ros2 run ... --ros-args -p laptop_ip:=192.168.x.x
        default_ip = os.environ.get('GSTREAM_IP', '127.0.0.1')
        self.declare_parameter('laptop_ip', default_ip)
        self.declare_parameter('port', 5000)

        self.laptop_ip = str(self.get_parameter('laptop_ip').value)
        self.port = int(self.get_parameter('port').value)
        self.image_topic = '/camera/image_raw'
        self.det_topic = '/detections'

        # SENDER PIPELINE (H.264 software encoding, ultra-low latency)
        self.gst_str = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.laptop_ip} port={self.port} sync=false"
        )

        self.bridge = CvBridge()
        self.frame_size = (640, 480)

        # Initialize Video Writer
        self.out = cv2.VideoWriter(
            self.gst_str,
            cv2.CAP_GSTREAMER,
            0,
            30,
            self.frame_size,
            True
        )

        if not self.out.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline! Check your OpenCV build.")
        else:
            self.get_logger().info(f"Streaming video to {self.laptop_ip}:{self.port}")

        # ── Detection state (thread-safe) ──
        self._det_lock = threading.Lock()
        self._latest_detections = None

        # ── Subscribers ──
        # IMPORTANT: Both topics use sensor_data QoS (BEST_EFFORT)
        # to match the publisher in camera_ai_start.py
        self.det_subscription = self.create_subscription(
            Detection2DArray,
            self.det_topic,
            self._det_callback,
            qos_profile_sensor_data
        )

        self.img_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("Subscribed to /camera/image_raw and /detections")

    def _det_callback(self, msg):
        """Cache the latest detections for the next frame draw."""
        with self._det_lock:
            self._latest_detections = msg

    def _image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            if (cv_image.shape[1], cv_image.shape[0]) != self.frame_size:
                cv_image = cv2.resize(cv_image, self.frame_size)

            # ── Draw bounding boxes from cached detections ──
            with self._det_lock:
                detections = self._latest_detections

            if detections is not None:
                for det in detections.detections:
                    cx = det.bbox.center.position.x
                    cy = det.bbox.center.position.y
                    w = det.bbox.size_x
                    h = det.bbox.size_y

                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)

                    # Green bounding box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Confidence label
                    if det.results:
                        score = det.results[0].hypothesis.score
                        label = f"Person {score:.0%}"
                        # Label background for readability
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(cv_image, (x1, max(y1 - th - 8, 0)),
                                      (x1 + tw + 4, max(y1, th + 8)), (0, 255, 0), -1)
                        cv2.putText(cv_image, label, (x1 + 2, max(y1 - 4, th + 4)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            # Write annotated frame to GStreamer pipeline
            self.out.write(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def destroy_node(self):
        if self.out.isOpened():
            self.out.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
