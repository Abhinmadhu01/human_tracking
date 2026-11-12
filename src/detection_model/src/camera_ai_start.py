#!/usr/bin/env python3
"""
ROS2 Camera Publisher for Pi 5 + IMX500
Fixed: Anti-flicker detection caching + lightweight pre_callback

Key fix: When imx500.get_outputs() returns None (inference not ready for this
frame), we CACHE and REUSE the last valid detections instead of publishing
empty — this is what the official Raspberry Pi IMX500 demo does.
"""

import collections
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # ── ROS2 Parameters (set via YAML or command line) ────────────────
        self.declare_parameter('resolution', '640x480')
        self.declare_parameter('fps', 30)
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('model_path',
            '/home/ros/top_human_detection/src/detection_model/model/'
            'imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk')

        resolution_str = str(self.get_parameter('resolution').value)
        self.resolution = tuple(map(int, resolution_str.split('x')))
        self.fps = int(self.get_parameter('fps').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.model_path = str(self.get_parameter('model_path').value)

        self.image_pub     = self.create_publisher(
            Image, 'camera/image_raw', qos_profile_sensor_data)
        self.detection_pub = self.create_publisher(
            Detection2DArray, 'detections', qos_profile_sensor_data)
        self.bridge = CvBridge()

        # ── Inter-thread data passing ──────────────────────────────────────
        self._frame_deque     = collections.deque(maxlen=4)
        self._deque_cond      = threading.Condition()

        self._img_lock        = threading.Lock()
        self._latest_img      = None
        self._img_event       = threading.Event()

        # ── Cached last valid detection (anti-flicker — official demo pattern) ──
        self._last_valid_det_array = None

        # ── Stats ─────────────────────────────────────────────────────────
        self.running          = True
        self._stat_lock       = threading.Lock()
        self._cap_count       = 0
        self._det_pub_count   = 0
        self._img_count       = 0
        self._stat_t0         = time.time()
        self._last_cap_time   = time.time()
        self._restarting      = False

        # ── Camera init ───────────────────────────────────────────────────
        self._init_camera()

        # ── Threads ───────────────────────────────────────────────────────
        self._process_thread  = threading.Thread(
            target=self._process_loop, name='process', daemon=True)
        self._image_thread    = threading.Thread(
            target=self._image_publish_loop, name='img_pub', daemon=True)

        self._process_thread.start()
        self._image_thread.start()

        self.create_timer(5.0, self._log_stats)
        self.add_on_set_parameters_callback(self._parameters_callback)
        self.get_logger().info(
            f'IMX500 Publisher ready — {self.resolution[0]}x{self.resolution[1]} @ {self.fps} fps | '
            f'conf={self.conf_threshold} nms={self.nms_threshold} | model={os.path.basename(self.model_path)}')

    # =========================================================
    # CAMERA INIT
    # =========================================================
    def _parameters_callback(self, params):
        """Live-tune conf/nms thresholds without restart."""
        for param in params:
            if param.name == 'conf_threshold':
                self.conf_threshold = float(param.value)
                self.get_logger().info(f'Live Tuning: conf_threshold = {param.value}')
            elif param.name == 'nms_threshold':
                self.nms_threshold = float(param.value)
                self.get_logger().info(f'Live Tuning: nms_threshold = {param.value}')
        return SetParametersResult(successful=True)

    def _init_camera(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'RPK missing: {self.model_path}')
            sys.exit(1)

        self.get_logger().info(f'Loading IMX500 firmware: {os.path.basename(self.model_path)}')
        self.imx500  = IMX500(self.model_path)
        self.picam2  = Picamera2(self.imx500.camera_num)

        self.imx500.show_network_fw_progress_bar()

        config = self.picam2.create_preview_configuration(
            main={'format': 'BGR888', 'size': self.resolution},
            controls={'FrameRate': self.fps},
            buffer_count=12,
        )
        self.picam2.configure(config)

        # pre_callback runs in the C++ libcamera thread
        self.picam2.pre_callback = self._on_frame

        self.picam2.start()
        self.imx500.set_auto_aspect_ratio()
        time.sleep(0.3)
        self.get_logger().info('Camera started successfully.')

    # =========================================================
    # HARDWARE CALLBACK (C++ libcamera thread — keep lightweight!)
    # =========================================================
    def _on_frame(self, request):
        if not self.running or self._restarting:
            return

        try:
            self._last_cap_time = time.time()

            # 1. Grab metadata + AI outputs (fast — just reads from HW buffer)
            metadata = request.get_metadata()
            np_outputs = self.imx500.get_outputs(metadata, add_batch=True)

            # 2. Grab frame (must copy before callback returns)
            raw_frame = request.make_array("main")
            frame = np.copy(raw_frame)
            del raw_frame
            del metadata

            stamp = self.get_clock().now().to_msg()

            # 3. Hand off frame for image publishing
            with self._img_lock:
                self._latest_img = (frame, stamp)
            self._img_event.set()

            # 4. Hand off detections for processing
            #    np_outputs can be None when inference hasn't completed — that's normal
            with self._deque_cond:
                self._frame_deque.append((np_outputs, stamp))
                self._deque_cond.notify()

            with self._stat_lock:
                self._cap_count += 1

        except Exception as e:
            # Log errors instead of silently swallowing them
            try:
                self.get_logger().debug(f'Frame callback error: {e}')
            except Exception:
                pass

    # =========================================================
    # THREAD 1 — PROCESS + PUBLISH DETECTIONS
    # =========================================================
    def _process_loop(self):
        while self.running and rclpy.ok():

            with self._deque_cond:
                while not self._frame_deque and self.running:
                    self._deque_cond.wait(timeout=1.0)
                if not self._frame_deque:
                    continue
                np_outputs, stamp = self._frame_deque.popleft()

            try:
                if np_outputs is not None:
                    # New inference result available — build and cache it
                    det_array = self._build_detections(np_outputs, stamp)
                    self._last_valid_det_array = det_array
                else:
                    # Inference not ready for this frame — reuse cached result
                    # (this is the official anti-flicker pattern)
                    if self._last_valid_det_array is not None:
                        det_array = self._last_valid_det_array
                        # Update the timestamp so downstream doesn't think it's stale
                        det_array.header.stamp = stamp
                    else:
                        # No cached result yet (startup) — publish empty
                        det_array = Detection2DArray()
                        det_array.header.stamp = stamp
                        det_array.header.frame_id = 'camera_imx500'

                self.detection_pub.publish(det_array)
                with self._stat_lock:
                    self._det_pub_count += 1

            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Process error: {e}')

    # =========================================================
    # THREAD 2 — IMAGE PUBLISHING
    # =========================================================
    def _image_publish_loop(self):
        while self.running and rclpy.ok():

            triggered = self._img_event.wait(timeout=1.0)
            if not triggered or not self.running:
                continue

            with self._img_lock:
                data = self._latest_img
                self._latest_img = None
            self._img_event.clear()

            if data is None:
                continue

            frame, stamp = data
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = stamp
                msg.header.frame_id = 'camera_imx500'
                self.image_pub.publish(msg)

                with self._stat_lock:
                    self._img_count += 1

            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Image publish error: {e}')

    # =========================================================
    # DETECTION BUILDER
    # =========================================================
    def _build_detections(self, np_outputs, stamp) -> Detection2DArray:
        det_array = Detection2DArray()
        det_array.header.stamp = stamp
        det_array.header.frame_id = 'camera_imx500'

        if np_outputs is None or len(np_outputs) < 3:
            return det_array

        # With add_batch=True, outputs have an extra batch dimension
        boxes = np_outputs[0][0] if np_outputs[0].ndim > 2 else np_outputs[0]
        scores = np_outputs[1][0] if np_outputs[1].ndim > 1 else np_outputs[1]
        classes = np_outputs[2][0] if np_outputs[2].ndim > 1 else np_outputs[2]

        W, H = self.resolution

        valid_boxes = []
        valid_scores = []
        valid_classes = []

        for i in range(len(scores)):
            conf = float(scores[i])
            class_id = int(classes[i])

            if class_id != 0 or conf < self.conf_threshold:
                continue

            y_min, x_min, y_max, x_max = (float(v) for v in boxes[i])

            if x_max <= 1.0 and y_max <= 1.0:
                px_min, py_min = int(x_min * W), int(y_min * H)
                px_max, py_max = int(x_max * W), int(y_max * H)
            else:
                px_min, py_min = int(x_min), int(y_min)
                px_max, py_max = int(x_max), int(y_max)

            w, h = px_max - px_min, py_max - py_min
            if w <= 0 or h <= 0:
                continue

            valid_boxes.append([int(px_min), int(py_min), int(w), int(h)])
            valid_scores.append(conf)
            valid_classes.append(class_id)

        if not valid_boxes:
            return det_array

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(valid_boxes, valid_scores, self.conf_threshold, self.nms_threshold)

        if len(indices) > 0:
            for i in indices.flatten():
                box = valid_boxes[i]
                conf = valid_scores[i]

                px_min, py_min, w, h = box

                det = Detection2D()
                det.bbox.center.position.x = float(px_min + w / 2.0)
                det.bbox.center.position.y = float(py_min + h / 2.0)
                det.bbox.size_x = float(w)
                det.bbox.size_y = float(h)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'person'
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

        return det_array

    # =========================================================
    # STATS  (every 5 s)
    # =========================================================
    def _log_stats(self):
        with self._stat_lock:
            dt = time.time() - self._stat_t0
            if dt <= 0:
                return
            cap  = self._cap_count      / dt
            det  = self._det_pub_count  / dt
            img  = self._img_count      / dt

            self._cap_count = self._det_pub_count = self._img_count = 0
            self._stat_t0 = time.time()

        self.get_logger().info(
            f'capture {cap:.1f} Hz  |  '
            f'det_pub {det:.1f} Hz  |  '
            f'image {img:.1f} Hz')

    def stop(self):
        self.running = False
        with self._deque_cond:
            self._deque_cond.notify_all()
        self._img_event.set()
        if hasattr(self, 'picam2'):
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down…')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()