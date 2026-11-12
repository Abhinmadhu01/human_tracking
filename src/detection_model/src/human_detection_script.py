#!/usr/bin/env python3

import importlib.util
import os
from pathlib import Path
import threading
import time
import numpy as np
import cv2

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_detector')

        # Parameters
        self.declare_parameter('input_topic', '/camera/image')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('debug_topic', 'yolo_debug_image')
        self.declare_parameter('model_variant', 'yolo11m')
        self.declare_parameter('backend_device', 'auto')
        self.declare_parameter('custom_model_path', '')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('imgsz', 384)
        self.declare_parameter('conf', 0.2)
        self.declare_parameter('min_confidence', 0.2)
        self.declare_parameter('max_det', 20)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('publish_all_detections', True)
        self.declare_parameter('max_bbox_width_ratio', 0.98)
        self.declare_parameter('max_bbox_height_ratio', 0.98)
        self.declare_parameter('max_bbox_area_ratio', 0.9)
        self.declare_parameter('save_detections', False)
        self.declare_parameter('fp_save_dir', '')
        
        # Camera parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('image_center_y', 240.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.model_variant = str(self.get_parameter('model_variant').value).strip().lower()
        self.backend_device = self._resolve_backend_device(str(self.get_parameter('backend_device').value).strip())
        self.custom_model_path = str(self.get_parameter('custom_model_path').value).strip()
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf = float(self.get_parameter('conf').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.nms_iou_threshold = float(self.get_parameter('nms_iou_threshold').value)
        self.use_half_precision = self.backend_device != 'cpu' and self._parameter_truthy('use_half_precision', True)
        self.target_class = str(self.get_parameter('target_class').value)
        self.publish_all_detections = bool(self.get_parameter('publish_all_detections').value)
        self.max_bbox_width_ratio = float(self.get_parameter('max_bbox_width_ratio').value)
        self.max_bbox_height_ratio = float(self.get_parameter('max_bbox_height_ratio').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)
        self.save_detections = bool(self.get_parameter('save_detections').value)
        self.fp_save_dir = str(self.get_parameter('fp_save_dir').value).strip()
        
        # Camera center
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.cx = self.get_parameter('image_center_x').value
        self.cy = self.get_parameter('image_center_y').value

        if self.max_det < 1:
            self.max_det = 1

        self.model_path, self.selected_backend = self._resolve_model_path()
        if self.selected_backend == 'ncnn' and self.imgsz != 640:
            self.get_logger().warn(
                f"ncnn backend works best with imgsz=640; forcing imgsz=640 from {self.imgsz}"
            )
            self.imgsz = 640

        try:
            self.get_logger().info(f"Attempting to load YOLO model from path: {self.model_path}")
            self.model = YOLO(self.model_path, task='detect')
            self.get_logger().info(
                f"{self.selected_backend.upper()} model loaded successfully: {self.model_path} on device={self.backend_device}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model at {self.model_path}. Is the path correct?")
            raise RuntimeError(f'Load Error: {e}') from e

        # Prepare FP save dir if requested
        if self.save_detections:
            if not self.fp_save_dir:
                try:
                    pkg_share = get_package_share_directory('detection_model')
                    self.fp_save_dir = os.path.join(pkg_share, 'collected_false_positives')
                except Exception:
                    self.fp_save_dir = str(Path(__file__).resolve().parents[2] / 'collected_false_positives')
            try:
                os.makedirs(self.fp_save_dir, exist_ok=True)
            except Exception as ex:
                self.get_logger().error(f'Failed to create FP save dir {self.fp_save_dir}: {ex}')

        self.publisher = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.image_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self._frame_lock = threading.Lock()
        self._new_frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._is_destroyed = False
        self._pending_frame = None
        self._frames_in = 0
        self._frames_processed = 0
        self._frames_overwritten = 0
        self._latency_sum_ms = 0.0
        self._latency_count = 0

        self.worker_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.worker_thread.start()
        self.status_timer = self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            f"Ready: Image center=({self.cx}, {self.cy}), debug={self.publish_debug_image}"
        )

    def _parameter_truthy(self, name, default=False):
        value = self.get_parameter(name).value if self.has_parameter(name) else default
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}

    @staticmethod
    def _encoding_channels(encoding):
        encoding = str(encoding).lower()
        channel_map = {
            'mono8': 1,
            '8uc1': 1,
            'bgr8': 3,
            'rgb8': 3,
            'bgra8': 4,
            'rgba8': 4,
            '8uc3': 3,
        }
        return channel_map.get(encoding, 0)

    @staticmethod
    def _image_msg_to_bgr8(msg):
        encoding = str(msg.encoding).lower()
        channels = YoloTrackerNode._encoding_channels(encoding)
        if channels == 0:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        row_width = int(msg.width) * channels
        if msg.step < row_width:
            raise ValueError(
                f"Image step {msg.step} is smaller than expected row width {row_width} for encoding {msg.encoding}"
            )

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = buffer.reshape((int(msg.height), int(msg.step)))[:, :row_width]

        if channels == 1:
            mono = frame.reshape((int(msg.height), int(msg.width)))
            return np.repeat(mono[:, :, None], 3, axis=2)

        frame = frame.reshape((int(msg.height), int(msg.width), channels))

        if encoding == 'rgb8' or encoding == 'rgba8':
            frame = frame[:, :, :3][:, :, ::-1]
        elif encoding == 'bgra8':
            frame = frame[:, :, :3]

        return np.ascontiguousarray(frame)

    @staticmethod
    def _bgr8_to_image_msg(frame, header):
        image_msg = Image()
        image_msg.header = header
        image_msg.height = int(frame.shape[0])
        image_msg.width = int(frame.shape[1])
        image_msg.encoding = 'bgr8'
        image_msg.is_bigendian = 0
        image_msg.step = int(frame.shape[1]) * 3
        image_msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        return image_msg

    def _bbox_is_plausible(self, width, height):
        if self.img_width <= 0 or self.img_height <= 0:
            return True

        width_ratio = float(width) / float(self.img_width)
        height_ratio = float(height) / float(self.img_height)
        area_ratio = float(width) * float(height) / float(self.img_width * self.img_height)

        return (
            width_ratio <= self.max_bbox_width_ratio
            and height_ratio <= self.max_bbox_height_ratio
            and area_ratio <= self.max_bbox_area_ratio
        )

    def _append_detection(self, det_array, detection):
        det = Detection2D()
        det.bbox.center.position.x = float((detection['x1'] + detection['x2']) / 2.0)
        det.bbox.center.position.y = float((detection['y1'] + detection['y2']) / 2.0)
        det.bbox.size_x = float(detection['x2'] - detection['x1'])
        det.bbox.size_y = float(detection['y2'] - detection['y1'])

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.target_class
        hyp.hypothesis.score = float(detection['confidence'])
        det.results.append(hyp)
        det_array.detections.append(det)

    @staticmethod
    def _resolve_backend_device(requested_device):
        if requested_device and requested_device != 'auto':
            return requested_device

        if importlib.util.find_spec('torch') is not None:
            import torch

            if torch.cuda.is_available():
                return '0'

        return 'cpu'

    @staticmethod
    def _module_available(module_name):
        return importlib.util.find_spec(module_name) is not None

    @staticmethod
    def _first_existing_path(candidates):
        for candidate in candidates:
            if candidate and os.path.exists(candidate):
                return candidate
        return ''

    @staticmethod
    def _first_existing_or_default(candidates):
        existing = YoloTrackerNode._first_existing_path(candidates)
        return existing if existing else candidates[0]

    @staticmethod
    def _safe_package_dir(package_name):
        """Safely get package directory, returning empty string if package not found"""
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return ''

    def _resolve_model_path(self):
        if self.custom_model_path:
            return self.custom_model_path, 'custom'

        package_share = get_package_share_directory('detection_model')
        package_root = Path(__file__).resolve().parents[1]
        workspace_root = Path(__file__).resolve().parents[3]
        model_interface_dir = self._safe_package_dir('model_interface_publisher')
        
        # Build candidate lists with fallback to workspace paths
        onnx_candidates = []
        if model_interface_dir:
            onnx_candidates.append(os.path.join(model_interface_dir, 'model', 'top_model_best.onnx'))
        onnx_candidates.append(str(workspace_root / 'src' / 'model_interface_publisher' / 'model' / 'top_model_best.onnx'))

        yolo11n_candidates = ['yolo11n.pt']
        if model_interface_dir:
            yolo11n_candidates.append(os.path.join(model_interface_dir, 'model', 'yolo11n.pt'))
        yolo11n_candidates.append(str(workspace_root / 'src' / 'model_interface_publisher' / 'model' / 'yolo11n.pt'))

        yolo11m_candidates = [
            os.path.join(package_share, 'model', 'yolov11m_gazebo_human_model.pt'),
            str(package_root / 'model' / 'yolov11m_gazebo_human_model.pt'),
        ]

        pt_candidates = []
        if model_interface_dir:
            pt_candidates.append(os.path.join(model_interface_dir, 'model', 'top_model_best.pt'))
        pt_candidates.append(str(workspace_root / 'src' / 'model_interface_publisher' / 'model' / 'top_model_best.pt'))

        model_candidates = {
            'yolo11m': ('torch', yolo11m_candidates),
            'yolo11n': ('torch', yolo11n_candidates),
            'pt': ('torch', pt_candidates),
            'onnx': ('onnxruntime', onnx_candidates),
            'openvino': ('openvino', [
                os.path.join(package_share, 'model', 'yolo26n_openvino_model'),
                str(package_root / 'model' / 'yolo26n_openvino_model'),
            ]),
            'ncnn': ('ncnn', [
                os.path.join(package_share, 'model', 'yolo26n_ncnn_model'),
                str(package_root / 'model' / 'yolo26n_ncnn_model'),
            ]),
        }

        variant = self.model_variant.strip().lower()
        if variant == 'auto':
            search_order = ['yolo11m', 'yolo11n', 'pt', 'onnx', 'openvino', 'ncnn']
        else:
            search_order = [variant]

        for backend in search_order:
            if backend not in model_candidates:
                raise ValueError(
                    f"Unsupported model_variant='{self.model_variant}'. Use one of: auto, yolo11m, yolo11n, pt, onnx, openvino, ncnn"
                )

            module_name, candidates = model_candidates[backend]
            if not self._module_available(module_name):
                continue

            if backend == 'yolo11n':
                model_path = self._first_existing_or_default(candidates)
            else:
                model_path = self._first_existing_path(candidates)
            if model_path:
                return model_path, backend

        raise RuntimeError(
            "No supported model backend is available. "
            "Install one of: torch, onnxruntime, openvino, or ncnn, or set custom_model_path to a valid export. "
            "The default auto mode prefers the packaged yolov11m_gazebo_human_model.pt first."
        )

    def image_callback(self, msg):
        try:
            cv_image = self._image_msg_to_bgr8(msg)
            with self._frame_lock:
                if self._pending_frame is not None:
                    self._frames_overwritten += 1
                self._pending_frame = (msg.header, cv_image)
                self._frames_in += 1
                self._new_frame_event.set()
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def inference_worker(self):
        while not self._stop_event.is_set():
            self._new_frame_event.wait(timeout=0.1)
            self._new_frame_event.clear()

            if self._stop_event.is_set():
                break

            with self._frame_lock:
                item = self._pending_frame
                self._pending_frame = None

            if item is None:
                continue

            header, cv_image = item

            try:
                start_t = time.perf_counter()
                results = self.model.predict(
                    source=cv_image,
                    classes=[0],  # person class
                    imgsz=self.imgsz,
                    conf=self.conf,
                    max_det=self.max_det,
                    device=self.backend_device,
                    half=self.use_half_precision,
                    iou=self.nms_iou_threshold,
                    verbose=False,
                )
                latency_ms = (time.perf_counter() - start_t) * 1000.0
                self._latency_sum_ms += latency_ms
                self._latency_count += 1

                # Process detections
                detections = []
                best_detection = None
                best_confidence = 0.0
                
                for r in results:
                    if self._stop_event.is_set():
                        break

                    # Find best person detection
                    for box in r.boxes:
                        # Safely convert tensors that may live on CUDA to CPU/Python types
                        # Confidence
                        try:
                            conf_val = box.conf[0]
                            if hasattr(conf_val, 'cpu'):
                                confidence = float(conf_val.cpu().item())
                            else:
                                confidence = float(conf_val)
                        except Exception:
                            try:
                                confidence = float(np.asarray(box.conf[0]).item())
                            except Exception:
                                continue

                        if confidence < self.min_confidence:
                            continue

                        # bbox xyxy
                        try:
                            xy = box.xyxy[0]
                            if hasattr(xy, 'cpu'):
                                xy = xy.cpu().numpy().tolist()
                            else:
                                xy = xy.tolist()
                        except Exception:
                            try:
                                xy = list(np.asarray(box.xyxy[0]).tolist())
                            except Exception:
                                continue

                        if len(xy) < 4:
                            continue

                        x1, y1, x2, y2 = map(float, xy[:4])
                        width = float(x2 - x1)
                        height = float(y2 - y1)
                        if not self._bbox_is_plausible(width, height):
                            continue

                        # class id
                        class_id = 0
                        try:
                            if hasattr(box, 'cls'):
                                cls_val = box.cls[0]
                                if hasattr(cls_val, 'cpu'):
                                    class_id = int(cls_val.cpu().item())
                                else:
                                    class_id = int(np.asarray(cls_val).item())
                        except Exception:
                            class_id = 0

                        detection = {
                            'x1': float(x1),
                            'y1': float(y1),
                            'x2': float(x2),
                            'y2': float(y2),
                            'confidence': float(confidence),
                            'class_id': int(class_id)
                        }
                        detections.append(detection)

                        if confidence > best_confidence:
                            best_confidence = float(confidence)
                            best_detection = detection

                # Build Detection2DArray message
                det_array = Detection2DArray()
                det_array.header = header

                if self.publish_all_detections:
                    for detection in detections:
                        self._append_detection(det_array, detection)
                elif best_detection:
                    self._append_detection(det_array, best_detection)

                # Publish debug image
                if self.publish_debug_image and len(results) > 0:
                    annotated_frame = results[0].plot()
                    # Optionally save detection frames for later manual review / retraining
                    if self.save_detections and det_array.detections:
                        try:
                            ts = int(time.time() * 1000)
                            conf_str = f"{best_confidence:.2f}" if best_confidence else "na"
                            fname = f"det_{ts}_{conf_str}.jpg"
                            fpath = os.path.join(self.fp_save_dir, fname)
                            cv2.imwrite(fpath, annotated_frame)
                        except Exception as ex:
                            self.get_logger().error(f'Failed to save detection image: {ex}')

                    img_msg = self._bgr8_to_image_msg(annotated_frame, header)
                    try:
                        self.image_pub.publish(img_msg)
                    except Exception:
                        if self._stop_event.is_set():
                            break
                        raise

                # Publish detections
                if self._stop_event.is_set():
                    continue

                try:
                    self.publisher.publish(det_array)
                except Exception:
                    if self._stop_event.is_set():
                        continue
                    raise

                self._frames_processed += 1

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')

    def log_status(self):
        avg_latency = (self._latency_sum_ms / self._latency_count) if self._latency_count else 0.0
        status_msg = (
            f'in={self._frames_in} processed={self._frames_processed} '
            f'overwritten={self._frames_overwritten} avg_infer={avg_latency:.1f}ms'
        )
        self.get_logger().info(status_msg)

    def destroy_node(self):
        if self._is_destroyed:
            return True
        self._is_destroyed = True

        self._stop_event.set()
        self._new_frame_event.set()

        try:
            if hasattr(self, 'subscription') and self.subscription is not None:
                self.destroy_subscription(self.subscription)
        except Exception:
            pass

        try:
            if hasattr(self, 'status_timer') and self.status_timer is not None:
                self.destroy_timer(self.status_timer)
        except Exception:
            pass

        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('yolo_tracker_detector').error(str(e))
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()