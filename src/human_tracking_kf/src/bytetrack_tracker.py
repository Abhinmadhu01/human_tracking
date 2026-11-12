#!/usr/bin/env python3
"""
ByteTrack Multi-Object Tracker Node
Manages multiple object tracks with ID assignment using ByteTrack algorithm.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from dataclasses import dataclass
from typing import List, Dict, Optional
from types import SimpleNamespace

# Safe imports for PyTorch and Ultralytics
try:
    import torch
    from ultralytics.trackers.byte_tracker import BYTETracker
    from ultralytics.engine.results import Boxes
except Exception:
    torch = None
    BYTETracker = None
    Boxes = None

try:
    from ultralytics.trackers.byte_tracker import ByteTrack as ByteTrackWrapper
except Exception:
    ByteTrackWrapper = None


@dataclass
class Track:
    """Represents a tracked object"""
    track_id: int
    bbox: tuple  # (cx, cy, width, height)
    confidence: float
    class_name: str
    age: int = 0
    last_seen: float = 0.0


class _DetectionResults:
    """Lightweight Results-like wrapper for numpy detection arrays.

    Ultralytics 8.4+ BYTETracker._split_detections() expects an object with:
      .conf  — 1-D array of confidence scores  (supports boolean masking)
      .cls   — 1-D array of class IDs
      .xyxy  — (N,4) array of bounding boxes
      __getitem__(mask)  — boolean / integer indexing
      __len__()

    Input ``dets`` shape: (N, 6) with columns [x1, y1, x2, y2, conf, cls].
    """

    def __init__(self, dets: np.ndarray):
        if dets.ndim != 2 or dets.shape[1] < 5:
            dets = np.empty((0, 6), dtype=np.float32)
        # Ensure we have 6 columns (cls column may be missing)
        if dets.shape[1] == 5:
            dets = np.hstack([dets, np.zeros((len(dets), 1), dtype=np.float32)])
        self._dets = dets.astype(np.float32)

    # --- attributes expected by BYTETracker ---
    @property
    def conf(self):
        return self._dets[:, 4]

    @property
    def cls(self):
        return self._dets[:, 5]

    @property
    def xyxy(self):
        return self._dets[:, :4]

    @property
    def xywh(self):
        """Return boxes in [cx, cy, w, h] format (used by init_track / parse_bboxes)."""
        boxes = self._dets[:, :4].copy()
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]
        cx = boxes[:, 0] + w / 2
        cy = boxes[:, 1] + h / 2
        return np.column_stack([cx, cy, w, h])

    def __len__(self):
        return len(self._dets)

    def __getitem__(self, idx):
        """Support boolean / integer indexing — return a new _DetectionResults."""
        return _DetectionResults(self._dets[idx])


class ByteTrackNode(Node):
    def __init__(self):
        super().__init__('bytetrack_tracker')
        
        # Parameters
        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/tracked_detections')
        self.declare_parameter('track_thresh', 0.5)
        self.declare_parameter('track_buffer', 90)
        self.declare_parameter('match_thresh', 0.7)
        self.declare_parameter('min_track_age', 3)
        self.declare_parameter('max_track_age', 30)
        self.declare_parameter('frame_rate', 20.0)
        self.declare_parameter('image_width', 0)
        self.declare_parameter('image_height', 0)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '/bytetrack_debug')
        
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.track_thresh = float(self.get_parameter('track_thresh').value)
        self.track_buffer = int(self.get_parameter('track_buffer').value)
        self.match_thresh = float(self.get_parameter('match_thresh').value)
        self.min_track_age = int(self.get_parameter('min_track_age').value)
        self.max_track_age = int(self.get_parameter('max_track_age').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.debug_topic = str(self.get_parameter('debug_image_topic').value)
        
        # Initialize ByteTrack
        try:
            self.tracker = self._init_bytetrack()
            if self.tracker is None:
                self.get_logger().warn("ByteTrack not available, using simple ID assignment fallback")
            else:
                self.get_logger().info("ByteTrack initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ByteTrack: {e}, using fallback")
            self.tracker = None

        
        # Track management
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1
        self._bt_update_warned = False
        
        # ROS 2 Setup
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.detection_sub = self.create_subscription(
            Detection2DArray, 
            self.input_topic,
            self.detection_callback, 
            qos
        )
        self.tracked_pub = self.create_publisher(Detection2DArray, self.output_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        
        self.get_logger().info("ByteTrack node ready")

    def _init_bytetrack(self):
        if BYTETracker is not None:
            # Ultralytics >=8.4 uses these arg names
            args = SimpleNamespace(
                track_high_thresh=self.track_thresh,
                track_low_thresh=max(0.1, self.track_thresh - 0.2),
                new_track_thresh=max(0.1, self.track_thresh - 0.1),
                track_buffer=self.track_buffer,
                match_thresh=self.match_thresh,
                fuse_score=True,
                # Legacy args for older Ultralytics versions
                track_thresh=self.track_thresh,
                mot20=False,
            )
            try:
                return BYTETracker(args, frame_rate=self.frame_rate)
            except TypeError:
                try:
                    return BYTETracker(args)
                except Exception:
                    pass

        if ByteTrackWrapper is not None:
            self.get_logger().warn(
                "Ultralytics ByteTrack wrapper expects Results objects; using IoU fallback instead."
            )
        return None
    
    @staticmethod
    def _stamp_to_sec(stamp) -> Optional[float]:
        if stamp is None:
            return None
        if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
            sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            return sec if sec > 0.0 else None
        return None

    def _get_time_sec(self, header) -> float:
        stamp_sec = self._stamp_to_sec(getattr(header, 'stamp', None))
        if stamp_sec is not None:
            return stamp_sec
        return self.get_clock().now().nanoseconds * 1e-9

    def _get_img_info(self):
        if self.image_width > 0 and self.image_height > 0:
            return (self.image_height, self.image_width)
        return None

    def _update_bytetrack(self, dets_xyxy: np.ndarray):
        """Bridges numpy detections into Ultralytics BYTETracker.
        
        Ultralytics 8.4+ expects a Results-like object with:
          .conf  - confidence scores (supports boolean indexing)
          .cls   - class IDs
          .xyxy  - bounding boxes in [x1,y1,x2,y2] format
          __getitem__ and __len__ for slicing
        
        Returns a numpy array with rows: [x1, y1, x2, y2, track_id, conf, cls, idx]
        
        IMPORTANT: We always call tracker.update(), even with empty detections,
        so ByteTrack can properly age and manage its internal track state.
        """
        if self.tracker is None:
            return None

        if dets_xyxy is None:
            dets_xyxy = np.empty((0, 6), dtype=np.float32)

        if dets_xyxy.ndim != 2:
            if dets_xyxy.size == 0:
                dets_xyxy = np.empty((0, 6), dtype=np.float32)
            else:
                dets_xyxy = np.atleast_2d(dets_xyxy)

        try:
            # Always pass to tracker — even empty frames need to advance internal state
            results = _DetectionResults(dets_xyxy)
            return self.tracker.update(results)

        except Exception as e:
            if not self._bt_update_warned:
                self.get_logger().warn(f"ByteTrack native update failed: {e}. Check Ultralytics version.")
                self._bt_update_warned = True
            return None

    @staticmethod
    def _compute_iou(bbox1, bbox2):
        """Compute IoU (Intersection over Union) between two bboxes
        bbox format: (cx, cy, w, h)
        """
        cx1, cy1, w1, h1 = bbox1
        cx2, cy2, w2, h2 = bbox2
        
        x1_min, y1_min = cx1 - w1/2, cy1 - h1/2
        x1_max, y1_max = cx1 + w1/2, cy1 + h1/2
        
        x2_min, y2_min = cx2 - w2/2, cy2 - h2/2
        x2_max, y2_max = cx2 + w2/2, cy2 + h2/2
        
        intersect_w = max(0, min(x1_max, x2_max) - max(x1_min, x2_min))
        intersect_h = max(0, min(y1_max, y2_max) - max(y1_min, y2_min))
        intersect_area = intersect_w * intersect_h
        
        union_area = w1 * h1 + w2 * h2 - intersect_area
        
        if union_area == 0:
            return 0.0
        return intersect_area / union_area
    
    def _match_detections_to_tracks(self, detections: List[dict]) -> Dict[int, dict]:
        """Simple fallback: match new detections to existing tracks by IoU"""
        matched = {}
        unmatched_dets = list(range(len(detections)))
        
        # Try to match active tracks
        for track_id, track in list(self.tracks.items()):
            best_iou = 0.0
            best_det_idx = -1
            
            for det_idx in unmatched_dets:
                det = detections[det_idx]
                iou = self._compute_iou(track.bbox, det['bbox'])
                
                if iou > best_iou and iou > self.match_thresh:
                    best_iou = iou
                    best_det_idx = det_idx
            
            if best_det_idx >= 0:
                matched[track_id] = detections[best_det_idx]
                unmatched_dets.remove(best_det_idx)
        
        # Assign new IDs to unmatched detections
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            if det.get('confidence', 0.5) >= self.track_thresh:
                new_id = self.next_track_id
                self.next_track_id += 1
                matched[new_id] = det
        
        return matched
    
    def detection_callback(self, msg: Detection2DArray):
        """Process detections and update tracks"""
        try:
            now = self._get_time_sec(msg.header)
            
            # Extract detection info
            detections = []
            for det in msg.detections:
                if not det.results:
                    continue
                
                cx = float(det.bbox.center.position.x)
                cy = float(det.bbox.center.position.y)
                w = float(det.bbox.size_x)
                h = float(det.bbox.size_y)
                conf = float(det.results[0].hypothesis.score) if det.results else 0.5
                class_name = str(det.results[0].hypothesis.class_id) if det.results else 'unknown'
                
                detections.append({
                    'bbox': (cx, cy, w, h),
                    'confidence': conf,
                    'class_name': class_name,
                    'original_det': det
                })
            
            # Update tracks using ByteTrack or fallback
            if self.tracker is not None and hasattr(self.tracker, 'update'):
                # Convert to format expected by ByteTrack: [x1, y1, x2, y2, conf, class_id]
                dets_xyxy = []
                for d in detections:
                    cx, cy, w, h = d['bbox']
                    x1 = cx - w/2
                    y1 = cy - h/2
                    x2 = cx + w/2
                    y2 = cy + h/2
                    dets_xyxy.append([x1, y1, x2, y2, d['confidence'], 0])
                
                if len(dets_xyxy) == 0:
                    dets_xyxy = np.empty((0, 6), dtype=np.float32)
                else:
                    dets_xyxy = np.array(dets_xyxy, dtype=np.float32)
                    
                online_targets = self._update_bytetrack(dets_xyxy)

                tracked_dict = {}
                if online_targets is not None and len(online_targets) > 0:
                    # Process tracked objects safely (handles both Object-based and Array-based returns)
                    for target in online_targets:
                        if isinstance(target, np.ndarray) or isinstance(target, list) or (hasattr(target, 'shape') and len(target) >= 5):
                            # Array API
                            x1, y1, x2, y2 = target[0:4]
                            track_id = int(target[4])
                            score = float(target[5]) if len(target) > 5 else 0.5
                        else:
                            # Object API
                            track_id = int(target.track_id)
                            x1, y1, x2, y2 = target.tlbr
                            score = float(target.score) if hasattr(target, 'score') else 0.5

                        cx = (x1 + x2) / 2
                        cy = (y1 + y2) / 2
                        w = x2 - x1
                        h = y2 - y1

                        # Find original detection for this track
                        orig_det = None
                        for d in detections:
                            iou = self._compute_iou(d['bbox'], (cx, cy, w, h))
                            if iou > 0.5:
                                orig_det = d
                                break

                        tracked_dict[track_id] = {
                            'bbox': (cx, cy, w, h),
                            'confidence': score,
                            'class_name': orig_det['class_name'] if orig_det else 'person',
                            'original_det': orig_det['original_det'] if orig_det else None
                        }

                if not tracked_dict and detections:
                    matched = self._match_detections_to_tracks(detections)
                    for track_id, det in matched.items():
                        tracked_dict[track_id] = {
                            'bbox': det['bbox'],
                            'confidence': det['confidence'],
                            'class_name': det['class_name'],
                            'original_det': det.get('original_det')
                        }

                # Update tracks — clear immediately when no tracked output
                self.tracks = {
                    tid: Track(
                        track_id=tid,
                        bbox=d['bbox'],
                        confidence=d['confidence'],
                        class_name=d['class_name'],
                        age=self.tracks[tid].age + 1 if tid in self.tracks else 1,
                        last_seen=now
                    )
                    for tid, d in tracked_dict.items()
                }
            else:
                # Fallback: simple IoU-based matching
                matched = self._match_detections_to_tracks(detections)
                
                new_tracks = {}
                for track_id, det in matched.items():
                    age = self.tracks[track_id].age + 1 if track_id in self.tracks else 1
                    new_tracks[track_id] = Track(
                        track_id=track_id,
                        bbox=det['bbox'],
                        confidence=det['confidence'],
                        class_name=det['class_name'],
                        age=age,
                        last_seen=now
                    )
                
                self.tracks = new_tracks
            
            # Prune old tracks
            frame_rate = self.frame_rate if self.frame_rate > 0.0 else 20.0
            max_age_sec = self.max_track_age / frame_rate
            self.tracks = {
                tid: t for tid, t in self.tracks.items()
                if (now - t.last_seen) < max_age_sec
            }
            
            # Publish tracked detections
            tracked_array = Detection2DArray()
            tracked_array.header = msg.header
            
            for track_id, track in self.tracks.items():
                if track.age >= self.min_track_age:
                    det = Detection2D()
                    det.header = msg.header
                    
                    cx, cy, w, h = track.bbox
                    det.bbox.center.position.x = float(cx)
                    det.bbox.center.position.y = float(cy)
                    det.bbox.size_x = float(w)
                    det.bbox.size_y = float(h)
                    
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = f"{track.class_name}_{track.track_id}"
                    hyp.hypothesis.score = float(track.confidence)
                    det.results.append(hyp)
                    
                    tracked_array.detections.append(det)
            
            if rclpy.ok():
                self.tracked_pub.publish(tracked_array)
            
        except Exception as e:
            self.get_logger().error(f"Detection callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ByteTrackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()