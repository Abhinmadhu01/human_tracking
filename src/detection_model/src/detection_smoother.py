#!/usr/bin/env python3
"""
DetectionSmoother – ROS2 node

* Subscribes to raw IMX500 detections on '/detections'.
* Performs a lightweight temporal‑consistency filter.
* Publishes a smoothed detection array on '/detections_smoothed'.

The algorithm is deliberately cheap (≈ 5 % CPU on a Pi 5) and works well
for human‑tracking pipelines where occasional frames miss a detection.
"""

import collections
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


def iou(box_a: Tuple[float, float, float, float],
        box_b: Tuple[float, float, float, float]) -> float:
    """Intersection‑over‑Union for two boxes (cx, cy, w, h)."""
    xa, ya, wa, ha = box_a
    xb, yb, wb, hb = box_b

    # Convert to corner coordinates
    ax1, ay1 = xa - wa / 2.0, ya - ha / 2.0
    ax2, ay2 = xa + wa / 2.0, ya + ha / 2.0
    bx1, by1 = xb - wb / 2.0, yb - hb / 2.0
    bx2, by2 = xb + wb / 2.0, yb + hb / 2.0

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    area_a = wa * ha
    area_b = wb * hb
    union = area_a + area_b - inter_area
    return 0.0 if union == 0 else inter_area / union


class SmoothedTrack:
    """Tracks a single detection across several frames."""

    def __init__(self, det: Detection2D, ts: float):
        self.history: List[Tuple[Detection2D, float]] = [(det, ts)]

    def add(self, det: Detection2D, ts: float):
        self.history.append((det, ts))

    def recent(self, max_age: float) -> List[Detection2D]:
        now = time.time()
        return [d for d, t in self.history if now - t <= max_age]

    def avg_bbox(self) -> Tuple[float, float, float, float]:
        xs, ys, ws, hs = [], [], [], []
        for d, _ in self.history:
            xs.append(d.bbox.center.position.x)
            ys.append(d.bbox.center.position.y)
            ws.append(d.bbox.size_x)
            hs.append(d.bbox.size_y)
        n = len(xs)
        return (sum(xs) / n, sum(ys) / n, sum(ws) / n, sum(hs) / n)

    def avg_score(self) -> float:
        scores = [d.results[0].hypothesis.score for d, _ in self.history if d.results]
        return sum(scores) / len(scores) if scores else 0.0


class DetectionSmoother(Node):
    def __init__(self):
        super().__init__('detection_smoother')

        # Parameters – you can tweak at runtime
        self.declare_parameter('window_frames', 3)   # how many past messages to keep
        self.declare_parameter('min_hits', 2)       # detections must appear in ≥ this many frames
        self.declare_parameter('iou_threshold', 0.5)

        self.window_frames = self.get_parameter('window_frames').get_parameter_value().integer_value
        self.min_hits = self.get_parameter('min_hits').get_parameter_value().integer_value
        self.iou_thresh = self.get_parameter('iou_threshold').get_parameter_value().double_value

        # FIFO of raw detection messages (oldest → newest)
        self.msg_window: collections.deque = collections.deque(maxlen=self.window_frames)

        # List of active per‑object tracks
        self.tracks: List[SmoothedTrack] = []

        self.sub = self.create_subscription(
            Detection2DArray,
            'detections',
            self.raw_cb,
            qos_profile_sensor_data)

        self.pub = self.create_publisher(
            Detection2DArray,
            'detections_smoothed',
            qos_profile_sensor_data)

        self.get_logger().info('DetectionSmoother ready – publishing /detections_smoothed')

    # ---------------------------------------------------------------------
    def raw_cb(self, msg: Detection2DArray):
        """Callback for raw detections."""
        ts = time.time()
        self.msg_window.append((msg, ts))

        # -------------------------------------------------------------
        # 1. Associate current detections with existing tracks
        # -------------------------------------------------------------
        current_dets = list(msg.detections)  # copy for safe iteration

        # Try to match each detection to an existing track
        unmatched = []
        for det in current_dets:
            best_iou = 0.0
            best_track = None
            # Convert detection bbox to (cx, cy, w, h) for IoU computation
            det_bbox = (
                det.bbox.center.position.x,
                det.bbox.center.position.y,
                det.bbox.size_x,
                det.bbox.size_y,
            )
            for track in self.tracks:
                # Compare with the *most recent* bbox of the track
                cx, cy, w, h = track.avg_bbox()
                i = iou(det_bbox, (cx, cy, w, h))
                if i > best_iou:
                    best_iou = i
                    best_track = track

            if best_iou >= self.iou_thresh and best_track is not None:
                best_track.add(det, ts)
            else:
                unmatched.append(det)

        # -------------------------------------------------------------
        # 2. Start new tracks for unmatched detections
        # -------------------------------------------------------------
        for det in unmatched:
            self.tracks.append(SmoothedTrack(det, ts))

        # -------------------------------------------------------------
        # 3. Prune stale tracks (no hits in the last window)
        # -------------------------------------------------------------
        max_age = self.window_frames * 0.5  # assume ~2 Hz detections -> 1.5 s, but we run at 30Hz!
        # At 30 Hz, 3 frames is 0.1 seconds.
        max_age = max(0.5, self.window_frames * 0.1)
        self.tracks = [t for t in self.tracks if any(t.recent(max_age))]

        # -------------------------------------------------------------
        # 4. Build the smoothed output message
        # -------------------------------------------------------------
        out_msg = Detection2DArray()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id

        for track in self.tracks:
            recent = track.recent(max_age)
            if len(recent) >= self.min_hits:
                cx, cy, w, h = track.avg_bbox()
                score = track.avg_score()

                # Build a new detection object
                new_det = Detection2D()
                new_det.bbox.center.position.x = cx
                new_det.bbox.center.position.y = cy
                new_det.bbox.size_x = w
                new_det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "0"          # human class
                hyp.hypothesis.score = float(score)
                new_det.results.append(hyp)

                out_msg.detections.append(new_det)

        # Publish the smoothed array
        self.pub.publish(out_msg)

    # ---------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = DetectionSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
