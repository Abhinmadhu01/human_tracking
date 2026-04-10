#!/usr/bin/env python3
"""
Human Tracker Pro - Fixed for numpy types
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from dataclasses import dataclass
from enum import Enum
import time
import math


class TrackerState(str, Enum):
    SEARCHING = "SEARCHING"
    RECOVERY = "RECOVERY"
    TRACKING = "TRACKING"


class SearchDirection(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UNKNOWN = "UNKNOWN"


def clamp(value, low, high):
    """Clamp value, handling both Python floats and numpy types"""
    # Convert numpy types to Python float
    if isinstance(value, np.ndarray):
        value = float(value.item()) if value.size == 1 else float(value[0])
    elif hasattr(value, 'item'):
        value = float(value.item())
    else:
        value = float(value)
    
    low = float(low)
    high = float(high)
    
    return max(low, min(high, value))


@dataclass
class TargetMeasurement:
    err_x: float = 0.0
    err_y: float = 0.0
    box_h: float = 0.0
    est_distance_m: float = float('nan')
    stamp: float = 0.0
    last_position_x: float = 320.0


class HumanTrackerPro(Node):
    def __init__(self):
        super().__init__('human_tracker_pro')

        # Camera parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('image_center_y', 240.0)
        
        self.declare_parameter('target_class', '0')
        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('target_distance_m', 3.0)
        self.declare_parameter('distance_tolerance_m', 0.35)
        self.declare_parameter('person_height_m', 1.70)
        self.declare_parameter('camera_focal_length_px', 700.0)
        self.declare_parameter('min_box_height_px', 50.0)
        self.declare_parameter('target_box_height', 225.0)
        self.declare_parameter('height_tolerance', 15.0)
        self.declare_parameter('fwd_sign', 1.0)
        
        # Search parameters
        self.declare_parameter('default_search_yaw_sign', -1.0)
        self.declare_parameter('search_speed', 0.8)
        self.declare_parameter('initial_radius', 1.0)
        self.declare_parameter('spiral_growth', 0.1)
        self.declare_parameter('recovery_time', 2.0)
        self.declare_parameter('detection_timeout', 0.35)
        
        # Gains
        self.declare_parameter('measurement_alpha', 0.3)
        self.declare_parameter('kp_strafe', 0.0012)
        self.declare_parameter('kp_vertical', 0.0012)
        self.declare_parameter('kp_dist_m', 0.20)
        self.declare_parameter('kp_dist', 0.004)
        self.declare_parameter('kp_yaw', 0.0015)
        self.declare_parameter('kp_alt', 0.5)
        
        self.declare_parameter('center_deadband_px', 25.0)
        self.declare_parameter('yaw_deadband_px', 30.0)
        self.declare_parameter('max_vx', 0.6)
        self.declare_parameter('max_vy', 0.6)
        self.declare_parameter('max_vz', 0.5)
        self.declare_parameter('max_yaw_rate', 0.5)
        
        # Smoother
        self.declare_parameter('cmd_smoothing_alpha', 0.3)
        self.declare_parameter('max_ax', 0.5)
        self.declare_parameter('max_ay', 0.5)
        self.declare_parameter('max_az', 0.3)
        self.declare_parameter('max_yaw_accel', 0.4)
        
        self.declare_parameter('log_interval_sec', 0.5)

        # Load parameters
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.cx = self.get_parameter('image_center_x').value
        self.cy = self.get_parameter('image_center_y').value
        
        self.get_logger().info(f"Camera: {self.img_width}x{self.img_height}, Center: ({self.cx}, {self.cy})")

        self.target_class = str(self.get_parameter('target_class').value)
        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.distance_tolerance_m = float(self.get_parameter('distance_tolerance_m').value)
        self.person_height_m = float(self.get_parameter('person_height_m').value)
        self.camera_focal_length_px = float(self.get_parameter('camera_focal_length_px').value)
        self.min_box_height_px = float(self.get_parameter('min_box_height_px').value)
        self.target_box_height = float(self.get_parameter('target_box_height').value)
        self.height_tolerance = float(self.get_parameter('height_tolerance').value)
        self.fwd_sign = float(self.get_parameter('fwd_sign').value)
        
        self.default_search_yaw_sign = float(self.get_parameter('default_search_yaw_sign').value)
        self.search_speed = float(self.get_parameter('search_speed').value)
        self.initial_radius = float(self.get_parameter('initial_radius').value)
        self.spiral_growth = float(self.get_parameter('spiral_growth').value)
        self.recovery_time = float(self.get_parameter('recovery_time').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        
        self.measurement_alpha = float(self.get_parameter('measurement_alpha').value)
        self.kp_strafe = float(self.get_parameter('kp_strafe').value)
        self.kp_vertical = float(self.get_parameter('kp_vertical').value)
        self.kp_dist_m = float(self.get_parameter('kp_dist_m').value)
        self.kp_dist = float(self.get_parameter('kp_dist').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kp_alt = float(self.get_parameter('kp_alt').value)
        self.center_deadband_px = float(self.get_parameter('center_deadband_px').value)
        self.yaw_deadband_px = float(self.get_parameter('yaw_deadband_px').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_vz = float(self.get_parameter('max_vz').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.cmd_smoothing_alpha = float(self.get_parameter('cmd_smoothing_alpha').value)
        self.max_ax = float(self.get_parameter('max_ax').value)
        self.max_ay = float(self.get_parameter('max_ay').value)
        self.max_az = float(self.get_parameter('max_az').value)
        self.max_yaw_accel = float(self.get_parameter('max_yaw_accel').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        # EKF
        self.ekf = KalmanFilter(dim_x=6, dim_z=3)
        self.ekf.F = np.array([
            [1, 0, 0, 0.05, 0,  0 ],
            [0, 1, 0, 0,  0.05, 0 ],
            [0, 0, 1, 0,  0,  0.05],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ])
        self.ekf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        self.ekf.Q = np.eye(6) * 0.01
        self.ekf.R = np.eye(3) * 0.5
        self.ekf.P *= 10

        # State
        now = time.time()
        self.state = TrackerState.SEARCHING
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.search_start_time = now
        self.last_sight_time = 0.0
        self.target_measurement = TargetMeasurement(stamp=0.0)
        self.last_log_time = 0.0
        self.prev_cmd = Twist()
        self.prev_cmd_time = now
        
        self.search_direction = SearchDirection.UNKNOWN
        self.current_search_yaw_sign = self.default_search_yaw_sign

        # ROS 2 Setup
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.create_subscription(Detection2DArray, '/detections', self.yolo_callback, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Tracker ready | Center=({self.cx}, {self.cy}) | "
            f"Search: LEFT→anti-clockwise | RIGHT→clockwise"
        )

    def _estimate_distance_m(self, box_height_px):
        if box_height_px < self.min_box_height_px:
            return float('nan')
        return (self.person_height_m * self.camera_focal_length_px) / box_height_px

    def _is_target_fresh(self, now):
        return (now - self.target_measurement.stamp) <= self.detection_timeout

    def _set_state(self, new_state, message):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(message)

    def _throttled_log(self, message):
        now = time.time()
        if (now - self.last_log_time) >= self.log_interval_sec:
            self.get_logger().info(message)
            self.last_log_time = now

    def _clamp_delta(self, new_value, prev_value, max_delta):
        # Convert to Python floats
        new_value = float(new_value) if not isinstance(new_value, np.ndarray) else float(new_value.item() if new_value.size == 1 else new_value[0])
        prev_value = float(prev_value) if not isinstance(prev_value, np.ndarray) else float(prev_value.item() if prev_value.size == 1 else prev_value[0])
        max_delta = float(max_delta)
        
        return prev_value + clamp(new_value - prev_value, -max_delta, max_delta)

    def _smooth_command(self, cmd, now):
        dt = max(0.02, now - self.prev_cmd_time)
        alpha = clamp(self.cmd_smoothing_alpha, 0.05, 1.0)

        out = Twist()
        fx = alpha * cmd.linear.x + (1.0 - alpha) * self.prev_cmd.linear.x
        fy = alpha * cmd.linear.y + (1.0 - alpha) * self.prev_cmd.linear.y
        fz = alpha * cmd.linear.z + (1.0 - alpha) * self.prev_cmd.linear.z
        fw = alpha * cmd.angular.z + (1.0 - alpha) * self.prev_cmd.angular.z

        out.linear.x = self._clamp_delta(fx, self.prev_cmd.linear.x, self.max_ax * dt)
        out.linear.y = self._clamp_delta(fy, self.prev_cmd.linear.y, self.max_ay * dt)
        out.linear.z = self._clamp_delta(fz, self.prev_cmd.linear.z, self.max_az * dt)
        out.angular.z = self._clamp_delta(fw, self.prev_cmd.angular.z, self.max_yaw_accel * dt)

        out.linear.x = clamp(out.linear.x, -self.max_vx, self.max_vx)
        out.linear.y = clamp(out.linear.y, -self.max_vy, self.max_vy)
        out.linear.z = clamp(out.linear.z, -self.max_vz, self.max_vz)
        out.angular.z = clamp(out.angular.z, -self.max_yaw_rate, self.max_yaw_rate)

        self.prev_cmd = out
        self.prev_cmd_time = now
        return out

    def pose_callback(self, msg):
        if not math.isfinite(msg.pose.position.z):
            return
        self.current_alt = float(msg.pose.position.z)
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = float(math.atan2(siny_cosp, cosy_cosp))

    def _determine_search_direction(self, last_x_position):
        if last_x_position < self.cx:
            return SearchDirection.LEFT
        else:
            return SearchDirection.RIGHT

    def yolo_callback(self, msg):
        target = None
        best_height = 0
        
        for detection in msg.detections:
            class_name = ''
            if detection.results:
                class_name = str(detection.results[0].hypothesis.class_id)
            if class_name == self.target_class or class_name == 'person':
                h = detection.bbox.size_y
                if h > best_height:
                    best_height = h
                    target = detection

        if target:
            now = time.time()
            
            # Get raw pixel coordinates
            raw_err_x = float(self.cx - target.bbox.center.position.x)
            raw_err_y = float(self.cy - target.bbox.center.position.y)
            raw_box_h = float(target.bbox.size_y)
            
            # Store last seen position
            last_x = float(target.bbox.center.position.x)
            
            raw_distance_m = self._estimate_distance_m(raw_box_h)

            # Update EKF
            self.ekf.predict()
            measurement = np.array([raw_err_x, raw_err_y, raw_box_h])
            confidence = float(target.results[0].hypothesis.score) if target.results else 0.5
            self.ekf.R = np.eye(3) * (0.5 / max(confidence, 0.1))
            self.ekf.update(measurement)
            
            # Get smoothed values - CONVERT TO PYTHON FLOATS
            smoothed = self.ekf.x
            smoothed_err_x = float(smoothed[0])
            smoothed_err_y = float(smoothed[1])
            smoothed_box_h = float(smoothed[2])
            
            # EMA smoothing - all as Python floats
            a = clamp(self.measurement_alpha, 0.0, 1.0)
            self.target_measurement.err_x = float(a * smoothed_err_x + (1.0 - a) * self.target_measurement.err_x)
            self.target_measurement.err_y = float(a * smoothed_err_y + (1.0 - a) * self.target_measurement.err_y)
            self.target_measurement.box_h = float(a * smoothed_box_h + (1.0 - a) * self.target_measurement.box_h)
            
            # Store last position
            self.target_measurement.last_position_x = float(last_x)
            
            if math.isfinite(raw_distance_m):
                prev_d = self.target_measurement.est_distance_m
                if math.isfinite(prev_d):
                    self.target_measurement.est_distance_m = float(a * raw_distance_m + (1.0 - a) * prev_d)
                else:
                    self.target_measurement.est_distance_m = float(raw_distance_m)
                    
            self.target_measurement.stamp = float(now)
            self.last_sight_time = float(now)

    def control_loop(self):
        now = time.time()
        cmd = Twist()
        target_visible = self._is_target_fresh(now)

        if target_visible:
            if self.state != TrackerState.TRACKING:
                self.get_logger().info("Target acquired - TRACKING")
                self.state = TrackerState.TRACKING
                self.search_direction = SearchDirection.UNKNOWN
            
            cmd = self._execute_tracking(now)

        else:
            time_since_lost = now - self.last_sight_time

            if self.last_sight_time > 0.0 and time_since_lost < self.recovery_time:
                if self.state != TrackerState.RECOVERY:
                    self.get_logger().info("Target lost - RECOVERY (holding position)")
                    self.state = TrackerState.RECOVERY
                cmd = self._execute_recovery()
            else:
                if self.state != TrackerState.SEARCHING:
                    self.search_direction = self._determine_search_direction(
                        self.target_measurement.last_position_x
                    )
                    
                    if self.search_direction == SearchDirection.LEFT:
                        self.current_search_yaw_sign = 1.0
                        direction_str = "anti-clockwise (LEFT side lost)"
                    elif self.search_direction == SearchDirection.RIGHT:
                        self.current_search_yaw_sign = -1.0
                        direction_str = "clockwise (RIGHT side lost)"
                    else:
                        self.current_search_yaw_sign = self.default_search_yaw_sign
                        direction_str = f"default ({self.default_search_yaw_sign})"
                    
                    self.get_logger().info(
                        f"Starting SEARCH | Last seen x={self.target_measurement.last_position_x:.0f} | "
                        f"Direction: {direction_str}"
                    )
                    self.search_start_time = now
                    self.state = TrackerState.SEARCHING
                    
                cmd = self._execute_smart_search(now)

        smooth_cmd = self._smooth_command(cmd, now)
        self.vel_pub.publish(smooth_cmd)

    def _execute_tracking(self, now):
        cmd = Twist()

        # Altitude
        alt_err = self.target_altitude - self.current_alt
        cmd.linear.z = clamp(alt_err * self.kp_alt, -self.max_vz, self.max_vz)

        # Distance
        est_distance_m = self.target_measurement.est_distance_m
        dist_error_m = 0.0
        body_vx = 0.0
        
        if math.isfinite(est_distance_m):
            dist_error_m = est_distance_m - self.target_distance_m
            if abs(dist_error_m) < self.distance_tolerance_m:
                dist_error_m = 0.0
            body_vx = dist_error_m * self.kp_dist_m * self.fwd_sign
        else:
            box_h = self.target_measurement.box_h
            lower = self.target_box_height - self.height_tolerance
            upper = self.target_box_height + self.height_tolerance
            dist_error_px = 0.0
            if box_h < lower or box_h > upper:
                dist_error_px = self.target_box_height - box_h
            body_vx = dist_error_px * self.kp_dist * self.fwd_sign

        # Centering
        err_x = self.target_measurement.err_x
        
        if abs(err_x) < self.center_deadband_px:
            err_x = 0.0
        body_vy = err_x * self.kp_strafe

        # Yaw
        if abs(err_x) > self.yaw_deadband_px:
            cmd.angular.z = clamp(err_x * self.kp_yaw, -self.max_yaw_rate, self.max_yaw_rate)

        # Transform to global frame
        global_vx = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
        global_vy = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)

        cmd.linear.x = clamp(global_vx, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(global_vy, -self.max_vy, self.max_vy)

        # Use Python floats for logging
        err_x_float = float(self.target_measurement.err_x)
        err_y_float = float(self.target_measurement.err_y)
        self._throttled_log(
            f"track dist={float(est_distance_m):.2f}m err=({err_x_float:.0f}, {err_y_float:.0f})px "
            f"vx={float(cmd.linear.x):.2f} vy={float(cmd.linear.y):.2f}"
        )
        
        return cmd
        
    def _execute_recovery(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = self._altitude_hold()
        cmd.angular.z = 0.0
        return cmd
        
    def _execute_smart_search(self, now):
        cmd = Twist()
        
        dt = now - self.search_start_time
        current_radius = max(0.1, self.initial_radius + (self.spiral_growth * dt))
        
        target_yaw_rate = (self.search_speed / current_radius) * self.current_search_yaw_sign
        
        real_forward_speed = self.search_speed * self.fwd_sign
        
        cmd.linear.x = real_forward_speed * math.cos(self.current_yaw)
        cmd.linear.y = real_forward_speed * math.sin(self.current_yaw)
        
        cmd.linear.x = clamp(cmd.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(cmd.linear.y, -self.max_vy, self.max_vy)
        
        cmd.angular.z = clamp(target_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        cmd.linear.z = self._altitude_hold()
        
        return cmd

    def _altitude_hold(self):
        alt_error = self.target_altitude - self.current_alt
        return clamp(self.kp_alt * alt_error, -self.max_vz, self.max_vz)


def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerPro()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()