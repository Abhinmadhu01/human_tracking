#!/usr/bin/env python3
"""
Human Tracker Pro - Fixed for numpy types
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from mavros_msgs.msg import State
from dataclasses import dataclass
from enum import Enum
from rcl_interfaces.msg import SetParametersResult
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

        self.declare_parameter('detections_topic', '/tracked_detections')
        self.declare_parameter('allowed_modes', ['GUIDED', 'AUTO'])
        
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('target_distance_m', 3.0)
        self.declare_parameter('distance_tolerance_m', 0.35)
        self.declare_parameter('person_height_m', 1.70)
        self.declare_parameter('camera_focal_length_px', 205.47)
        self.declare_parameter('min_box_height_px', 50.0)
        self.declare_parameter('target_box_height', 380.0)
        self.declare_parameter('height_tolerance', 15.0)
        self.declare_parameter('fwd_sign', 1.0)
        
        # Search parameters
        self.declare_parameter('default_search_yaw_sign', -1.0)
        self.declare_parameter('search_speed', 0.8)
        self.declare_parameter('initial_radius', 1.0)
        self.declare_parameter('spiral_growth', 0.1)
        self.declare_parameter('search_max_radius', 1.0)
        self.declare_parameter('search_min_yaw_rate', 0.2)
        self.declare_parameter('recovery_time', 2.0)
        self.declare_parameter('detection_timeout', 0.35)
        
        # Gains
        self.declare_parameter('measurement_alpha', 0.3)
        self.declare_parameter('kp_strafe', 0.0012)
        self.declare_parameter('ki_strafe', 0.0)
        self.declare_parameter('kd_strafe', 0.0)
        self.declare_parameter('int_strafe_lim', 200.0)
        self.declare_parameter('kp_vertical', 0.0)
        self.declare_parameter('kp_dist_m', 0.20)
        self.declare_parameter('ki_dist_m', 0.0)
        self.declare_parameter('kd_dist_m', 0.0)
        self.declare_parameter('int_dist_m_lim', 5.0)
        self.declare_parameter('kp_dist', 0.004)
        self.declare_parameter('kp_yaw', 0.0015)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.0)
        self.declare_parameter('int_yaw_lim', 200.0)
        self.declare_parameter('kp_alt', 0.5)
        self.declare_parameter('ki_alt', 0.0)
        self.declare_parameter('kd_alt', 0.0)
        self.declare_parameter('int_alt_lim', 1.0)
        
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

        self.detections_topic = str(self.get_parameter('detections_topic').value)
        
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
        self.search_max_radius = float(self.get_parameter('search_max_radius').value)
        self.search_min_yaw_rate = float(self.get_parameter('search_min_yaw_rate').value)
        self.recovery_time = float(self.get_parameter('recovery_time').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        
        self.measurement_alpha = float(self.get_parameter('measurement_alpha').value)
        self.kp_strafe = float(self.get_parameter('kp_strafe').value)
        self.ki_strafe = float(self.get_parameter('ki_strafe').value)
        self.kd_strafe = float(self.get_parameter('kd_strafe').value)
        self.int_strafe_lim = float(self.get_parameter('int_strafe_lim').value)
        self.kp_vertical = float(self.get_parameter('kp_vertical').value)
        self.kp_dist_m = float(self.get_parameter('kp_dist_m').value)
        self.ki_dist_m = float(self.get_parameter('ki_dist_m').value)
        self.kd_dist_m = float(self.get_parameter('kd_dist_m').value)
        self.int_dist_m_lim = float(self.get_parameter('int_dist_m_lim').value)
        self.kp_dist = float(self.get_parameter('kp_dist').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.int_yaw_lim = float(self.get_parameter('int_yaw_lim').value)
        self.kp_alt = float(self.get_parameter('kp_alt').value)
        self.ki_alt = float(self.get_parameter('ki_alt').value)
        self.kd_alt = float(self.get_parameter('kd_alt').value)
        self.int_alt_lim = float(self.get_parameter('int_alt_lim').value)
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

        # State
        now = self._now()
        self.state = TrackerState.SEARCHING
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.search_start_time = now
        self.last_sight_time = 0.0
        self.target_measurement = TargetMeasurement(stamp=0.0)
        self.last_log_time = 0.0
        self.prev_cmd = Twist()
        self.prev_cmd_time = now
        self.last_cmd_publish_time = now
        self.prev_control_time = now

        self._int_dist_m = 0.0
        self._prev_err_dist_m = 0.0
        self._int_strafe = 0.0
        self._prev_err_strafe = 0.0
        self._int_yaw = 0.0
        self._prev_err_yaw = 0.0
        self._int_alt = 0.0
        self._prev_err_alt = 0.0

        allowed_modes = self.get_parameter('allowed_modes').value
        if isinstance(allowed_modes, str):
            allowed_modes = [allowed_modes]
        self.allowed_modes = {str(mode).upper() for mode in allowed_modes}
        self.fc_armed = False
        self.fc_mode = ''
        
        self.search_direction = SearchDirection.UNKNOWN
        self.current_search_yaw_sign = self.default_search_yaw_sign

        # ROS 2 Setup
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.create_subscription(Detection2DArray, self.detections_topic, self.yolo_callback, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        self.create_subscription(State, '/mavros/state', self.state_callback, state_qos)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f"Tracker ready | Center=({self.cx}, {self.cy}) | "
            f"Search: LEFT→anti-clockwise | RIGHT→clockwise"
        )
        
        # Add parameter callback for live rqt_reconfigure tuning
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'target_altitude':
                self.target_altitude = float(param.value)
            elif param.name == 'target_distance_m':
                self.target_distance_m = float(param.value)
            elif param.name == 'target_box_height':
                self.target_box_height = float(param.value)
            elif param.name == 'kp_dist':
                self.kp_dist = float(param.value)
            elif param.name == 'kp_dist_m':
                self.kp_dist_m = float(param.value)
            elif param.name == 'kp_strafe':
                self.kp_strafe = float(param.value)
            elif param.name == 'kp_vertical':
                self.kp_vertical = float(param.value)
            elif param.name == 'kp_yaw':
                self.kp_yaw = float(param.value)
            elif param.name == 'kp_alt':
                self.kp_alt = float(param.value)
            elif param.name == 'fwd_sign':
                self.fwd_sign = float(param.value)
            
            self.get_logger().info(f"Live Tuning: Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

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
        now = self._now()
        if (now - self.last_log_time) >= self.log_interval_sec:
            self.get_logger().info(message)
            self.last_log_time = now

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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

    def state_callback(self, msg: State):
        self.fc_armed = bool(msg.armed)
        self.fc_mode = str(msg.mode).upper()

    def _flight_control_ok(self) -> bool:
        if not self.fc_armed:
            return False
        if not self.allowed_modes:
            return True
        return self.fc_mode in self.allowed_modes

    def _publish_zero(self, now):
        if not rclpy.ok():
            return
        self.vel_pub.publish(Twist())
        self.last_cmd_publish_time = now

    def _watchdog(self):
        now = self._now()
        if (now - self.last_cmd_publish_time) > 0.5:
            self._publish_zero(now)

    def _determine_search_direction(self, last_x_position):
        if last_x_position < self.cx:
            return SearchDirection.LEFT
        else:
            return SearchDirection.RIGHT

    def _pid_step(self, error, prev_error, integral, kp, ki, kd, int_lim, dt):
        if int_lim > 0.0:
            integral = clamp(integral + (error * dt), -int_lim, int_lim)
        else:
            integral = integral + (error * dt)
        derivative = (error - prev_error) / dt if dt > 1e-6 else 0.0
        output = (kp * error) + (ki * integral) + (kd * derivative)
        return output, integral, error

    def _reset_tracking_pid(self):
        self._int_dist_m = 0.0
        self._prev_err_dist_m = 0.0
        self._int_strafe = 0.0
        self._prev_err_strafe = 0.0
        self._int_yaw = 0.0
        self._prev_err_yaw = 0.0

    def yolo_callback(self, msg):
        target = None
        best_height = 0
        
        for detection in msg.detections:
            class_name = ''
            if detection.results:
                class_name = str(detection.results[0].hypothesis.class_id)
            base_class = class_name.split('_', 1)[0] if class_name else ''
            if base_class == self.target_class or base_class == 'person':
                h = detection.bbox.size_y
                if h > best_height:
                    best_height = h
                    target = detection

        if target:
            now = self._now()
            
            # Get raw pixel coordinates
            raw_err_x = float(self.cx - target.bbox.center.position.x)
            raw_err_y = float(self.cy - target.bbox.center.position.y)
            raw_box_h = float(target.bbox.size_y)
            
            # Store last seen position
            last_x = float(target.bbox.center.position.x)
            
            raw_distance_m = self._estimate_distance_m(raw_box_h)

            # EMA smoothing on raw measurements
            a = clamp(self.measurement_alpha, 0.0, 1.0)
            self.target_measurement.err_x = float(a * raw_err_x + (1.0 - a) * self.target_measurement.err_x)
            self.target_measurement.err_y = float(a * raw_err_y + (1.0 - a) * self.target_measurement.err_y)
            self.target_measurement.box_h = float(a * raw_box_h + (1.0 - a) * self.target_measurement.box_h)
            
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
        now = self._now()
        dt = max(0.02, now - self.prev_control_time)
        cmd = Twist()
        target_visible = self._is_target_fresh(now)

        if target_visible:
            if self.state != TrackerState.TRACKING:
                self.get_logger().info("Target acquired - TRACKING")
                self.state = TrackerState.TRACKING
                self.search_direction = SearchDirection.UNKNOWN
            
            cmd = self._execute_tracking(now, dt)

        else:
            self._reset_tracking_pid()
            time_since_lost = now - self.last_sight_time

            if self.last_sight_time > 0.0 and time_since_lost < self.recovery_time:
                if self.state != TrackerState.RECOVERY:
                    self.get_logger().info("Target lost - RECOVERY (holding position)")
                    self.state = TrackerState.RECOVERY
                cmd = self._execute_recovery(dt)
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
                    
                cmd = self._execute_smart_search(now, dt)

        smooth_cmd = self._smooth_command(cmd, now)
        if self._flight_control_ok():
            if rclpy.ok():
                self.vel_pub.publish(smooth_cmd)
                self.last_cmd_publish_time = now
        else:
            self.prev_cmd = Twist()
            self.prev_cmd_time = now
        self.prev_control_time = now

    def _execute_tracking(self, now, dt):
        cmd = Twist()

        # Altitude
        # If kp_vertical is tuned (> 0), use the bounding box Y error to control altitude (fly up/down to center target)
        if self.kp_vertical > 0.0:
            err_y = self.target_measurement.err_y
            alt_cmd = err_y * self.kp_vertical
            # Safety floor
            if self.current_alt < 1.0 and alt_cmd < 0.0:
                alt_cmd = 0.0
        else:
            # Otherwise use fixed altitude hold
            alt_err = self.target_altitude - self.current_alt
            alt_cmd, self._int_alt, self._prev_err_alt = self._pid_step(
                alt_err,
                self._prev_err_alt,
                self._int_alt,
                self.kp_alt,
                self.ki_alt,
                self.kd_alt,
                self.int_alt_lim,
                dt,
            )
        cmd.linear.z = clamp(alt_cmd, -self.max_vz, self.max_vz)

        # Distance
        est_distance_m = self.target_measurement.est_distance_m
        dist_error_m = 0.0
        body_vx = 0.0
        
        if math.isfinite(est_distance_m):
            dist_error_m = est_distance_m - self.target_distance_m
            if abs(dist_error_m) < self.distance_tolerance_m:
                dist_error_m = 0.0
            dist_cmd, self._int_dist_m, self._prev_err_dist_m = self._pid_step(
                dist_error_m,
                self._prev_err_dist_m,
                self._int_dist_m,
                self.kp_dist_m,
                self.ki_dist_m,
                self.kd_dist_m,
                self.int_dist_m_lim,
                dt,
            )
            body_vx = dist_cmd * self.fwd_sign
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
        strafe_cmd, self._int_strafe, self._prev_err_strafe = self._pid_step(
            err_x,
            self._prev_err_strafe,
            self._int_strafe,
            self.kp_strafe,
            self.ki_strafe,
            self.kd_strafe,
            self.int_strafe_lim,
            dt,
        )
        body_vy = strafe_cmd

        # Yaw
        if abs(err_x) > self.yaw_deadband_px:
            yaw_cmd, self._int_yaw, self._prev_err_yaw = self._pid_step(
                err_x,
                self._prev_err_yaw,
                self._int_yaw,
                self.kp_yaw,
                self.ki_yaw,
                self.kd_yaw,
                self.int_yaw_lim,
                dt,
            )
            cmd.angular.z = clamp(yaw_cmd, -self.max_yaw_rate, self.max_yaw_rate)

        # Transform body velocities (forward/left) to global ENU frame (East/North)
        # MAVROS cmd_vel defaults to LOCAL_ENU unless configured otherwise.
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
        
    def _execute_recovery(self, dt):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = self._altitude_hold(dt)
        cmd.angular.z = 0.0
        return cmd
        
    def _execute_smart_search(self, now, dt):
        cmd = Twist()
        
        elapsed = now - self.search_start_time
        current_radius = max(0.1, self.initial_radius + (self.spiral_growth * elapsed))
        if self.search_max_radius > 0.0:
            current_radius = min(current_radius, self.search_max_radius)
        
        target_yaw_rate = (self.search_speed / current_radius) * self.current_search_yaw_sign
        if abs(target_yaw_rate) < self.search_min_yaw_rate:
            target_yaw_rate = self.search_min_yaw_rate * self.current_search_yaw_sign
        
        real_forward_speed = self.search_speed * self.fwd_sign
        
        # Search pattern also needs global transformation
        global_search_vx = real_forward_speed * math.cos(self.current_yaw)
        global_search_vy = real_forward_speed * math.sin(self.current_yaw)
        
        cmd.linear.x = clamp(global_search_vx, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(global_search_vy, -self.max_vy, self.max_vy)
        
        cmd.angular.z = clamp(target_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        cmd.linear.z = self._altitude_hold(dt)
        
        return cmd

    def _altitude_hold(self, dt):
        # Safety guards
        if self.current_alt < 1.0:  # Hard floor (1m)
            return self.max_vz
        if self.current_alt > 30.0: # Hard ceiling (30m)
            return -self.max_vz
            
        alt_error = self.target_altitude - self.current_alt
        alt_cmd, self._int_alt, self._prev_err_alt = self._pid_step(
            alt_error,
            self._prev_err_alt,
            self._int_alt,
            self.kp_alt,
            self.ki_alt,
            self.kd_alt,
            self.int_alt_lim,
            dt,
        )
        return clamp(alt_cmd, -self.max_vz, self.max_vz)


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