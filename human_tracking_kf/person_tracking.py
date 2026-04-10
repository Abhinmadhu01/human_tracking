#!/usr/bin/env python3
"""
Human Tracker Pro - With YOUR Perfect Circular Search & Smoother
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

# --- PURE PYTHON CLAMP (Fixes the ROS2 numpy.float64 silent assignment bug) ---
def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))

@dataclass
class PIDConfig:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 1.0
    output_limit: float = 1.0


class CascadePIDController:
    def __init__(self, outer_config: PIDConfig, inner_config: PIDConfig, dt: float = 0.05):
        self.dt = dt
        self.outer_kp = outer_config.kp
        self.outer_ki = outer_config.ki
        self.outer_kd = outer_config.kd
        self.outer_integral_limit = outer_config.integral_limit
        self.outer_output_limit = outer_config.output_limit
        
        self.inner_kp = inner_config.kp
        self.inner_ki = inner_config.ki
        self.inner_kd = inner_config.kd
        self.inner_output_limit = inner_config.output_limit
        
        self.pos_integral = 0.0
        self.pos_prev_error = 0.0
        self.vel_integral = 0.0
        self.vel_prev_error = 0.0
        
    def reset(self):
        self.pos_integral = 0.0
        self.pos_prev_error = 0.0
        self.vel_integral = 0.0
        self.vel_prev_error = 0.0
        
    def compute(self, position_error: float, current_velocity: float) -> float:
        # Outer loop
        self.pos_integral += position_error * self.dt
        self.pos_integral = clamp(self.pos_integral, -self.outer_integral_limit, self.outer_integral_limit)
        
        pos_derivative = (position_error - self.pos_prev_error) / self.dt
        self.pos_prev_error = position_error
        
        velocity_setpoint = (
            self.outer_kp * position_error +
            self.outer_ki * self.pos_integral +
            self.outer_kd * pos_derivative
        )
        velocity_setpoint = clamp(velocity_setpoint, -self.outer_output_limit, self.outer_output_limit)
        
        # Inner loop
        velocity_error = velocity_setpoint - current_velocity
        
        self.vel_integral += velocity_error * self.dt
        self.vel_integral = clamp(self.vel_integral, -1.0, 1.0)
        
        vel_derivative = (velocity_error - self.vel_prev_error) / self.dt
        self.vel_prev_error = velocity_error
        
        acceleration = (
            self.inner_kp * velocity_error +
            self.inner_ki * self.vel_integral +
            self.inner_kd * vel_derivative
        )
        
        velocity_cmd = current_velocity + acceleration * self.dt
        return clamp(velocity_cmd, -self.inner_output_limit, self.inner_output_limit)


class HumanStateEKF:
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        
        self.kf.F = np.array([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ])
        
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        self.kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=2.0, block_size=2)
        self.kf.R = np.eye(3) * 0.1
        self.kf.P *= 10
        
        self.is_initialized = False
        self.last_update_time = 0.0
        
    def predict(self):
        self.kf.predict()
        
    def update(self, measurement: np.ndarray, confidence: float = 1.0):
        noise_var = 0.1 / max(confidence, 0.1)
        self.kf.R = np.eye(3) * noise_var
        self.kf.update(measurement)
        self.last_update_time = time.time()
        self.is_initialized = True
        
    def get_state(self):
        return self.kf.x.copy()
    
    def get_position(self):
        return self.kf.x[:3].copy()
    
    def get_velocity(self):
        return self.kf.x[3:6].copy()
    
    def get_predicted_position(self, horizon_seconds: float = 0.3):
        if not self.is_initialized:
            return np.array([0.0, 0.0, 3.0])
        steps = int(horizon_seconds / self.dt)
        x_pred = self.kf.x.copy()
        for _ in range(steps):
            x_pred = self.kf.F @ x_pred
        return x_pred[:3]


class HumanTrackerPro(Node):
    def __init__(self):
        super().__init__('human_tracker_pro')
        
        # ==================== PARAMETERS ====================
        self.declare_parameter('target_distance', 3.0)
        self.declare_parameter('distance_tolerance', 0.3)
        self.declare_parameter('target_altitude', 3.0)
        self.declare_parameter('target_class', 'person')
        
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_focal_length', 700.0)
        self.declare_parameter('person_height', 1.70)
        
        # Cascade PID - Distance
        self.declare_parameter('dist_outer_kp', 0.6)
        self.declare_parameter('dist_outer_ki', 0.05)
        self.declare_parameter('dist_outer_kd', 0.2)
        self.declare_parameter('dist_outer_output_limit', 1.5)
        self.declare_parameter('dist_inner_kp', 3.0)
        self.declare_parameter('dist_inner_ki', 0.2)
        self.declare_parameter('dist_inner_kd', 0.5)
        self.declare_parameter('dist_inner_output_limit', 1.0)
        
        # Cascade PID - Lateral
        self.declare_parameter('lat_outer_kp', 0.0010)
        self.declare_parameter('lat_outer_ki', 0.0001)
        self.declare_parameter('lat_outer_kd', 0.0005)
        self.declare_parameter('lat_outer_output_limit', 0.8)
        self.declare_parameter('lat_inner_kp', 2.0)
        self.declare_parameter('lat_inner_ki', 0.1)
        self.declare_parameter('lat_inner_kd', 0.3)
        self.declare_parameter('lat_inner_output_limit', 0.8)
        
        # Yaw and Altitude PID
        self.declare_parameter('yaw_kp', 0.002)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.001)
        self.declare_parameter('alt_kp', 0.5)
        self.declare_parameter('alt_ki', 0.05)
        self.declare_parameter('alt_kd', 0.1)
        
        # Safety limits
        self.declare_parameter('max_vx', 0.8)
        self.declare_parameter('max_vy', 0.8)
        self.declare_parameter('max_vz', 0.5)
        self.declare_parameter('max_yaw_rate', 0.7)
        
        # Smoother Dynamics
        self.declare_parameter('cmd_smoothing_alpha', 0.25)
        self.declare_parameter('max_ax', 0.8)
        self.declare_parameter('max_ay', 0.8)
        self.declare_parameter('max_az', 0.5)
        self.declare_parameter('max_yaw_accel', 0.6)
        
        # Timing
        self.declare_parameter('detection_timeout', 0.35)
        self.declare_parameter('prediction_horizon', 0.3)
        self.declare_parameter('recovery_time', 2.0)
        
        # ==================== YOUR CIRCULAR SEARCH PARAMS ====================
        self.declare_parameter('fwd_sign', 1.0)
        self.declare_parameter('search_yaw_sign', -1.0)
        self.declare_parameter('search_speed', 0.8)
        self.declare_parameter('initial_radius', 1.0)
        self.declare_parameter('spiral_growth', 0.1)
        
        # Load parameters
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.target_class = self.get_parameter('target_class').value
        
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        self.cx = self.img_width / 2.0
        self.cy = self.img_height / 2.0
        self.focal_length = self.get_parameter('camera_focal_length').value
        self.person_height = self.get_parameter('person_height').value
        
        # Smoother parameters
        self.cmd_smoothing_alpha = self.get_parameter('cmd_smoothing_alpha').value
        self.max_ax = self.get_parameter('max_ax').value
        self.max_ay = self.get_parameter('max_ay').value
        self.max_az = self.get_parameter('max_az').value
        self.max_yaw_accel = self.get_parameter('max_yaw_accel').value
        
        # EKF
        self.ekf = HumanStateEKF(dt=0.05)
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        
        # Cascade PID controllers
        self.dist_controller = CascadePIDController(
            outer_config=PIDConfig(
                kp=self.get_parameter('dist_outer_kp').value,
                ki=self.get_parameter('dist_outer_ki').value,
                kd=self.get_parameter('dist_outer_kd').value,
                output_limit=self.get_parameter('dist_outer_output_limit').value
            ),
            inner_config=PIDConfig(
                kp=self.get_parameter('dist_inner_kp').value,
                ki=self.get_parameter('dist_inner_ki').value,
                kd=self.get_parameter('dist_inner_kd').value,
                output_limit=self.get_parameter('dist_inner_output_limit').value
            ),
            dt=0.05
        )
        
        self.lat_controller = CascadePIDController(
            outer_config=PIDConfig(
                kp=self.get_parameter('lat_outer_kp').value,
                ki=self.get_parameter('lat_outer_ki').value,
                kd=self.get_parameter('lat_outer_kd').value,
                output_limit=self.get_parameter('lat_outer_output_limit').value
            ),
            inner_config=PIDConfig(
                kp=self.get_parameter('lat_inner_kp').value,
                ki=self.get_parameter('lat_inner_ki').value,
                kd=self.get_parameter('lat_inner_kd').value,
                output_limit=self.get_parameter('lat_inner_output_limit').value
            ),
            dt=0.05
        )
        
        # Simple PID for yaw and altitude
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.yaw_ki = self.get_parameter('yaw_ki').value
        self.yaw_kd = self.get_parameter('yaw_kd').value
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        
        self.alt_kp = self.get_parameter('alt_kp').value
        self.alt_ki = self.get_parameter('alt_ki').value
        self.alt_kd = self.get_parameter('alt_kd').value
        self.alt_integral = 0.0
        self.alt_prev_error = 0.0
        
        # Safety limits
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_vz = self.get_parameter('max_vz').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        
        # ==================== CRITICAL FIX ====================
        # State and Timing logic (Using time.time() prevents the infinite radius bug)
        now = time.time()
        self.state = TrackerState.SEARCHING
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_altitude = 0.0
        self.last_detection_time = 0.0
        self.last_sight_time = 0.0
        
        # Initialize search_start_time and prev_cmd_time to NOW
        self.search_start_time = now
        self.prev_cmd_time = now
        self.prev_cmd = Twist()
        
        # Velocity estimate
        self.prev_position = np.array([0.0, 0.0, 3.0])
        
        # Timing
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.recovery_time = self.get_parameter('recovery_time').value
        
        # Search variables
        self.fwd_sign = self.get_parameter('fwd_sign').value
        self.search_yaw_sign = self.get_parameter('search_yaw_sign').value
        self.search_speed = self.get_parameter('search_speed').value
        self.initial_radius = self.get_parameter('initial_radius').value
        self.spiral_growth = self.get_parameter('spiral_growth').value
        
        # ROS2 SETUP
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, qos)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.5, self.log_status)
        
        self.get_logger().info("HumanTrackerPro initialized with Fixed Timing & Spiral logic")
        
    def detection_callback(self, msg):
        best_detection = None
        best_score = 0.0
        
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            
            if class_id == self.target_class and score > best_score:
                best_score = score
                best_detection = det
                
        if best_detection is None:
            return
            
        bbox = best_detection.bbox
        pixel_x = bbox.center.position.x
        pixel_y = bbox.center.position.y
        pixel_height = bbox.size_y
        
        distance = self._estimate_distance(pixel_height)
        
        if not np.isfinite(distance) or distance <= 0 or distance > 50:
            return
            
        x_cam = (pixel_x - self.cx) * distance / self.focal_length
        y_cam = (pixel_y - self.cy) * distance / self.focal_length
        z_cam = distance
        
        measurement = np.array([x_cam, y_cam, z_cam])
        
        self.ekf.update(measurement, confidence=best_score)
        self.last_detection_time = time.time()
        self.last_sight_time = time.time()
        
    def _estimate_distance(self, pixel_height):
        if pixel_height < 10:
            return float('nan')
        return (self.person_height * self.focal_length) / pixel_height
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.current_altitude = msg.pose.position.z
        
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        now = time.time()
        
        self.ekf.predict()
        
        time_since_detection = now - self.last_detection_time
        target_visible = time_since_detection < self.detection_timeout
        
        if target_visible:
            if self.state != TrackerState.TRACKING:
                self.get_logger().info("Target acquired - TRACKING")
                self.state = TrackerState.TRACKING
                self.dist_controller.reset()
                self.lat_controller.reset()
                
            cmd = self._execute_tracking(now)
            
        else:
            time_since_lost = now - self.last_sight_time
            
            if self.last_sight_time > 0.0 and time_since_lost < self.recovery_time:
                if self.state != TrackerState.RECOVERY:
                    self.get_logger().info("Target lost - RECOVERY")
                    self.state = TrackerState.RECOVERY
                cmd = self._execute_recovery()
            else:
                if self.state != TrackerState.SEARCHING:
                    self.get_logger().info("Starting SEARCH pattern")
                    self.state = TrackerState.SEARCHING
                    self.search_start_time = now
                    
                cmd = self._execute_search(now)
                
        # Apply limits -> Smooth -> Publish
        cmd = self._apply_safety_limits(cmd)
        smooth_cmd = self._smooth_command(cmd, now)
        self.vel_pub.publish(smooth_cmd)
        
    def _execute_tracking(self, now):
        cmd = Twist()
        
        predicted_pos = self.ekf.get_predicted_position(self.prediction_horizon)
        current_vel = self.ekf.get_velocity()
        
        x_lat = predicted_pos[0]
        z_dist = predicted_pos[2]
        vx = current_vel[2]
        vy = current_vel[0]
        
        # Distance control
        dist_error = z_dist - self.target_distance
        if abs(dist_error) < self.distance_tolerance:
            dist_error = 0.0
            
        vx_body = self.dist_controller.compute(dist_error, vx)
        
        # Lateral control
        lateral_error = -x_lat
        vy_body = self.lat_controller.compute(lateral_error, vy)
        
        # Yaw control
        yaw_error = math.atan2(x_lat, z_dist)
        self.yaw_integral += yaw_error * 0.05
        self.yaw_integral = clamp(self.yaw_integral, -1.0, 1.0)
        yaw_derivative = (yaw_error - self.yaw_prev_error) / 0.05
        self.yaw_prev_error = yaw_error
        
        yaw_rate = (
            self.yaw_kp * yaw_error +
            self.yaw_ki * self.yaw_integral +
            self.yaw_kd * yaw_derivative
        )
        yaw_rate = clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        # Altitude control
        alt_error = self.target_altitude - self.current_altitude
        self.alt_integral += alt_error * 0.05
        self.alt_integral = clamp(self.alt_integral, -1.0, 1.0)
        alt_derivative = (alt_error - self.alt_prev_error) / 0.05
        self.alt_prev_error = alt_error
        
        vz_cmd = (
            self.alt_kp * alt_error +
            self.alt_ki * self.alt_integral +
            self.alt_kd * alt_derivative
        )
        vz_cmd = clamp(vz_cmd, -self.max_vz, self.max_vz)
        
        # Transform body to global frame
        cmd.linear.x = vx_body * math.cos(self.current_yaw) - vy_body * math.sin(self.current_yaw)
        cmd.linear.y = vx_body * math.sin(self.current_yaw) + vy_body * math.cos(self.current_yaw)
        cmd.linear.z = vz_cmd
        cmd.angular.z = yaw_rate
        
        return cmd
        
    def _execute_recovery(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = self._altitude_hold()
        cmd.angular.z = 0.0
        return cmd
        
    def _execute_search(self, now):
        """
        YOUR EXACT CIRCULAR SEARCH ALGORITHM
        """
        cmd = Twist()
        
        dt = now - self.search_start_time
        current_radius = max(0.1, self.initial_radius + (self.spiral_growth * dt))
        target_yaw_rate = (self.search_speed / current_radius) * self.search_yaw_sign
        
        real_forward_speed = self.search_speed * self.fwd_sign
        
        cmd.linear.x = real_forward_speed * math.cos(self.current_yaw)
        cmd.linear.y = real_forward_speed * math.sin(self.current_yaw)
        
        cmd.linear.x = clamp(cmd.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(cmd.linear.y, -self.max_vy, self.max_vy)
        cmd.angular.z = clamp(target_yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        cmd.linear.z = self._altitude_hold()
        
        return cmd

    def _clamp_delta(self, new_value: float, prev_value: float, max_delta: float) -> float:
        return prev_value + clamp(new_value - prev_value, -max_delta, max_delta)

    def _smooth_command(self, cmd: Twist, now: float) -> Twist:
        """
        RESTORED FROM YOUR ORIGINAL CODE
        """
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
        
    def _altitude_hold(self):
        alt_error = self.target_altitude - self.current_altitude
        return clamp(self.alt_kp * alt_error, -self.max_vz, self.max_vz)
        
    def _apply_safety_limits(self, cmd):
        cmd.linear.x = clamp(cmd.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = clamp(cmd.linear.y, -self.max_vy, self.max_vy)
        cmd.linear.z = clamp(cmd.linear.z, -self.max_vz, self.max_vz)
        cmd.angular.z = clamp(cmd.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        return cmd
        
    def log_status(self):
        if self.state == TrackerState.TRACKING:
            pos = self.ekf.get_position()
            vel = self.ekf.get_velocity()
            dist_error = np.linalg.norm(pos) - self.target_distance
            self.get_logger().info(
                f"TRACKING: dist={pos[2]:.1f}m, lat={pos[0]:.1f}m, err={dist_error:.1f}m"
            )
        else:
            self.get_logger().info(f"State: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerPro()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.vel_pub.publish(stop_cmd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()