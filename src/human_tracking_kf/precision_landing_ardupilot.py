#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from apriltag_msgs.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode
import time
import math

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')

        # ================= CONFIGURATION =================
        # PID GAINS
        self.KP_STRAFE = 0.008       
        self.KP_FWD = 0.003          
        self.KP_YAW = 0.005          
        self.KD_STRAFE = 0.004       
        self.KD_FWD = 0.002          
        self.KD_YAW = 0.002          
        self.KI_XY = 0.0005          # Integral gain (small to avoid windup)
        
        # CAMERA & TAG SETTINGS
        self.CX, self.CY = 320.0, 240.0  
        self.SIGN_X, self.SIGN_Y = 1.0, 1.0
        
        # LANDING LOGIC
        self.LAND_ALT_TRIGGER = 0.3       
        self.FORCED_LAND_ALT = 1.5        
        self.FORCED_LAND_TIMEOUT = 8.0    
        self.CENTER_TOLERANCE_PX = 35     
        self.YAW_TOLERANCE_DEG = 5.0      
        self.MAX_SPEED_MPS = 0.35         
        self.FINAL_APPROACH_ALT = 2.0     
        self.FINAL_GAIN_MULTIPLIER = 0.6  

        # Tag Weights
        self.TARGET_TAGS = {0: 1.0, 1: 1.0, 2: 1.0}

        # ================= STATE VARIABLES =================
        self.state = "WAITING"
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.altitude_received = False
        self.landing_engaged = False
        self.land_mode_future = None
        
        self.tag_visible = False
        self.last_sight_time = 0.0
        self.current_tag_id = -1
        self.err_x, self.err_y, self.err_yaw = 0.0, 0.0, 0.0
        self.prev_err_x, self.prev_err_y, self.prev_err_yaw = 0.0, 0.0, 0.0
        self.int_err_x, self.int_err_y = 0.0, 0.0
        self.last_control_time = 0.0
        self.low_alt_start_time = None

        # ================= ROS 2 SETUP =================
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, qos_sensor)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_sensor)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_sensor)
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.log_status)

        self.get_logger().info("🚀 PID Precision Landing Node Online")

    def log_status(self):
        """Low-frequency status logger"""
        if self.tag_visible:
            self.get_logger().info(
                f"[{self.state}] Tag:{self.current_tag_id} Alt:{self.current_alt:.2f}m Err:{self.err_x:.0f},{self.err_y:.0f}"
            )
        elif self.state != "WAITING":
            self.get_logger().warn(f"[{self.state}] NO TAG DETECTED - SEARCHING...")

    def pose_callback(self, msg):
        """Extract Yaw from Compass/IMU"""
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def rel_alt_callback(self, msg):
        """Update current altitude"""
        self.current_alt = msg.data
        self.altitude_received = True

    def tag_callback(self, msg):
        """Process AprilTag detections"""
        detected = [t for t in msg.detections if t.id in self.TARGET_TAGS]
        if not detected:
            self.tag_visible = False
            return

        self.tag_visible = True
        self.last_sight_time = time.time()
        
        # Select largest tag (best SNR at distance)
        best_tag = max(detected, key=lambda t: len(t.corners))
        self.current_tag_id = best_tag.id
        
        self.err_x = (self.CX - best_tag.centre.x) * self.SIGN_X
        self.err_y = (self.CY - best_tag.centre.y) * self.SIGN_Y
        
        if len(best_tag.corners) >= 2:
            dx = best_tag.corners[1].x - best_tag.corners[0].x
            dy = best_tag.corners[1].y - best_tag.corners[0].y
            if math.hypot(dx, dy) > 20:  # Sanity check
                self.err_yaw = math.degrees(math.atan2(dy, dx))

    def control_loop(self):
        """PID Control Loop with Forced Landing Fallback"""
        if self.landing_engaged: 
            return 

        cmd = Twist()
        now = time.time()
        dt = now - self.last_control_time if self.last_control_time > 0 else 0.05
        self.last_control_time = now
        
        # Timeout check
        if not self.tag_visible or (now - self.last_sight_time > 0.5):
            self.handle_lost_behavior(cmd, now)
            return

        # PID calculations with conditional integration
        d_err_x = (self.err_x - self.prev_err_x) / dt
        d_err_y = (self.err_y - self.prev_err_y) / dt
        
        # Only integrate when error is small (prevents windup during approach)
        if abs(self.err_x) < 100:
            self.int_err_x += self.err_x * dt
        if abs(self.err_y) < 100:
            self.int_err_y += self.err_y * dt
            
        # Anti-windup clamp
        self.int_err_x = max(min(self.int_err_x, 5.0), -5.0)
        self.int_err_y = max(min(self.int_err_y, 5.0), -5.0)

        # Gain multiplier for final approach
        gain_mult = 1.0
        if self.altitude_received and self.current_alt < self.FINAL_APPROACH_ALT:
            gain_mult = self.FINAL_GAIN_MULTIPLIER

        # PID velocity calculation
        body_vx = (self.err_y * self.KP_FWD + d_err_y * self.KD_FWD + self.int_err_y * self.KI_XY) * gain_mult
        body_vy = (self.err_x * self.KP_STRAFE + d_err_x * self.KD_STRAFE + self.int_err_x * self.KI_XY) * gain_mult
        
        self.prev_err_x, self.prev_err_y = self.err_x, self.err_y

        # State machine
        if self.state == "WAITING":
            self.state = "APPROACH"
            self.get_logger().info(f"Target Acquired (Tag {self.current_tag_id}).")

        if self.state == "APPROACH":
            # Only apply yaw correction when centered enough
            if math.hypot(self.err_x, self.err_y) < 120:
                d_err_yaw = (self.err_yaw - self.prev_err_yaw) / dt
                cmd.angular.z = (-self.err_yaw * self.KP_YAW - d_err_yaw * self.KD_YAW) * gain_mult
                self.prev_err_yaw = self.err_yaw
            
            if math.hypot(self.err_x, self.err_y) < self.CENTER_TOLERANCE_PX:
                self.state = "FINAL_APPROACH"
                self.prev_err_x, self.prev_err_y = self.err_x, self.err_y
                self.int_err_x, self.int_err_y = 0.0, 0.0  # Reset integrator
                self.get_logger().info("Alignment Lock. Descending.")

        elif self.state == "FINAL_APPROACH":
            # Altitude-dependent descent rate
            if self.current_alt > 5.0:
                cmd.linear.z = -0.6
            elif self.current_alt > 3.0:
                cmd.linear.z = -0.4
            elif self.current_alt > 1.0:
                cmd.linear.z = -0.25
            else:
                cmd.linear.z = -0.15
                
            cmd.angular.z = 0.0

            # Forced landing timeout
            if self.altitude_received and self.current_alt < self.FORCED_LAND_ALT:
                if self.low_alt_start_time is None:
                    self.low_alt_start_time = now
                elif (now - self.low_alt_start_time) > self.FORCED_LAND_TIMEOUT:
                    self.get_logger().warn(f"⚠️ Timeout at {self.current_alt:.2f}m. Forcing LAND!")
                    self.trigger_auto_land()
                    return
            else:
                self.low_alt_start_time = None

            # Re-acquire check - if tag drifts too far, go back to APPROACH
            if math.hypot(self.err_x, self.err_y) > self.CENTER_TOLERANCE_PX * 2:
                self.state = "APPROACH"
                self.low_alt_start_time = None
                self.get_logger().warn("Tag drifted. Re-centering...")

            # Normal land trigger
            if self.altitude_received and self.current_alt < self.LAND_ALT_TRIGGER:
                self.trigger_auto_land()
                return

        # Transform body to local frame and clamp speed
        cmd.linear.x = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
        cmd.linear.y = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
        
        speed = math.hypot(cmd.linear.x, cmd.linear.y)
        if speed > self.MAX_SPEED_MPS:
            scale = self.MAX_SPEED_MPS / speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale
        
        self.vel_pub.publish(cmd)

    def handle_lost_behavior(self, cmd, now):
        """Smart coasting behavior when tag is lost"""
        if self.state == "FINAL_APPROACH" and self.altitude_received and self.current_alt < self.FORCED_LAND_ALT:
            self.get_logger().warn("Tag lost in final. Coasting...")
            cmd.linear.z = -0.15
            
            # Use last known error with reduced gain
            gain_mult = self.FINAL_GAIN_MULTIPLIER * 0.5
            body_vx = self.err_y * self.KP_FWD * gain_mult
            body_vy = self.err_x * self.KP_STRAFE * gain_mult
            
            cmd.linear.x = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
            cmd.linear.y = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
            
            if self.low_alt_start_time is None:
                self.low_alt_start_time = now
            elif (now - self.low_alt_start_time) > self.FORCED_LAND_TIMEOUT:
                self.get_logger().warn("⚠️ Tag lost timeout. Forcing LAND!")
                self.trigger_auto_land()
        else:
            cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0
            if self.state != "WAITING":
                self.get_logger().warn("Tag Lost. Braking.")
                self.state = "WAITING"
                self.low_alt_start_time = None
        
        self.vel_pub.publish(cmd)

    def trigger_auto_land(self):
        """Switch to ArduPilot LAND mode"""
        if self.landing_engaged:
            return
        self.get_logger().info(f"⬇️ Final Touchdown at {self.current_alt:.2f}m")
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        self.land_mode_future = self.set_mode_client.call_async(req)
        self.land_mode_future.add_done_callback(self.land_mode_result)
        self.landing_engaged = True

    def land_mode_result(self, future):
        """Handle LAND mode service response"""
        try:
            if future.result().mode_sent:
                self.get_logger().info("✅ LAND mode confirmed")
            else:
                self.get_logger().error("❌ LAND mode rejected")
                self.landing_engaged = False
        except Exception as e:
            self.get_logger().error(f"Service failed: {e}")
            self.landing_engaged = False

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
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