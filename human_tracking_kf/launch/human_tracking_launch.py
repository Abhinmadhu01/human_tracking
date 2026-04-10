from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Detection node
        Node(
            package='detection_model',
            executable='human_detection_script.py',
            name='yolo_tracker_detector',
            parameters=[{
                'input_topic': '/camera/image',
                'detections_topic': '/detections',
                'debug_topic': '/yolo_debug_image',
                'model_variant': 'yolo11n',
                'conf': 0.25,
                'imgsz': 384,
                'use_ekf_smoothing': True,
                'publish_debug_image': True
            }],
            output='screen'
        ),
        
        # Tracking node
        Node(
            package='human_tracking_kf',
            executable='person_tracking.py',
            name='human_tracker_pro',
            parameters=[{
                # Target parameters
                'target_distance': 3.0,
                'target_altitude': 3.0,
                
                # Distance PID (Cascade)
                'dist_outer_kp': 0.6,
                'dist_outer_ki': 0.05,
                'dist_outer_kd': 0.2,
                'dist_inner_kp': 3.0,
                'dist_inner_ki': 0.2,
                'dist_inner_kd': 0.5,
                
                # Lateral PID (Cascade)
                'lat_outer_kp': 0.0010,
                'lat_outer_ki': 0.0001,
                'lat_outer_kd': 0.0005,
                'lat_inner_kp': 2.0,
                'lat_inner_ki': 0.1,
                'lat_inner_kd': 0.3,
                
                # Safety limits
                'max_vx_forward': 1.0,
                'max_vx_reverse': -0.3,  # CRITICAL SAFETY LIMIT
                'max_vy': 0.8,
                'max_yaw_rate': 0.7,
                
                # Timing
                'detection_timeout': 0.5,
                'prediction_horizon': 0.3
            }],
            output='screen'
        )
    ])
