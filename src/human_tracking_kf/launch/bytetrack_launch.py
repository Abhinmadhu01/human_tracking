import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    spawn_arg = DeclareLaunchArgument(
        'spawn_actor',
        default_value='false',
        description='Spawn walking actor in Gazebo'
    )
    actor_file_arg = DeclareLaunchArgument(
        'actor_sdf',
        default_value='',
        description='Path to actor SDF file'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image',
        description='Input camera topic'
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/detections',
        description='YOLO detections output topic'
    )
    tracked_topic_arg = DeclareLaunchArgument(
        'tracked_detections_topic',
        default_value='/tracked_detections',
        description='ByteTrack output topic'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from /clock'
    )
    debug_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='false',
        description='Publish debug image from the YOLO node'
    )
    save_detections_arg = DeclareLaunchArgument(
        'save_detections',
        default_value='false',
        description='Save debug frames to disk'
    )
    fp_save_dir_arg = DeclareLaunchArgument(
        'fp_save_dir',
        default_value='',
        description='Directory for saved debug frames (empty = auto)'
    )
    bytetrack_rate_arg = DeclareLaunchArgument(
        'bytetrack_frame_rate',
        default_value='20.0',
        description='Expected detection frame rate for ByteTrack'
    )
    tracker_config_default = os.path.join(
        get_package_share_directory('human_tracking_kf'),
        'config',
        'tracker_params.yaml'
    )
    tracker_config_arg = DeclareLaunchArgument(
        'tracker_config_file',
        default_value=tracker_config_default,
        description='Path to tracker parameter YAML file'
    )
    # Override with: ros2 launch ... custom_model_path:=/path/to/your/model
    custom_model_path_default = os.path.join(
        get_package_share_directory('detection_model'),
        'model',
        'yolo11n_base.rpk'
    )
    custom_model_path_arg = DeclareLaunchArgument(
        'custom_model_path',
        default_value=custom_model_path_default,
        description='Path to custom YOLO model (empty to use model_variant auto-resolution)'
    )
    
    spawn_cfg = LaunchConfiguration('spawn_actor')
    actor_file = LaunchConfiguration('actor_sdf')

    camera_topic = LaunchConfiguration('camera_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    tracked_topic = LaunchConfiguration('tracked_detections_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_debug = LaunchConfiguration('publish_debug_image')
    save_detections = LaunchConfiguration('save_detections')
    fp_save_dir = LaunchConfiguration('fp_save_dir')
    bytetrack_rate = LaunchConfiguration('bytetrack_frame_rate')
    tracker_config_file = LaunchConfiguration('tracker_config_file')
    custom_model_path = LaunchConfiguration('custom_model_path')

    return LaunchDescription([
        spawn_arg,
        actor_file_arg,
        camera_topic_arg,
        detections_topic_arg,
        tracked_topic_arg,
        use_sim_time_arg,
        debug_arg,
        save_detections_arg,
        fp_save_dir_arg,
        bytetrack_rate_arg,
        tracker_config_arg,
        custom_model_path_arg,
        
        # Spawn walking actor in Gazebo (optional)
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file', actor_file, '-entity', 'walking_man_circle'],
            output='screen',
            condition=IfCondition(spawn_cfg),
        ),

        # Camera detection node is handled separately by the camera_start_container
        # to avoid "Device or resource busy" hardware conflicts.
        
        # ByteTrack Multi-Object Tracker
        Node(
            package='human_tracking_kf',
            executable='bytetrack_tracker.py',
            name='bytetrack_tracker',
            parameters=[
                tracker_config_file,
                {
                    'use_sim_time': use_sim_time,
                    'input_topic': detections_topic,
                    'output_topic': tracked_topic,
                    'debug_image_topic': '/bytetrack_debug'
                }
            ],
            output='screen'
        ),

        Node(
            package='human_tracking_kf',
            executable='person_tracking.py',
            name='human_tracker_pro',
            parameters=[
                tracker_config_file,
                {
                    'use_sim_time': use_sim_time,
                    'detections_topic': tracked_topic,
                    'target_class': 'person'
                }
            ],
            output='screen'
        ),
    ])
