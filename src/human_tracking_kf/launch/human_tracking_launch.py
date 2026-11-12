from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    spawn_arg = DeclareLaunchArgument('spawn_actor', default_value='false', description='Spawn walking actor in Gazebo')
    actor_file_arg = DeclareLaunchArgument('actor_sdf', default_value='', description='Path to actor SDF file')

    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/camera/image', description='Input camera topic')
    detections_topic_arg = DeclareLaunchArgument('detections_topic', default_value='/detections', description='YOLO detections output topic')
    tracked_topic_arg = DeclareLaunchArgument('tracked_detections_topic', default_value='/tracked_detections', description='ByteTrack output topic')
    tracker_input_topic_arg = DeclareLaunchArgument(
        'tracker_input_topic',
        default_value='/tracked_detections',
        description='Input topic for person tracker (use /detections to bypass ByteTrack)'
    )
    use_bytetrack_arg = DeclareLaunchArgument('use_bytetrack', default_value='true', description='Enable ByteTrack node')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time from /clock')
    debug_arg = DeclareLaunchArgument('publish_debug_image', default_value='false', description='Publish debug image from YOLO node')
    save_detections_arg = DeclareLaunchArgument('save_detections', default_value='false', description='Save debug frames to disk')
    fp_save_dir_arg = DeclareLaunchArgument('fp_save_dir', default_value='', description='Directory for saved debug frames (empty = auto)')
    bytetrack_rate_arg = DeclareLaunchArgument('bytetrack_frame_rate', default_value='20.0', description='Expected detection frame rate for ByteTrack')

    spawn_cfg = LaunchConfiguration('spawn_actor')
    actor_file = LaunchConfiguration('actor_sdf')
    camera_topic = LaunchConfiguration('camera_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    tracked_topic = LaunchConfiguration('tracked_detections_topic')
    tracker_input_topic = LaunchConfiguration('tracker_input_topic')
    use_bytetrack = LaunchConfiguration('use_bytetrack')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_debug = LaunchConfiguration('publish_debug_image')
    save_detections = LaunchConfiguration('save_detections')
    fp_save_dir = LaunchConfiguration('fp_save_dir')
    bytetrack_rate = LaunchConfiguration('bytetrack_frame_rate')

    return LaunchDescription([
        spawn_arg,
        actor_file_arg,
        camera_topic_arg,
        detections_topic_arg,
        tracked_topic_arg,
        tracker_input_topic_arg,
        use_bytetrack_arg,
        use_sim_time_arg,
        debug_arg,
        save_detections_arg,
        fp_save_dir_arg,
        bytetrack_rate_arg,

        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file', actor_file, '-entity', 'walking_man_circle'],
            output='screen',
            condition=IfCondition(spawn_cfg),
        ),

        Node(
            package='detection_model',
            executable='human_detection_script.py',
            name='yolo_tracker_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': camera_topic,
                'detections_topic': detections_topic,
                'debug_topic': '/yolo_debug_image',
                'model_variant': 'yolo11m',
                'conf': 0.6,
                'imgsz': 384,
                'publish_debug_image': publish_debug,
                'save_detections': save_detections,
                'fp_save_dir': fp_save_dir,
            }],
            output='screen'
        ),

        Node(
            package='human_tracking_kf',
            executable='bytetrack_tracker.py',
            name='bytetrack_tracker',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': detections_topic,
                'output_topic': tracked_topic,
                'frame_rate': bytetrack_rate,
                'track_thresh': 0.5,
                'track_buffer': 30,
                'match_thresh': 0.8,
                'min_track_age': 3,
                'max_track_age': 30,
                'publish_debug_image': False,
                'debug_image_topic': '/bytetrack_debug'
            }],
            output='screen',
            condition=IfCondition(use_bytetrack)
        ),

        Node(
            package='human_tracking_kf',
            executable='person_tracking.py',
            name='human_tracker_pro',
            parameters=[{
                'use_sim_time': use_sim_time,
                'detections_topic': tracker_input_topic,
                'target_class': 'person'
            }],
            output='screen'
        )
    ])
