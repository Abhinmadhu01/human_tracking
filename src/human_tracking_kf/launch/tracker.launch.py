"""
tracker.launch.py
------------------
Launch the human_tracking_kf node with parameters loaded from YAML.

Usage
-----
  ros2 launch human_tracking_kf tracker.launch.py

Override a single param at launch time:
  ros2 launch human_tracking_kf tracker.launch.py kf_accel_std:=30.0

Override the entire config file:
  ros2 launch human_tracking_kf tracker.launch.py config_file:=/path/to/my_params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

  pkg_share = get_package_share_directory("human_tracking_kf")
  default_config = os.path.join(pkg_share, "config", "tracker_params.yaml")

  config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the tracker parameter YAML file",
    )
  use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time from /clock",
    )

  tracker_node = Node(
      package    = "human_tracking_kf",
      executable = "person_tracking.py",
        name       = "human_tracker",
        output     = "screen",
        parameters = [
          LaunchConfiguration("config_file"),
          {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        # Remap topics here if your MAVROS or YOLO use different names
        remappings = [
            # ("/yolo/detections",  "/your/yolo/topic"),
            # ("/mavros/state",     "/your/mavros/state"),
        ],
    )

  return LaunchDescription([config_arg, use_sim_time_arg, tracker_node])
