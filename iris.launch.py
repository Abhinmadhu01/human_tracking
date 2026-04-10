# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""
Launch an iris quadcopter in Gazebo (Robot only, NO SITL, NO MAVROS).

SITL and MAVROS are launched by the parent launch file (iris_runway.launch.py).
This file only handles:
- robot_state_publisher
- Gazebo spawn
- ROS-Gazebo bridge
"""
from typing import List

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_robot_launch_actions(context: LaunchContext, *args, **kwargs):
    """Launch the robot_state_publisher and ros_gz bridge nodes."""
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Ensure the ArduPilot plugin and SITL have a consistent sim_address
    sim_address = LaunchConfiguration("sim_address").perform(context)
    robot_desc = robot_desc.replace(
        "<fdm_addr>127.0.0.1</fdm_addr>",
        f"<fdm_addr>{sim_address}</fdm_addr>",
    )

    # Publish /tf and /tf_static.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=LaunchConfiguration("robot_name"),
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            LaunchConfiguration("robot_name"),
            "-topic",
            "/robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    # Bridge (ROS-Gazebo)
    config_template_file = os.path.join(
        pkg_project_bringup, "config", "iris_bridge.yaml"
    )
    with open(config_template_file, "r") as infp:
        config = infp.read()

    world_name = LaunchConfiguration("world_name").perform(context)
    config = config.replace("{{ world_name }}", f"{world_name}")

    robot_name = LaunchConfiguration("robot_name").perform(context)
    config = config.replace("{{ robot_name }}", f"{robot_name}")

    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    config_file = temp_file.name
    with open(config_file, "w") as outfp:
        outfp.write(config)

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": config_file,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay for Gazebo TF
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    on_robot_state_publisher_start = RegisterEventHandler(
        OnProcessStart(target_action=robot_state_publisher, on_start=[spawn_robot])
    )

    on_bridge_start = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    return [
        robot_state_publisher,
        bridge,
        on_robot_state_publisher_start,
        on_bridge_start,
    ]


def generate_launch_description():
    """Generate a launch description for a iris quadcopter (Robot only)."""
    launch_arguments = generate_launch_arguments()

    # Ensure `SDF_PATH` is populated
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    robot_launch_actions = OpaqueFunction(function=generate_robot_launch_actions)

    return LaunchDescription(
        launch_arguments
        + [
            robot_launch_actions,
        ]
    )


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
    # FIX: Removed unused pkg_ardupilot_sitl and pkg_ardupilot_gazebo fetches
    # that were leftover from the original DDS setup. These are only needed
    # inside generate_robot_launch_actions() which fetches them locally.
    return [
        DeclareLaunchArgument(
            "sim_address",
            default_value="127.0.0.1",
            description="Address for Gazebo-ArduPilot plugin.",
        ),
        DeclareLaunchArgument(
            "use_gz_tf",
            default_value="true",
            description="Use Gazebo TF.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="runway",
            description="Name for the world instance.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="iris",
            description="Name for the model instance.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="The initial 'x' position (m).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="The initial 'y' position (m).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.2",
            description="The initial 'z' position (m).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0.0",
            description="The initial roll angle (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0.0",
            description="The initial pitch angle (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value=f"{math.radians(90)}",
            description="The initial yaw angle (radians).",
        ),
    ]