# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""Launch an iris quadcopter in Gazebo with MAVROS (no DDS, no MAVProxy window)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for iris with MAVROS."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")

    # Launch SITL via ardupilot_sitl package (NO sim_vehicle.py, NO MAVProxy window)
    # This is the same approach as the original DDS setup, just without dds_udp.parm
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "model": "json",
            "defaults": (
                str(Path(pkg_ardupilot_sitl) / "config" / "default_params" / "copter.parm")
                + ","
                + str(Path(pkg_ardupilot_gazebo) / "config" / "gazebo-iris-gimbal.parm")
                # NOTE: dds_udp.parm intentionally excluded — using MAVROS instead
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Single MAVROS node connecting via TCP (reliable, no port conflicts)
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        parameters=[
            {
                "fcu_url": "tcp://127.0.0.1:5760",  # Connect to SITL TCP
                "gcs_url": "udp://@127.0.0.1:14550",  # Forward to QGC if needed
                "target_system_id": 1,
                "target_component_id": 1,
                "fcu_protocol": "v2.0",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Robot (spawn in Gazebo, NO SITL here, NO MAVROS here)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_gz_bringup"),
                        "launch",
                        "robots",
                        "iris.launch.py",
                    ]
                ),
            ]
        ),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Gazebo Server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "runway.sdf"}'
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gz_sim_server",
                default_value="true",
                description="Run the Gazebo server.",
            ),
            DeclareLaunchArgument(
                "use_gz_sim_gui",
                default_value="true",
                description="Run the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description="Spawn the robot and start SITL+MAVROS.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
            sitl,      # Launch SITL via ardupilot_sitl package (no MAVProxy)
            mavros,    # Launch MAVROS (TCP mode)
            robot,     # Launch robot (Gazebo spawn only)
            rviz,
        ]
    )