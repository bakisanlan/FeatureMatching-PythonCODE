#!/usr/bin/env python3
"""
Launch RViz2 with odometry path visualization for NED frame.
"""

import os
import math
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    # Get paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, '..', 'config')
    scripts_dir = os.path.join(script_dir, '..', 'scripts')
    
    rviz_config = os.path.join(config_dir, 'odom_ned_visualization.rviz')
    odom_to_path_script = os.path.join(scripts_dir, 'odom_to_path.py')

    # Static TF publisher for world frame
    static_tf_world = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'odom_ned'
        ],
        output='screen'
    )

    # Static TF publisher for NED axes frame (rotated 180° around X to flip Z down)
    # This creates a frame where Z points DOWN for proper NED visualization
    static_tf_ned_axes = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', str(math.pi), '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'ned_axes_frame'
        ],
        output='screen'
    )

    # GT odometry to path converter
    # flip_z=true: NED Down becomes Up for visualization
    # flip_y=true: Flip Y to match NED East direction with rotated axes
    gt_odom_to_path = ExecuteProcess(
        cmd=[
            'python3', odom_to_path_script,
            '--ros-args',
            '-r', '__node:=gt_odom_to_path',
            '-p', 'odom_topic:=/gt/odom_ned',
            '-p', 'path_topic:=/gt/path',
            '-p', 'max_length:=100000',
            '-p', 'flip_z:=true',
            '-p', 'flip_y:=true',
        ],
        output='screen'
    )

    # VIO odometry to path converter
    vio_odom_to_path = ExecuteProcess(
        cmd=[
            'python3', odom_to_path_script,
            '--ros-args',
            '-r', '__node:=vio_odom_to_path',
            '-p', 'odom_topic:=/vio/odom_ned',
            '-p', 'path_topic:=/vio/path',
            '-p', 'max_length:=100000',
            '-p', 'flip_z:=true',
            '-p', 'flip_y:=true',
        ],
        output='screen'
    )

    # RViz2 - launch after a short delay to ensure TF is available
    rviz = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        static_tf_world,
        static_tf_ned_axes,
        gt_odom_to_path,
        vio_odom_to_path,
        rviz,
    ])
