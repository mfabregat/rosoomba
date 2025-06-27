#!/usr/bin/env python3
# Copyright (c) 2023 rosoomba Rrosoomba
# This file is part of the rosoomba rrosoomba project

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument('joy_vel', default_value='/diff_cont/cmd_vel'),
        DeclareLaunchArgument('joy_dev', default_value='0'),
        DeclareLaunchArgument('publish_stamped_twist', default_value='true'),
        DeclareLaunchArgument('config_filepath', default_value=[
            PathJoinSubstitution([
                FindPackageShare("rosoomba_bringup"), 'config', 'joy_config.yaml'
            ])
        ]),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
            
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
            ),
    ])
