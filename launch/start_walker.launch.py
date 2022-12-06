# Copyright 2022, Chang-Hong Chen
# All rights reserved.
#
# Author: Chang-Hong Chen
# Email: longhongc@gmail.com

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    record = LaunchConfiguration('record', default='false')

    start_rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        condition=IfCondition(record)
        )

    pkg_turtlebot3_gazebo = FindPackageShare(
        package='turtlebot3_gazebo').find('turtlebot3_gazebo')

    start_turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py')
        ))

    start_simple_walker = Node(
        package='ros2_turtlebot_practice',
        executable='simple_walker',
        name='simple_walker_node',
        output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(start_rosbag_record)
    ld.add_action(start_turtlebot3_gazebo)
    ld.add_action(start_simple_walker)

    return ld
