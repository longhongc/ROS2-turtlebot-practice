import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo'
        ).find('turtlebot3_gazebo')

    start_turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo,
                'launch', 'turtlebot3_world.launch.py')
        ))

    ld = LaunchDescription()
    ld.add_action(start_turtlebot3_gazebo)

    return ld

