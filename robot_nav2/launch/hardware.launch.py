import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths to individual launch files
    ydlidar_launch_file = PathJoinSubstitution([
        get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py'
    ])

    cartographer_launch_file = PathJoinSubstitution([
        get_package_share_directory('cartographer_ros2'), 'launch', 'cartographer_localization.launch.py'
    ])

    robot_description_launch_file = PathJoinSubstitution([
        get_package_share_directory('robot_description'), 'launch', 'robot_description_launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_description_launch_file)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ydlidar_launch_file)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cartographer_launch_file)),
        # Node(
        #     package='nav2',
        #     executable ='odometry_publisher',
        #     name ='odometry_publisher',
        #     output ='screen'
        # ),
        Node(
            package='arduino_ros2_pkg',
            executable='ros2_control',
            name='ros2_control',
            output='screen'
        ),
    ])
