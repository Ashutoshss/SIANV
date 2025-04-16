import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('reactiveplanner'), 'config', 'planner.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=rviz_config_path),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],  # or True if using simulation
            arguments=['-d', rviz_config_path]
        ),
    ])
