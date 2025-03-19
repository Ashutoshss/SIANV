import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav2_params = LaunchConfiguration('nav2_params')
    map_file = LaunchConfiguration('map')
    rviz_config_file = os.path.join(
        get_package_share_directory('robot_nav2'), 'rviz', 'rviz2_config.rviz')
    
    declare_nav2_params_cmd = DeclareLaunchArgument(
        'nav2_params',
        default_value='/home/singh/robot_ws/src/robot_nav2/params/nav2_params.yaml',
        description='Path to the navigation parameters file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/singh/robot_ws/src/robot_nav2/maps/map.yaml',
        description='Path to the map file')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nav2_bringup"), "/launch", "/bringup_launch.py"
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'map': map_file
        }.items(),
    )
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    
    return LaunchDescription([
        declare_nav2_params_cmd,
        declare_map_cmd,
        nav2_launch,
        start_rviz_cmd
    ])
