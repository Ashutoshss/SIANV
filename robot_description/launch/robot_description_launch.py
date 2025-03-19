import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():

    # Declare the 'use_sim_time' argument, defaulting to 'false'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file from the Xacro file
    pkg_path = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_path, 'models', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a dictionary of parameters to pass to the node
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    # Node to run the robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #path od the rviz2----> '/home/ashutosh/.rviz2/default.rviz'
    rviz_file_name = "default.rviz"  # Ensure this file exists in your package or absolute path
    rviz_config_file = os.path.join(pkg_path, 'rviz', rviz_file_name)  # Adjust path as needed
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch all nodes
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        robot_state_publisher,
        # rviz_node  
    ])
