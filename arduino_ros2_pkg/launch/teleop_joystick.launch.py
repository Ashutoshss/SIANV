# robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_ros2_pkg',
            executable='ros2_control',
            name='ros2_control_node',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy_node',
        #     parameters=['/home/singh/robot_ws/src/arduino_ros2_pkg/config/Joystick.yaml'],
        #     output='screen'
        # ),
    ])