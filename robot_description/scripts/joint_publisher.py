#! /usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class FakeJointPublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_publisher')
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('wheel_separation', 0.3)  

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.joint_state = JointState()
        self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_state.position = [0.0, 0.0]  
        self.joint_state.velocity = [0.0, 0.0]

        self.timer = self.create_timer(0.05, self.publish_joint_states)  
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        omega_left = ((linear/2) - (self.wheel_separation / 2) * angular) / self.wheel_radius
        omega_right = ((linear/2) + (self.wheel_separation / 2) * angular) / self.wheel_radius

        self.joint_state.velocity = [omega_left, omega_right]

    def publish_joint_states(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  
        self.last_time = current_time

        # Fake position update (integration)
        self.joint_state.position[0] += self.joint_state.velocity[0] * dt
        self.joint_state.position[1] += self.joint_state.velocity[1] * dt

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

def main():
    rclpy.init(args=sys.argv)

    node = FakeJointPublisher() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()