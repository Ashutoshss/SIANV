#! /usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Joystick(Node):
    def __init__(self):
        super().__init__('JoystickChaluHai')
        self.scale_linear = 1.0
        self.scale_angular = 1.0

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def reset(self):
        self.scale_linear = 1.0
        self.scale_angular = 1.0

    def joy_callback(self, msg):
        if msg.buttons[9] == 1:
            self.reset()

        if msg.buttons[4] == 1:
            self.scale_linear +=0.5
        if msg.buttons[6] == 1:
            self.scale_linear -=0.5

        if msg.buttons[5] == 1:
            self.scale_angular +=0.5
        if msg.buttons[7] == 1:
            self.scale_angular +=0.5
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.scale_linear
        twist.angular.z = msg.axes[2] * self.scale_angular
        # twist.linear.x = msg.axes[1] 
        # twist.angular.z = msg.axes[0]
        self.publisher.publish(twist)
        

        # print(f"linear.x{msg.axes[0]}  ,  angualr.x{msg.axes[1]}  ,  lin inc{msg.buttons[4]}  ,  ang in{msg.buttons[5]}  ,  lin dec{msg.buttons[6]}  ,  lin ang{msg.buttons[7]}  ,  reset{msg.buttons[9]}")
        

def main():
    rclpy.init(args=sys.argv)

    node = Joystick() 

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()