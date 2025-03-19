#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Odom(Node):
    def __init__(self):
        super().__init__('tf_to_odom_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            msg.pose.pose.position.x = trans.transform.translation.x
            msg.pose.pose.position.y = trans.transform.translation.y
            msg.pose.pose.position.z = trans.transform.translation.z
            msg.pose.pose.orientation = trans.transform.rotation

            self.odom_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not get TF: {str(e)}")

def main():
    rclpy.init()
    node = Odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
