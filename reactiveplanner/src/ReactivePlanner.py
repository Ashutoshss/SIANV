#!/usr/bin/env python3

# script: ReactivePlanner
# author: Ashutosh Singh
# usecase: autonomous navigation between two points with collision and object avoidance without pre-built map.

import rclpy
import sys
import time

from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker   # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
from geometry_msgs.msg import Point         # https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html

import math
import numpy as np
import matplotlib.pyplot as plt

class ReactivePlanner(Node):
    def __init__(self):
        super().__init__('planner')
        self.get_logger().info('planner initiated')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Initialize robot_pose to store x, y, and yaw
        self.robot_pose = [0.0, 0.0, 0.0]

        # __________________________________________________-
        # self.target_pos = [-0.590868, -1.324407, 0.007841]      # <--------------------------------- set the targeted position through
        self.target_pos =[4.209856, -1.91521, -0.00726]
        # self.target_pos = [-0.537840, -1.089835, 0.007824]
        self.goal_tolerance = 0.07
        self.angular_speed = 0.77
        self.linear_speed = 0.2
        self.twist = Twist()

        # self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})

        self.safeDistance = 0.50
        self.obstacle_threshold = 0.5
        self.obstacle_detected= False
        self.ranges = None
        self.angles = None
        self.obstacle_indices = None
        self.front_fov_indices = None
        self.left_fov_indices = None
        self.right_fov_indices = None
        processing_rate = 0.1

        self.timer = self.create_timer(processing_rate,self.navigate_to_goal)

    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x  
        self.robot_pose[1] = msg.pose.pose.position.y  

        quaternion = msg.pose.pose.orientation
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

        marker = Marker()
        marker.header.frame_id = "odom"     # odom -> base_footprint -> base_link -> wheel_left_link and wheel_right_link
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [Point(x=self.robot_pose[0], y = self.robot_pose[1], z = 0.0), Point(x=self.target_pos[0], y = self.target_pos[1], z = 0.0)]
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.color.r = 0.0  
        marker.color.g = 1.0  
        marker.color.b = 0.0  
        marker.color.a = 1.0  # Fully opaque
        self.marker_pub.publish(marker)

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)
        self.fakeranges = np.zeros_like(self.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(self.ranges))    # start index , last index , length of the list or array
        self.angles_ = np.round(np.degrees(self.angles))
        # print(self.angles_)
        
        self.front_fov_indices = (self.angles_ <= 45.0) | (self.angles_ >= 315.0)
        self.left_fov_indices = (self.angles_ > 0.0) & (self.angles_ <= 45.0)
        self.right_fov_indices = (self.angles_ >= 315.0) & (self.angles_ < 360.0)

        if np.any(self.ranges[self.front_fov_indices] < self.safeDistance):
            self.obstacle_detected = True
            # print("path obstructed ...")
            self.obstacle_indices = np.where(self.ranges < self.front_fov_indices)[0]
            self.fakeranges[self.obstacle_indices] = self.ranges[self.obstacle_indices]
            # print(self.fakeranges)
            # #plotter params     https://matplotlib.org/stable/gallery/pie_and_polar_charts/polar_demo.html
            # plt.ion()   # interactive mode
            # self.ax.clear()
            # self.ax.set_title("LIDAR Scan")
            # self.ax.plot(self.angles, self.fakeranges, 'r.')  
            # self.ax.set_theta_zero_location("N")  # Set zeroth degree to upward in North Direction
            # self.ax.set_theta_direction(1)  # Set direction to anti-clockwise
            # self.ax.set_rlim(0, max(self.fakeranges))  
            # # self.ax.plot(angle1, self.fakeranges[start], 'bo', label="Angle1")  # Blue for angle1
            # # self.ax.plot(angle2, self.fakeranges[end], 'go', label="Angle2") 
            # plt.pause(0.1)  
            # # print(self.obstacle_indices)

        else:
            self.obstacle_detected = False

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def bypass_nodes(self):
        corner_index = np.where(self.fakeranges==0.0)[0]
        if len(corner_index) > 0:
            start = corner_index[0] -1
            end = corner_index[-1] +1
            angle1 = self.angles_[start]
            angle2 = self.angles_[end]
            # print(f"angle1 : {angle1} , angle2 : {angle2}")
            x1 = self.fakeranges[start] * np.cos(angle1)
            y1 = self.fakeranges[start] * np.sin(angle1)
            x2 = self.fakeranges[end] * np.cos(angle2)
            y2 = self.fakeranges[end] * np.sin(angle2)

            points = [[x1,y1],[x2,y2]]
            angle = [angle1, angle2]
            # print(self.points)
            return points ,angle

        else:
            start = end = None
            return [] ,[]

    def ideal_bypass_node(self,points,angle):
        node1 = points[0]
        node2 = points[1]

        curr2node1, _ = self.euclidean_distance(self.robot_pose, node1)
        curr2node2, _ = self.euclidean_distance(self.robot_pose, node2)
        node2target1, _ = self.euclidean_distance(node1,self.target_pos)
        node2target2, _ = self.euclidean_distance(node2,self.target_pos)
        distance1 = curr2node1 + node2target1
        distance2 = curr2node2 + node2target2

        if distance1 < distance2:
            print("left bypass node is selected")
            return node1, angle[0]
        else:
            print("right bypass node is selected")
            return node2, angle[1]
        
    def go_to_node(self, angle):
        angular_speed = 0.15
        angle = np.radians(angle)
        angle_error = angle - self.robot_pose[2]
        # print(angle_error)
        if abs(angle_error) > 0.01:  
            self.twist.angular.z = angular_speed * angle_error
            self.cmd_pub.publish(self.twist)
        else:
            self.stop_robot()
        
    def euclidean_distance(self,from_coordinates,to_coordinates):
        dx = to_coordinates[0] - from_coordinates[0]
        dy = to_coordinates[1] - from_coordinates[1]

        return math.sqrt(dx**2 + dy**2), math.atan2(dy, dx)
    
    # def curve_path(self, left_obstacle, right_obstacle):
    #     if left_obstacle:
    #         left_distance = np.min(self.ranges[self.left_fov_indices])
    #         self.twist.linear.x = 0.1
    #         if left_distance < self.safeDistance:
    #             self.twist.angular.z = -0.7
    #         else:
    #             self.twist.angular.z = -0.4
    #         front_obstacle = np.any(self.ranges[self.front_fov_indices] < self.safeDistance)
    #         if not front_obstacle:
    #             self.obstacle_detected = False
    #     elif right_obstacle:
    #         right_distance = np.min(self.ranges[self.right_fov_indices])
    #         self.twist.linear.x = 0.1
    #         if right_distance < self.safeDistance:
    #             self.twist.angular.z = 0.7
    #         self.twist.angular.z = 0.4
    #         front_obstacle = np.any(self.ranges[self.front_fov_indices] < self.safeDistance)
    #         if not front_obstacle:
    #             self.obstacle_detected = False
    #     else:
    #         self.twist.linear.x = 0.0
    #         self.twist.angular.z = 0.0
            
    #     self.cmd_pub.publish(self.twist)

    def curve_path(self, left_obstacle, right_obstacle):
        Kp_angular = 4.0
        Kp_linear = 1.0
        max_angular_speed = 0.77  # Limit for angular speed
        max_linear_speed = 4.0  # Limit for linear speed

        if left_obstacle:
            left_distance = np.min(self.ranges[self.left_fov_indices])
            self.twist.linear.x = max(0.03, Kp_linear * (self.safeDistance - left_distance))
            angular_speed = Kp_angular * (self.safeDistance - left_distance)
            self.twist.angular.z = - min(angular_speed, max_angular_speed)
            
            front_obstacle = np.any(self.ranges[self.front_fov_indices] < self.safeDistance)
            if not front_obstacle:
                self.obstacle_detected = False
        elif right_obstacle:
            right_distance = np.min(self.ranges[self.right_fov_indices])
            self.twist.linear.x = max(0.03, Kp_linear * (self.safeDistance - right_distance))
            angular_speed = Kp_angular * (self.safeDistance - right_distance)
            self.twist.angular.z = min(angular_speed, max_angular_speed)

            front_obstacle = np.any(self.ranges[self.front_fov_indices] < self.safeDistance)
            if not front_obstacle:
                self.obstacle_detected = False
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
        self.cmd_pub.publish(self.twist)


    def navigate_to_goal(self):
        distance_to_goal,angle2goal = self.euclidean_distance(self.robot_pose, self.target_pos)
        angle_error = angle2goal - self.robot_pose[2]
        if self.obstacle_detected:
            left_obstacle = np.any(self.ranges[self.left_fov_indices] < self.safeDistance)
            right_obstacle = np.any(self.ranges[self.right_fov_indices] < self.safeDistance)
            # print(f"front: {front_obstacle}")
            # print(f"left: {left_obstacle}")
            # print(f"right: {right_obstacle}")
            if left_obstacle & right_obstacle :
                angular_difference_flag = abs(angle_error) > np.pi / 2
                if angular_difference_flag:
                    self.twist.angular.z = self.angular_speed * angle_error
                    self.cmd_pub.publish(self.twist)
                else:
                    print("Obstacle detected directly in front!")
                    self.stop_robot()
                    points,angle = self.bypass_nodes()
                    if len(points) == 0 & len(angle) == 0:
                        # self.get_logger().warn("No valid bypass nodes found. Stopping the robot.")
                        self.stop_robot()
                        return
                    tatgetNode_pos,angleNode = self.ideal_bypass_node(points, angle)
                    self.go_to_node(angleNode)

            elif left_obstacle:
                print("Obstacle detected in front (left side) navigating through right side!")
                self.stop_robot()
                self.curve_path(left_obstacle, right_obstacle)
                left_obstacle = False

            elif right_obstacle:
                print("Obstacle detected in front (right side) navigating through left side!")
                self.stop_robot()
                self.curve_path(left_obstacle, right_obstacle)
                right_obstacle = False

        else:
            if distance_to_goal > self.goal_tolerance:
                self.get_logger().info("Navigating to the Target...")
                if abs(angle_error) > 0.01:  
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.angular_speed * angle_error  
                else:  # If aligned, move forward
                    self.twist.linear.x = min(self.linear_speed, distance_to_goal) 
                    self.twist.angular.z = 0.0
            else:
                self.get_logger().info("Goal reached!")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            self.cmd_pub.publish(self.twist)


def main():

    rclpy.init(args=sys.argv)       # initialisation

    node = ReactivePlanner()        # object

    rclpy.spin(node)           

    node.destroy_node()

    rclpy.shutdown()        

if __name__ == '__main__':
    main()    
