#!/bin/bash

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/singh/robot_ws/src/robot_nav2/maps/map.yaml

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=/home/singh/robot_ws/src/robot_nav2/maps/map.yaml
