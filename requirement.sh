#!/bin/bash

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/singh/robot_ws/src/robot_nav2/maps/map.yaml

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=/home/singh/robot_ws/src/robot_nav2/maps/map.yaml

<<<<<<< HEAD
ros2 run nav2_map_server map_saver_cli -t carto_map -f college_map
=======
ros2 run nav2_map_server map_saver_cli -f my_map
>>>>>>> 9fc77141a9ac0f3a0658e16bb704c015111cba70
