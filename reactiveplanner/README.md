# Reactive Planner for TurtleBot3

## **Overview**
This Package contain the script implements a Reactive Planner for autonomous robot navigation between two points in an environment with dynamic collision avoidance and obstacle bypassing, without relying on a pre-built map. Below is a detailed explanation of the code.

---


## **Features**
- Straight-line navigation between points.
- Real-time obstacle avoidance using LiDAR.
- Completely self-implemented path planning algorithm.

---

## **Setup Instructions**

### **Steps to Set Up**
1. Clone this src into your ROS 2 workspace
2. add few lines to your **`.bashrc`**
   ```bash
   echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
   echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
   echo "source ~/colcon_ws/install/setup.bash    # <----- Your workspace name" >> ~/.bashrc
   ```
3. Build the workspace with updated packages:
   ```bash
   cd ~/colcon_ws
   colcon build
   # After the package has been successfully built,source it
   source install/setup.bash
   ```
### **Execution Steps**
1. launch the gazebo simulation
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
2. launch rviz2 visualizer
   ```bash
   ros2 launch reactiveplanner rviz2.launch.py 
   ```
3. run the Reactive Planner for TurtleBot3
   ```bash
   ros2 run reactiveplanner ReactivePlanner.py 
   ```
---

## **Algorithm Explanation**

#### The planner follows these steps:
1. Accepts start (A) and goal (B) coordinates.
2. Computes a straight-line path between A and B.
3. Continuously monitors obstacles using LiDAR data.
4. Adjusts the robot's trajectory in real-time to avoid collisions while moving toward the goal.

---

## **Dependencies**

- ROS 2 Humble
- TurtleBot3 packages
- Python libraries: numpy, rclpy

---
### Package Structure
```bash
reactiveplanner/
├── CMakeLists.txt
├── config
│   └── planner.rviz
├── include
│   └── reactiveplanner
├── launch
│   └── rviz2.launch.py
├── video_demo
│   └── planner_working.mp4
├── package.xml
├── params
│   └── planner_params.yaml
└── src
    └── ReactivePlanner.py

```




