cmake_minimum_required(VERSION 3.8)
project(arduino_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(serial REQUIRED)

include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

add_executable(ros2_control src/main.cpp)

target_link_libraries(ros2_control
  ${serial_LIBRARIES}
)

ament_target_dependencies(ros2_control rclcpp geometry_msgs)

install(TARGETS
  ros2_control
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
  scripts/joystick_telop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
