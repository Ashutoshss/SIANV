cmake_minimum_required(VERSION 3.8)
project(imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(serial REQUIRED)

include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

add_executable(imu_driver src/imu.cpp)

target_link_libraries(imu_driver
  ${serial_LIBRARIES}
)

ament_target_dependencies(imu_driver rclcpp sensor_msgs)

install(TARGETS
  imu_driver
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY 
    scripts
    launch
    DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
