cmake_minimum_required(VERSION 3.5)
project(pattern_matcher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(matcher_action_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(robot_control src/robot_control.cpp)
ament_target_dependencies(robot_control rclcpp sensor_msgs pcl_conversions pcl_ros PCL nav2_msgs tf2 tf2_ros geometry_msgs Eigen3 matcher_action_interfaces)

add_executable(recorder src/recorder.cpp)
ament_target_dependencies(recorder rclcpp sensor_msgs pcl_conversions pcl_ros PCL nav2_msgs laser_geometry)

install(TARGETS
  robot_control
  recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch src urdf meshes pcd config
    DESTINATION share/${PROJECT_NAME})

ament_package()
