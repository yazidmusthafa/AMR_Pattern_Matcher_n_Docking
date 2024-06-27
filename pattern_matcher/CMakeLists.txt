cmake_minimum_required(VERSION 3.5)
project(pattern_matcher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(scan_to_pointcloud_node src/scan_to_pointcloud_node.cpp)
ament_target_dependencies(scan_to_pointcloud_node rclcpp sensor_msgs pcl_conversions pcl_ros PCL nav2_msgs)

add_executable(recorder src/recorder.cpp)
ament_target_dependencies(recorder rclcpp sensor_msgs pcl_conversions pcl_ros PCL nav2_msgs laser_geometry)

install(TARGETS
  scan_to_pointcloud_node
  recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch src urdf meshes pcd config
    DESTINATION share/${PROJECT_NAME})

ament_package()
