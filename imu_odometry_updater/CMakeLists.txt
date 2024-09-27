cmake_minimum_required(VERSION 3.8)
project(imu_odometry_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_executable(imu_odometry src/imu_odometry.cpp)
ament_target_dependencies(imu_odometry rclcpp nav_msgs tf2 tf2_ros)

target_link_libraries(imu_odometry ${SERIAL_LIBRARIES})

install(TARGETS
  imu_odometry
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    src
    DESTINATION share/${PROJECT_NAME})

ament_package()
