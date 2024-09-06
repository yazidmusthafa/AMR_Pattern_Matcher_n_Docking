cmake_minimum_required(VERSION 3.8)
project(ultrasonic_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_executable(ultrasonic_sensor_node src/ultrasonic_sensor_node.cpp)
ament_target_dependencies(ultrasonic_sensor_node rclcpp custom_msgs nlohmann_json)

target_link_libraries(ultrasonic_sensor_node ${SERIAL_LIBRARIES})

install(TARGETS
  ultrasonic_sensor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    src
    DESTINATION share/${PROJECT_NAME})

ament_package()
