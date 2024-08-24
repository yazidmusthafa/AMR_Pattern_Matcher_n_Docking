cmake_minimum_required(VERSION 3.22)
project(diff_drive)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#Added##

add_library(
  diff_drive
  SHARED
  hardware/diff_drive_hardware.cpp
)

target_compile_features(diff_drive PUBLIC cxx_std_17)
target_include_directories(diff_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diff_drive>
)

ament_target_dependencies(
  diff_drive PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

target_link_libraries(diff_drive PUBLIC serial)    #####

pluginlib_export_plugin_description_file(hardware_interface diff_drive.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/diff_drive
)

install(TARGETS diff_drive
  EXPORT export_diff_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#   diff_drive_moveit
#   DESTINATION lib/${PROJECT_NAME}) 

install(
  DIRECTORY config launch src urdf meshes worlds
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_diff_drive HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
