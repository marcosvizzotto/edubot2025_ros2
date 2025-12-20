cmake_minimum_required(VERSION 3.8)
project(bluepill_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(bluepill_ros2_control SHARED
  src/bluepill_system.cpp
)

target_include_directories(bluepill_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bluepill_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Exporta a descrição do plugin (pluginlib)
pluginlib_export_plugin_description_file(hardware_interface bluepill_ros2_control.xml)

install(
  TARGETS bluepill_ros2_control
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES bluepill_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
