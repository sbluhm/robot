cmake_minimum_required(VERSION 3.16)
project(bluhmbot LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  bluhmbot
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(bluhmbot PUBLIC cxx_std_17)
target_include_directories(bluhmbot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/bluhmbot>
)

include_directories(/usr/local/src/rpi_pwm)

ament_target_dependencies(
  bluhmbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    gpiod
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bluhmbot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/bluhmbot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf src/description
  DESTINATION share/bluhmbot
)
install(
  DIRECTORY bringup/launch  config launch
  DESTINATION share/bluhmbot
)

install(
  DIRECTORY maps
  DESTINATION share/bluhmbot
)

install(
  DIRECTORY rviz
  DESTINATION share/bluhmbot
)

# Install Python scripts
install(PROGRAMS
  src/teleop/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bluhmbot
  EXPORT export_bluhmbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_bluhmbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
