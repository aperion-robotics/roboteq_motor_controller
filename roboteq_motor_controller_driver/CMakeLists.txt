cmake_minimum_required(VERSION 3.16)
project(roboteq_motor_controller_driver LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  serial
  rclcpp
  pluginlib
  realtime_tools
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  transmission_interface
  generate_parameter_library
  roboteq_motor_controller_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(
  roboteq_hardware_interface_parameters # cmake target name for the parameter library
  src/roboteq_hardware_interface_parameters.yaml # path to input yaml file
)

# hardware interface plugin
add_library(roboteq_motor_controller_driver_plugin SHARED
  src/roboteq_hardware_interface.cpp
)
target_include_directories(roboteq_motor_controller_driver_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(roboteq_motor_controller_driver_plugin
  PUBLIC
    roboteq_hardware_interface_parameters
    serial::serial
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    hardware_interface::hardware_interface
    transmission_interface::transmission_interface
  PRIVATE
    pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface roboteq_hardware_interface_plugin.xml)

# broadcaster controller plugin
add_library(roboteq_broadcaster_plugin SHARED
  src/roboteq_broadcaster.cpp
)
target_include_directories(roboteq_broadcaster_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(roboteq_broadcaster_plugin
  PUBLIC
    roboteq_hardware_interface_parameters
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    realtime_tools::realtime_tools
    controller_interface::controller_interface
    ${roboteq_motor_controller_msgs_TARGETS}
  PRIVATE
    pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(controller_interface roboteq_broadcaster_plugin.xml)

# install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    roboteq_motor_controller_driver_plugin
    roboteq_broadcaster_plugin
    roboteq_hardware_interface_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
