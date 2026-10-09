cmake_minimum_required(VERSION 3.8)
project(roboteq_motor_controller_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(serial REQUIRED)
find_package(roboteq_motor_controller_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

generate_parameter_library(
  roboteq_hardware_interface_parameters # cmake target name for the parameter library
  src/roboteq_hardware_interface_parameters.yaml # path to input yaml file
)

add_library(roboteq_motor_controller_driver_plugin
  SHARED
  src/roboteq_hardware_interface.cpp
)

target_link_libraries(roboteq_motor_controller_driver_plugin
  rclcpp::rclcpp
  roboteq_hardware_interface_parameters
)

target_include_directories(roboteq_motor_controller_driver_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboteq_motor_controller_driver_plugin
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  transmission_interface
  serial
)

pluginlib_export_plugin_description_file(hardware_interface roboteq_hardware_interface_plugin.xml)

add_library(roboteq_broadcaster_plugin
  SHARED
  src/roboteq_broadcaster.cpp
)

target_link_libraries(roboteq_broadcaster_plugin
  rclcpp::rclcpp
  roboteq_hardware_interface_parameters
)

target_include_directories(roboteq_broadcaster_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(roboteq_broadcaster_plugin
  rclcpp
  rclcpp_lifecycle
  controller_interface
  realtime_tools
  roboteq_motor_controller_msgs
)

pluginlib_export_plugin_description_file(controller_interface roboteq_broadcaster_plugin.xml)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include 
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS roboteq_motor_controller_driver_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS roboteq_broadcaster_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(roboteq_motor_controller_driver_plugin)

ament_export_libraries(roboteq_broadcaster_plugin)

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

ament_package()
