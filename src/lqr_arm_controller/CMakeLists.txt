cmake_minimum_required(VERSION 3.8)
project(lqr_arm_controller)

# Export compile commands to help IDEs (e.g., VS Code C/C++ extension)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(lqr_arm_controller SHARED
  src/lqr_arm_controller.cpp)

target_include_directories(lqr_arm_controller PRIVATE include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)

ament_target_dependencies(lqr_arm_controller
  controller_interface
  rclcpp
  pluginlib
  hardware_interface
  realtime_tools
  Eigen3)

pluginlib_export_plugin_description_file(controller_interface lqr_arm_controller.xml)

install(TARGETS lqr_arm_controller
  DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include) 

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
