cmake_minimum_required(VERSION 3.8)
project(skyforge_controllers)

# Export compile commands to help IDEs (e.g., VS Code C/C++ extension)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
