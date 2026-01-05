cmake_minimum_required(VERSION 3.8)
project(skyforge_controllers)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
