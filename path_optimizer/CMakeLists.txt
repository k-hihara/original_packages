cmake_minimum_required(VERSION 3.8)
project(path_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(path_optimizer
  src/path_optimizer.cpp
)

ament_auto_package()