cmake_minimum_required(VERSION 3.8)
project(chimple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(chimple_pure_pursuit
  src/chimple_pure_pursuit.cpp
)

ament_auto_package()