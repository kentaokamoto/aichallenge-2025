cmake_minimum_required(VERSION 3.8)
project(simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_pure_pursuit
  src/simple_pure_pursuit.cpp
)

target_link_libraries(simple_pure_pursuit Eigen3::Eigen)

ament_auto_package()
