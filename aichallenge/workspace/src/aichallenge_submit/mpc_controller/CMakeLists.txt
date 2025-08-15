cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(mpc_controller
  src/mpc_controller.cpp
)

target_link_libraries(mpc_controller
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

ament_auto_package()
