cmake_minimum_required(VERSION 3.5)
project(imu_gnss_poser)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# ★★★★★ここから追加★★★★★
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# ★★★★★ここまで追加★★★★★

add_executable(imu_gnss_poser_node src/imu_gnss_poser_node.cpp)

ament_target_dependencies(
    imu_gnss_poser_node
    "rclcpp"
    "geometry_msgs"
    "sensor_msgs"
    # ★★★★★ここから追加★★★★★
    "tf2"
    "tf2_ros"
    # ★★★★★ここまで追加★★★★★
)

install(
    TARGETS imu_gnss_poser_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()