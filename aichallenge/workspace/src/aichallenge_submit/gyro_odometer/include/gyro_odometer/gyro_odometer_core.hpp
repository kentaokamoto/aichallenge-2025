#ifndef GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"
#include "tier4_autoware_utils/ros/self_pose_listener.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <deque>
#include <memory>
#include <string>

class GyroOdometer : public rclcpp::Node
{
public:
  explicit GyroOdometer(const rclcpp::NodeOptions & options);
  ~GyroOdometer();

private:
  // Callback
  void callbackVehicleTwist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_ptr);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  
  // ★★★★★ここから追加★★★★★
  void onTimer();
  // ★★★★★ここまで追加★★★★★

  // Publisher
  void publishData(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);
  
  // ★★★★★ここから追加★★★★★
  // Calculator
  geometry_msgs::msg::TwistWithCovarianceStamped calculateTwist(
    const std::deque<sensor_msgs::msg::Imu> & imu_deque,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_deque);
  // ★★★★★ここまで追加★★★★★

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    vehicle_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;

  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  // Buffer
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;

  // Parameter
  const std::string output_frame_;
  const double message_timeout_sec_;
  
  // ★★★★★ここから追加★★★★★
  rclcpp::TimerBase::SharedPtr timer_;
  // ★★★★★ここまで追加★★★★★
};

#endif  // GYRO_ODOMETER__GYRO_ODOMETER_CORE_HPP_