// Copyright 2015-2019 Autoware Foundation
// (license text omitted for brevity)

#include "gyro_odometer/gyro_odometer_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <fmt/core.h>
#include <Eigen/Dense> // Eigen for matrix operations

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>


// Main calculation logic
geometry_msgs::msg::TwistWithCovarianceStamped GyroOdometer::calculateTwist(
  const std::deque<sensor_msgs::msg::Imu> & imu_deque,
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_deque)
{
  using COV_IDX_XYZRPY = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;

  // Use the latest timestamp
  twist_with_cov.header.stamp = (rclcpp::Time(imu_deque.back().header.stamp) > rclcpp::Time(twist_deque.back().header.stamp)) ?
                                imu_deque.back().header.stamp : twist_deque.back().header.stamp;
  twist_with_cov.header.frame_id = output_frame_;

  // Calculate mean values
  double linear_x_sum = 0.0;
  double angular_z_sum = 0.0;
  double linear_x_cov_sum = 0.0;
  double angular_z_cov_sum = 0.0;

  for (const auto & twist : twist_deque) {
    linear_x_sum += twist.twist.twist.linear.x;
    linear_x_cov_sum += twist.twist.covariance[COV_IDX_XYZRPY::X_X];
  }
  for (const auto & imu : imu_deque) {
    angular_z_sum += imu.angular_velocity.z;
    angular_z_cov_sum += imu.angular_velocity_covariance[8]; // Z*Z
  }

  const double linear_x = linear_x_sum / twist_deque.size();
  const double angular_z = angular_z_sum / imu_deque.size();
  const double linear_x_cov = (twist_deque.size() > 1) ? (linear_x_cov_sum / (twist_deque.size() * twist_deque.size())) : linear_x_cov_sum;
  const double angular_z_cov = (imu_deque.size() > 1) ? (angular_z_cov_sum / (imu_deque.size() * imu_deque.size())) : angular_z_cov_sum;

  // Store results in the message
  twist_with_cov.twist.twist.linear.x = linear_x;
  twist_with_cov.twist.twist.angular.z = angular_z;

  // Set covariance
  twist_with_cov.twist.covariance.fill(0.0);
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] = linear_x_cov;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] = imu_deque.back().angular_velocity_covariance[0];
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] = imu_deque.back().angular_velocity_covariance[4];
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] = angular_z_cov;

  return twist_with_cov;
}

GyroOdometer::GyroOdometer(const rclcpp::NodeOptions & options)
: Node("gyro_odometer", options),
  output_frame_(declare_parameter("base_link", "base_link")),
  message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2))
{
  const auto rv_qos = rclcpp::QoS(rclcpp::KeepLast(100));
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rv_qos,
    std::bind(&GyroOdometer::callbackVehicleTwist, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rv_qos, std::bind(&GyroOdometer::callbackImu, this, std::placeholders::_1));

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rv_qos);
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rv_qos);

  // Timer-based processing
  const double timer_rate = 50.0;
  timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(timer_rate).period(),
    std::bind(&GyroOdometer::onTimer, this));
}

GyroOdometer::~GyroOdometer() {}

void GyroOdometer::onTimer()
{
    const auto current_time = this->now();

    // Remove timed-out messages from queues
    while (!vehicle_twist_queue_.empty() && (current_time - vehicle_twist_queue_.front().header.stamp).seconds() > message_timeout_sec_) {
        vehicle_twist_queue_.pop_front();
    }
    while (!gyro_queue_.empty() && (current_time - gyro_queue_.front().header.stamp).seconds() > message_timeout_sec_) {
        gyro_queue_.pop_front();
    }

    if (vehicle_twist_queue_.empty() || gyro_queue_.empty()) {
        return;
    }

    // Synchronize data
    std::deque<sensor_msgs::msg::Imu> imu_sync_deque;
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_sync_deque;
    
    // Find the latest common timestamp
    rclcpp::Time latest_imu_time = gyro_queue_.back().header.stamp;
    rclcpp::Time latest_twist_time = vehicle_twist_queue_.back().header.stamp;
    rclcpp::Time sync_time = (latest_imu_time < latest_twist_time) ? latest_imu_time : latest_twist_time;

    for(const auto& imu_msg : gyro_queue_){
        if(rclcpp::Time(imu_msg.header.stamp) <= sync_time){
            imu_sync_deque.push_back(imu_msg);
        }
    }
    for(const auto& twist_msg : vehicle_twist_queue_){
        if(rclcpp::Time(twist_msg.header.stamp) <= sync_time){
            twist_sync_deque.push_back(twist_msg);
        }
    }

    if (twist_sync_deque.empty() || imu_sync_deque.empty()) {
        return;
    }

    // Calculate and publish
    const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov = calculateTwist(imu_sync_deque, twist_sync_deque);
    publishData(twist_with_cov);

    // Remove used data from queues
    while(!gyro_queue_.empty() && rclcpp::Time(gyro_queue_.front().header.stamp) <= sync_time){
        gyro_queue_.pop_front();
    }
    while(!vehicle_twist_queue_.empty() && rclcpp::Time(vehicle_twist_queue_.front().header.stamp) <= sync_time){
        vehicle_twist_queue_.pop_front();
    }
}

void GyroOdometer::callbackVehicleTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_ptr)
{
  vehicle_twist_queue_.push_back(*vehicle_twist_ptr);
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(imu_msg_ptr->header.frame_id, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Please publish TF from %s to %s",
      (imu_msg_ptr->header.frame_id).c_str(), output_frame_.c_str());
    return;
  }
  
  sensor_msgs::msg::Imu imu_base_link = *imu_msg_ptr;
  
  // Transform angular velocity
  geometry_msgs::msg::Vector3Stamped angular_velocity, transformed_angular_velocity;
  angular_velocity.header = imu_msg_ptr->header;
  angular_velocity.vector = imu_msg_ptr->angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);
  
  // Transform linear acceleration
  geometry_msgs::msg::Vector3Stamped linear_acceleration, transformed_linear_acceleration;
  linear_acceleration.header = imu_msg_ptr->header;
  linear_acceleration.vector = imu_msg_ptr->linear_acceleration;
  tf2::doTransform(linear_acceleration, transformed_linear_acceleration, *tf_imu2base_ptr);

  // Update imu_base_link message
  imu_base_link.header.frame_id = output_frame_;
  imu_base_link.angular_velocity = transformed_angular_velocity.vector;
  imu_base_link.linear_acceleration = transformed_linear_acceleration.vector;
  // (Note: Covariance transformation is more complex, here we assume it's diagonal and rotation dominates)
  // A proper covariance transformation should be R * C * R^T
  
  gyro_queue_.push_back(imu_base_link);
}

void GyroOdometer::publishData(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = twist_with_cov_raw.header;
  twist.twist = twist_with_cov_raw.twist.twist;

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance = twist_with_cov_raw;
  
  // Clear yaw bias if vehicle is stopped
  const double linear_vel_threshold = 0.1;
  const double angular_vel_threshold = 0.05;
  if (
    std::fabs(twist_with_cov_raw.twist.twist.linear.x) < linear_vel_threshold &&
    std::fabs(twist_with_cov_raw.twist.twist.angular.z) < angular_vel_threshold) 
  {
    twist.twist.angular.z = 0.0;
    twist_with_covariance.twist.twist.angular.z = 0.0;
    twist_with_covariance.twist.covariance[5*6 + 5] *= 10.0;
  }

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}