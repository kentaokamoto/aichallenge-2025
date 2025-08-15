#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

namespace mpc_controller
{

class MPCController : public rclcpp::Node
{
public:
  MPCController();

private:
  void onTimer();
  bool subscribeMessageAvailable();
  void solveMPC();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_cmd_;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr traj_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mpc_controller
