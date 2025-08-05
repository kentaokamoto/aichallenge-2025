#ifndef SIMPLE_PURE_PURSUIT_HPP_
#define SIMPLE_PURE_PURSUIT_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include <deque>

namespace simple_pure_pursuit {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

// Vehicle state structure for MPC
struct VehicleState {
  double x;      // x position [m]
  double y;      // y position [m]
  double yaw;    // yaw angle [rad]
  double v;      // velocity [m/s]
  double delta;  // steering angle [rad]
  double a;      // acceleration [m/s^2]
};

// Reference trajectory point
struct ReferencePoint {
  double x;
  double y;
  double yaw;
  double v;
};

class SimplePurePursuit : public rclcpp::Node {
 public:
  explicit SimplePurePursuit();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;

  // Vehicle parameters
  const double wheel_base_;
  const double max_steer_angle_;
  const double max_acceleration_;
  const double min_acceleration_;
  const double max_steer_rate_;
  const double max_velocity_;
  
  // MPC parameters
  const int prediction_horizon_;
  const double dt_;
  const double q_lateral_;
  const double q_longitudinal_;
  const double q_steer_rate_;
  const double q_acceleration_;
  const double r_steer_;
  const double r_acceleration_;
  
  // Control history for MPC
  std::deque<double> steer_history_;
  std::deque<double> acceleration_history_;
  
  // Previous control inputs
  double previous_steer_;
  double previous_acceleration_;

 private:
  void onTimer();
  bool subscribeMessageAvailable();
  
  // MPC control methods
  VehicleState getCurrentVehicleState();
  std::vector<ReferencePoint> getReferenceTrajectory(size_t start_idx, int horizon);
  Eigen::VectorXd solveMPC(const VehicleState& current_state, 
                          const std::vector<ReferencePoint>& reference);
  VehicleState predictVehicleState(const VehicleState& state, double steer, double accel, double dt);
  double normalizeAngle(double angle);
};

}  // namespace simple_pure_pursuit

#endif  // SIMPLE_PURE_PURSUIT_HPP_
