#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  speed_kp_(declare_parameter<float>("speed_kp", 1.0)),
  speed_ki_(declare_parameter<float>("speed_ki", 0.0)),
  speed_kd_(declare_parameter<float>("speed_kd", 0.0)),
  steer_kp_(declare_parameter<float>("steer_kp", 1.0)),
  steer_ki_(declare_parameter<float>("steer_ki", 0.0)),
  steer_kd_(declare_parameter<float>("steer_kd", 0.0)),
  speed_integral_(0.0),
  speed_prev_error_(0.0),
  steer_integral_(0.0),
  steer_prev_error_(0.0),
  prev_time_(now())
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
  sub_steering_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1,
    [this](const SteeringReport::SharedPtr msg) { steering_status_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  rclcpp::Time now = get_clock()->now();
  double dt = (now - prev_time_).seconds();
  if (dt <= 0.0) dt = 1e-3; // 0割防止

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(now);

  // if (
  //   (closet_traj_point_idx == trajectory_->points.size() - 1) ||
  //   (trajectory_->points.size() <= 2)) {
  //   cmd.longitudinal.speed = 0.0;
  //   cmd.longitudinal.acceleration = -10.0;
  //   RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  // } else {
    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

    // calc longitudinal speed and acceleration
    double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
    double current_longitudinal_vel = odometry_->twist.twist.linear.x;

    // --- 速度PID ---
    double speed_error = target_longitudinal_vel - current_longitudinal_vel;
    speed_integral_ += speed_error * dt;
    double speed_derivative = (speed_error - speed_prev_error_) / dt;
    double accel_cmd = speed_kp_ * speed_error + speed_ki_ * speed_integral_ + speed_kd_ * speed_derivative;
    speed_prev_error_ = speed_error;

    cmd.longitudinal.speed = target_longitudinal_vel;
    cmd.longitudinal.acceleration = accel_cmd;

    // --- 操舵PID ---
    //// calc lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
    //// calc center coordinate of rear wheel
    double rear_x = odometry_->pose.pose.position.x -
                    wheel_base_ / 2.0 * std::cos(odometry_->pose.pose.orientation.z);
    double rear_y = odometry_->pose.pose.position.y -
                    wheel_base_ / 2.0 * std::sin(odometry_->pose.pose.orientation.z);
    //// search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
               lookahead_distance;
      });
    // if (lookahead_point_itr == trajectory_->points.end()) {
    //   lookahead_point_itr = trajectory_->points.end() - 1;
    // }e
    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;

    geometry_msgs::msg::PointStamped lookahead_point_msg;
    lookahead_point_msg.header.stamp = get_clock()->now();
    lookahead_point_msg.header.frame_id = "map";
    lookahead_point_msg.point.x = lookahead_point_x;
    lookahead_point_msg.point.y = lookahead_point_y;
    lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
    pub_lookahead_point_->publish(lookahead_point_msg);

    // calc steering angle for lateral control
    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                   tf2::getYaw(odometry_->pose.pose.orientation);
    double steer_target = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);

    // 現在の操舵角を取得
    double current_steer_angle = 0.0;
    if (steering_status_) {
      current_steer_angle = steering_status_->steering_tire_angle;
    }

    double steer_cmd = 0.0;  // ここで宣言

    // PID制御
    if(current_longitudinal_vel > 1.0) {
      // 速度が出ている場合のみ操舵のPID制御を行う
      double steer_error = steer_target - current_steer_angle;
      steer_integral_ += steer_error * dt;
      double steer_derivative = (steer_error - steer_prev_error_) / dt;
      steer_cmd = steer_kp_ * steer_error + steer_ki_ * steer_integral_ + steer_kd_ * steer_derivative;
      steer_prev_error_ = steer_error;
    } else {
      // 速度が低い場合は操舵角を0にする
      steer_integral_ = 0.0;
      steer_prev_error_ = 0.0;
      steer_cmd = 0.0;
    }

    cmd.lateral.steering_tire_angle = steer_cmd;

    // ...lookahead_pointのpublish...
  // }
  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);

  prev_time_ = now;
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
