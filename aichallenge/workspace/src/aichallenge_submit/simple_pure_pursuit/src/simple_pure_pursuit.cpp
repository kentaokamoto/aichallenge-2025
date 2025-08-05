#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <Eigen/Dense>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // Vehicle parameters
  wheel_base_(declare_parameter<float>("wheel_base", 1.087)),
  max_steer_angle_(declare_parameter<float>("max_steer_angle", 0.64)),
  max_acceleration_(declare_parameter<float>("max_acceleration", 3.0)),
  min_acceleration_(declare_parameter<float>("min_acceleration", -3.0)),
  max_steer_rate_(declare_parameter<float>("max_steer_rate", 0.5)),
  max_velocity_(declare_parameter<float>("max_velocity", 15.0)),
  
  // MPC parameters
  prediction_horizon_(declare_parameter<int>("prediction_horizon", 10)),
  dt_(declare_parameter<float>("dt", 0.1)),
  q_lateral_(declare_parameter<float>("q_lateral", 5.0)),
  q_longitudinal_(declare_parameter<float>("q_longitudinal", 2.0)),
  q_steer_rate_(declare_parameter<float>("q_steer_rate", 1.0)),
  q_acceleration_(declare_parameter<float>("q_acceleration", 1.0)),
  r_steer_(declare_parameter<float>("r_steer", 0.5)),
  r_acceleration_(declare_parameter<float>("r_acceleration", 0.5)),
  
  previous_steer_(0.0),
  previous_acceleration_(0.0)
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

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

  size_t closest_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  if (
    (closest_traj_point_idx == trajectory_->points.size() - 1) ||
    (trajectory_->points.size() <= 2)) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    // Get current vehicle state
    VehicleState current_state = getCurrentVehicleState();
    
    // Get reference trajectory
    std::vector<ReferencePoint> reference = getReferenceTrajectory(closest_traj_point_idx, prediction_horizon_);
    
    // Debug: Check reference trajectory
    double curvature = 0.0;
    if (closest_traj_point_idx + 5 < trajectory_->points.size()) {
      double x1 = trajectory_->points[closest_traj_point_idx].pose.position.x;
      double y1 = trajectory_->points[closest_traj_point_idx].pose.position.y;
      double x2 = trajectory_->points[closest_traj_point_idx + 2].pose.position.x;
      double y2 = trajectory_->points[closest_traj_point_idx + 2].pose.position.y;
      double x3 = trajectory_->points[closest_traj_point_idx + 5].pose.position.x;
      double y3 = trajectory_->points[closest_traj_point_idx + 5].pose.position.y;
      
      double dx1 = x2 - x1;
      double dy1 = y2 - y1;
      double dx2 = x3 - x2;
      double dy2 = y3 - y2;
      
      double cross_product = dx1 * dy2 - dy1 * dx2;
      double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
      
      if (d1 > 0.1 && d2 > 0.1) {
        curvature = std::abs(cross_product) / (d1 * d2);
      }
    }
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, 
      "Reference - Vel: %.3f, Pos: (%.3f, %.3f), Lookahead: %.3f, Curvature: %.3f, Steer: %.3f, Velocity Factor: %.3f", 
      reference[0].v, reference[0].x, reference[0].y, 
      std::sqrt(std::pow(reference[0].x - current_state.x, 2) + std::pow(reference[0].y - current_state.y, 2)),
      curvature, control_inputs(0));
    
    // Solve MPC for both lateral and longitudinal control
    Eigen::VectorXd control_inputs = solveMPC(current_state, reference);
    
    // Extract control inputs
    double steer_angle = control_inputs(0);
    double acceleration = control_inputs(1);
    
    // Apply constraints
    steer_angle = std::clamp(steer_angle, -max_steer_angle_, max_steer_angle_);
    acceleration = std::clamp(acceleration, min_acceleration_, max_acceleration_);
    
    // Apply rate limits
    double steer_rate_limit = max_steer_rate_ * dt_;
    double steer_diff = steer_angle - previous_steer_;
    if (std::abs(steer_diff) > steer_rate_limit) {
      steer_angle = previous_steer_ + (steer_diff > 0 ? steer_rate_limit : -steer_rate_limit);
    }
    
    // Additional conservative steering limits
    double max_steer_for_safety = max_steer_angle_ * 0.98; // 98% of max steer angle
    steer_angle = std::clamp(steer_angle, -max_steer_for_safety, max_steer_for_safety);
    
    // Ensure minimum acceleration for movement
    if (current_state.v < 1.0 && acceleration < 0.5) {
      acceleration = 0.5; // Minimum acceleration to start moving
    }
    
    // Set control command
    cmd.longitudinal.speed = reference[0].v;
    cmd.longitudinal.acceleration = acceleration;
    cmd.lateral.steering_tire_angle = steer_angle;
    
    // Update previous control inputs
    previous_steer_ = steer_angle;
    previous_acceleration_ = acceleration;
    
    // Debug information
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, 
      "Full MPC Control - Steer: %.3f, Accel: %.3f, Target Vel: %.3f, Current Vel: %.3f", 
      steer_angle, acceleration, reference[0].v, current_state.v);
    
    // Publish lookahead point for visualization
    geometry_msgs::msg::PointStamped lookahead_point_msg;
    lookahead_point_msg.header.stamp = get_clock()->now();
    lookahead_point_msg.header.frame_id = "map";
    lookahead_point_msg.point.x = reference[0].x;
    lookahead_point_msg.point.y = reference[0].y;
    lookahead_point_msg.point.z = odometry_->pose.pose.position.z;
    pub_lookahead_point_->publish(lookahead_point_msg);
  }
  
  pub_cmd_->publish(cmd);
  pub_raw_cmd_->publish(cmd);
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

VehicleState SimplePurePursuit::getCurrentVehicleState()
{
  VehicleState state;
  state.x = odometry_->pose.pose.position.x;
  state.y = odometry_->pose.pose.position.y;
  state.yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  state.v = odometry_->twist.twist.linear.x;
  state.delta = previous_steer_;
  state.a = previous_acceleration_;
  return state;
}

std::vector<ReferencePoint> SimplePurePursuit::getReferenceTrajectory(size_t start_idx, int horizon)
{
  std::vector<ReferencePoint> reference;
  size_t trajectory_size = trajectory_->points.size();
  
  // Find the closest point to current position
  size_t current_idx = start_idx;
  
  for (int i = 0; i < horizon; ++i) {
    // Shorter lookahead for delayed steering initiation
    double current_velocity = std::max(3.0, odometry_->twist.twist.linear.x);
    
    // Calculate path curvature using closer points for more immediate response
    double curvature = 0.0;
    if (current_idx + 3 < trajectory_size) {
      // Calculate curvature using closer points (reduced from 5 to 3)
      double x1 = trajectory_->points[current_idx].pose.position.x;
      double y1 = trajectory_->points[current_idx].pose.position.y;
      double x2 = trajectory_->points[current_idx + 1].pose.position.x;
      double y2 = trajectory_->points[current_idx + 1].pose.position.y;
      double x3 = trajectory_->points[current_idx + 3].pose.position.x;
      double y3 = trajectory_->points[current_idx + 3].pose.position.y;
      
      // Simple curvature calculation
      double dx1 = x2 - x1;
      double dy1 = y2 - y1;
      double dx2 = x3 - x2;
      double dy2 = y3 - y2;
      
      double cross_product = dx1 * dy2 - dy1 * dx2;
      double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
      double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
      
      if (d1 > 0.1 && d2 > 0.1) {
        curvature = std::abs(cross_product) / (d1 * d2);
      }
    }
    
    // Shorter lookahead for delayed steering initiation
    double base_lookahead = 1.5 - 1.0 * std::min(curvature, 1.0); // Reduced from 2.0-1.5 to 1.5-1.0
    double velocity_factor = std::min(current_velocity / 8.0, 1.5); // Reduced from 5.0 to 8.0, max from 2.0 to 1.5
    double lookahead_multiplier = base_lookahead * velocity_factor;
    double lookahead_distance = (i + 1) * dt_ * current_velocity * lookahead_multiplier;
    
    // Find point at lookahead distance along the trajectory
    size_t target_idx = current_idx;
    double accumulated_distance = 0.0;
    
    for (size_t j = current_idx; j < trajectory_size - 1; ++j) {
      double dx = trajectory_->points[j + 1].pose.position.x - trajectory_->points[j].pose.position.x;
      double dy = trajectory_->points[j + 1].pose.position.y - trajectory_->points[j].pose.position.y;
      double segment_length = std::sqrt(dx * dx + dy * dy);
      
      accumulated_distance += segment_length;
      if (accumulated_distance >= lookahead_distance) {
        target_idx = j;
        break;
      }
    }
    
    target_idx = std::min(target_idx, trajectory_size - 1);
    const auto& point = trajectory_->points[target_idx];
    
    ReferencePoint ref;
    ref.x = point.pose.position.x;
    ref.y = point.pose.position.y;
    ref.yaw = tf2::getYaw(point.pose.orientation);
    ref.v = point.longitudinal_velocity_mps;
    
    // Velocity constraints for stability - ensure minimum movement
    ref.v = std::max(ref.v, 3.0); // Higher minimum velocity to ensure movement
    ref.v = std::min(ref.v, max_velocity_); // Maximum velocity constraint
    
    reference.push_back(ref);
  }
  
  return reference;
}

VehicleState SimplePurePursuit::predictVehicleState(const VehicleState& state, double steer, double accel, double dt)
{
  VehicleState next_state;
  
  // Bicycle model (rear wheel drive) - corrected implementation
  if (std::abs(state.v) < 0.1) {
    // When velocity is very low, use simplified model
    next_state.x = state.x + state.v * std::cos(state.yaw) * dt;
    next_state.y = state.y + state.v * std::sin(state.yaw) * dt;
    next_state.yaw = state.yaw;
  } else {
    // Normal bicycle model
    double beta = std::atan2(wheel_base_ * std::tan(steer), wheel_base_);
    
    next_state.x = state.x + state.v * std::cos(state.yaw + beta) * dt;
    next_state.y = state.y + state.v * std::sin(state.yaw + beta) * dt;
    next_state.yaw = state.yaw + (state.v / wheel_base_) * std::sin(beta) * dt;
  }
  
  next_state.v = state.v + accel * dt;
  next_state.delta = steer;
  next_state.a = accel;
  
  // Normalize yaw angle
  next_state.yaw = normalizeAngle(next_state.yaw);
  
  return next_state;
}

double SimplePurePursuit::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

Eigen::VectorXd SimplePurePursuit::solveMPC(const VehicleState& current_state, 
                                           const std::vector<ReferencePoint>& reference)
{
  // MPC implementation optimized for velocity-adaptive path following
  
  double best_steer = 0.0;
  double best_accel = 0.0;
  double best_cost = std::numeric_limits<double>::max();
  
  // Grid search for control inputs with velocity-adaptive control
  double steer_range = max_steer_angle_;
  double accel_range = max_acceleration_;
  int grid_size = 15; // Increased resolution for better control
  
  // Use previous control as initial guess to reduce oscillation
  double current_steer = previous_steer_;
  double current_accel = previous_acceleration_;
  
  // Calculate current path curvature for adaptive control
  double current_curvature = 0.0;
  if (reference.size() > 3) {
    double x1 = reference[0].x;
    double y1 = reference[0].y;
    double x2 = reference[1].x;
    double y2 = reference[1].y;
    double x3 = reference[3].x;
    double y3 = reference[3].y;
    
    double dx1 = x2 - x1;
    double dy1 = y2 - y1;
    double dx2 = x3 - x2;
    double dy2 = y3 - y2;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (d1 > 0.1 && d2 > 0.1) {
      current_curvature = std::abs(cross_product) / (d1 * d2);
    }
  }
  
  // Calculate velocity-adaptive weights
  double velocity_factor = std::min(current_state.v / 5.0, 2.0);
  double base_lateral_weight = q_lateral_;
  double base_yaw_weight = q_lateral_;
  
  for (int i = 0; i < grid_size; ++i) {
    for (int j = 0; j < grid_size; ++j) {
      double steer = -steer_range + (2.0 * steer_range * i) / (grid_size - 1);
      double accel = -accel_range + (2.0 * accel_range * j) / (grid_size - 1);
      
      // Predict vehicle trajectory
      VehicleState predicted_state = current_state;
      double total_cost = 0.0;
      
      for (int k = 0; k < prediction_horizon_; ++k) {
        predicted_state = predictVehicleState(predicted_state, steer, accel, dt_);
        
        // Calculate tracking errors
        double lateral_error = reference[k].y - predicted_state.y;
        double longitudinal_error = reference[k].v - predicted_state.v;
        double yaw_error = normalizeAngle(reference[k].yaw - predicted_state.yaw);
        
        // Position error in reference frame
        double dx = reference[k].x - predicted_state.x;
        double dy = reference[k].y - predicted_state.y;
        double position_error = std::sqrt(dx * dx + dy * dy);
        
        // Velocity-adaptive weights for better high-speed following
        double lateral_weight = base_lateral_weight * velocity_factor;
        double yaw_weight = base_yaw_weight * velocity_factor;
        
        // Always use maximum weights for strict following
        lateral_weight *= 3.0; // Always triple weight
        yaw_weight *= 3.0; // Always triple weight
        
        // Additional weights for curves - reduced for delayed steering
        if (current_curvature > 0.2) { // Increased threshold from 0.1 to 0.2
          lateral_weight *= 1.5; // Reduced from 2.0 to 1.5
          yaw_weight *= 1.5; // Reduced from 2.0 to 1.5
        }
        if (current_curvature > 0.5) { // Increased threshold from 0.3 to 0.5
          lateral_weight *= 1.2; // Reduced from 1.5 to 1.2
          yaw_weight *= 1.2; // Reduced from 1.5 to 1.2
        }
        
        // Extreme weights for any error
        if (std::abs(lateral_error) > 0.1) {
          lateral_weight *= 5.0; // Extreme weight for any lateral error
        }
        if (std::abs(yaw_error) > 0.05) {
          yaw_weight *= 5.0; // Extreme weight for any yaw error
        }
        
        // Cost function with velocity-adaptive weights
        total_cost += lateral_weight * lateral_error * lateral_error;
        total_cost += q_longitudinal_ * longitudinal_error * longitudinal_error;
        total_cost += yaw_weight * yaw_error * yaw_error;
        total_cost += q_lateral_ * position_error * position_error;
        
        // Control effort cost (only for first step) - minimized for strict following
        if (k == 0) {
          total_cost += r_steer_ * steer * steer;
          total_cost += r_acceleration_ * accel * accel;
          
          // Minimal rate limiting cost for strict control
          double steer_rate = (steer - current_steer) / dt_;
          double accel_rate = (accel - current_accel) / dt_;
          total_cost += q_steer_rate_ * steer_rate * steer_rate;
          total_cost += q_acceleration_ * accel_rate * accel_rate;
        }
        
        // Safety constraints (soft constraints)
        if (std::abs(predicted_state.v) > max_velocity_) {
          total_cost += 1000.0 * (std::abs(predicted_state.v) - max_velocity_) * (std::abs(predicted_state.v) - max_velocity_);
        }
        
        if (std::abs(predicted_state.delta) > max_steer_angle_) {
          total_cost += 1000.0 * (std::abs(predicted_state.delta) - max_steer_angle_) * (std::abs(predicted_state.delta) - max_steer_angle_);
        }
        
        // Minimal penalty for large steering angles (very permissive for strict following)
        if (std::abs(steer) > max_steer_angle_ * 0.95) {
          total_cost += 100.0 * steer * steer; // Very light penalty
        }
        
        // Extreme penalty for any position error
        if (position_error > 0.5) {
          total_cost += 10000.0 * position_error * position_error;
        }
        if (position_error > 1.0) {
          total_cost += 20000.0 * position_error * position_error;
        }
        
        // Penalize negative acceleration when vehicle is slow
        if (current_state.v < 1.0 && accel < 0.0) {
          total_cost += 1000.0 * accel * accel; // Heavy penalty for negative acceleration when slow
        }
      }
      
      if (total_cost < best_cost) {
        best_cost = total_cost;
        best_steer = steer;
        best_accel = accel;
      }
    }
  }
  
  // Apply rate limits (minimal for strict control)
  double steer_rate_limit = max_steer_rate_ * dt_;
  double steer_diff = best_steer - previous_steer_;
  if (std::abs(steer_diff) > steer_rate_limit) {
    best_steer = previous_steer_ + (steer_diff > 0 ? steer_rate_limit : -steer_rate_limit);
  }
  
  Eigen::VectorXd control_inputs(2);
  control_inputs(0) = best_steer;
  control_inputs(1) = best_accel;
  
  return control_inputs;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
