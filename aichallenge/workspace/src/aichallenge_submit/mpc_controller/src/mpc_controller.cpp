// src/mpc_controller.cpp

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class MPCController : public rclcpp::Node {
public:
  MPCController() : rclcpp::Node("mpc_controller_node") {
    // ---- Parameters ----
    speed_kp_       = this->declare_parameter<double>("speed_kp", 1.0);
    lookahead_min_  = this->declare_parameter<double>("lookahead_min", 2.2);
    lookahead_k_    = this->declare_parameter<double>("lookahead_k", 0.75);   // Ld = min + k*v
    wheel_base_     = this->declare_parameter<double>("wheel_base", 1.087);   // 車両実寸に合わせる
    max_steer_rad_  = this->declare_parameter<double>("max_steer_rad", 0.64); // 車両実寸に合わせる
    invert_steer_   = this->declare_parameter<bool>("invert_steer", false);
    ref_v_cap_      = this->declare_parameter<double>("ref_v_cap", 6.0);      // 調整用の上限（チューニング時）

    // ---- QoS ----
    auto traj_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(); // VOLATILE + BEST_EFFORT
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto pub_qos  = rclcpp::QoS(rclcpp::KeepLast(10));

    // ---- I/O ----
    sub_traj_ = this->create_subscription<Trajectory>(
      "input/trajectory", traj_qos,
      [this](Trajectory::SharedPtr msg){ traj_ = std::move(msg); });

    sub_odom_ = this->create_subscription<Odometry>(
      "input/kinematics", odom_qos,
      [this](Odometry::SharedPtr msg){ odom_ = std::move(msg); });

    pub_cmd_ = this->create_publisher<AckermannControlCommand>(
      "output/control_cmd", pub_qos);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(20ms, std::bind(&MPCController::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "mpc_controller started. QoS: traj(BEST_EFFORT + VOLATILE), odom(RELIABLE)");
  }

private:
  // ===== Timer =====
  static constexpr int WARMUP_KICK_CYCLES = 20; // 起動直後の配線確認用に ~1秒だけ軽く前進
  int    kick_count_{0};
  double prev_steer_{0.0};
  const double steer_lpf_alpha_{0.1}; // 0(強)～1(なし) の一次遅れ

  void onTimer() {
    if (!odom_ || !traj_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "waiting inputs: odom=%d traj=%d", odom_ ? 1 : 0, traj_ ? 1 : 0);
      warmupKick();
      return;
    }
    if (kick_count_ < WARMUP_KICK_CYCLES) {
      warmupKick();
      return;
    }

    const auto& pose = odom_->pose.pose;
    const double yaw = yawFromQuat(pose.orientation);

    // 現在速度（m/s）
    const double cur_v = std::hypot(
      odom_->twist.twist.linear.x,
      odom_->twist.twist.linear.y);

    // 動的ルックアヘッド（必要なら上限を 6.0〜8.0 で調整）
    const double Ld = std::clamp(lookahead_min_ + lookahead_k_ * cur_v, 2.0, 8.0);

    // 最近傍点 → Ld 先のターゲット（線形補間で生成）
    const auto nearest = nearestPoint(traj_, pose.position);
    if (!nearest) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "nearest point not found; skip");
      return;
    }
    const auto target = forwardPoint(*traj_, pose.position, Ld, *nearest);

    // 横偏差（車体座標系：x前方, y左方）— ログ用
    const double ey = lateralError(pose.position, yaw, target.pose.position);

    // 速度目標（Trajectoryからの生値）
    const double ref_v_raw = std::max(0.0,
      static_cast<double>(nearest->longitudinal_velocity_mps));

    // ラテラルエラーに応じて“ソフト減速”（|ey| 1 m につき 1 m/s 減らす、上限3）
    double ref_v_soft = 0.0;
    if(ref_v_raw > 3.0) {
      ref_v_soft = std::max(0.0, ref_v_raw - std::min(std::abs(ey), 3.0));
    } else {
      ref_v_soft = ref_v_raw; // 3 m/s 以下はそのまま
    }

    // チューニング時の上限（安定したら launch で引き上げ／外し）
    const double ref_v = std::min(ref_v_soft, ref_v_cap_);

    // 加速度コマンド（P相当）
    double accel_cmd = std::clamp(speed_kp_ * (ref_v - cur_v), -1.0, 1.0);

    // ========= 幾何 Pure Pursuit =========
    // 目標点を車体座標へ
    const double dx = target.pose.position.x - pose.position.x;
    const double dy = target.pose.position.y - pose.position.y;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double x_local =  c*dx + s*dy;   // 前方
    const double y_local = -s*dx + c*dy;   // 左+

    // 目標角 alpha を使う PP（2L*sin(alpha)/Ld）
    const double alpha = std::atan2(y_local, std::max(1e-3, x_local));
    double steer_cmd = std::atan2(2.0 * wheel_base_ * std::sin(alpha),
                                  std::max(1e-3, Ld));
    if (invert_steer_) steer_cmd = -steer_cmd;

    // ステアLPF + 飽和
    steer_cmd   = prev_steer_ + steer_lpf_alpha_ * (steer_cmd - prev_steer_);
    prev_steer_ = steer_cmd;
    steer_cmd   = std::clamp(steer_cmd, -max_steer_rad_, max_steer_rad_);

    // publish
    AckermannControlCommand cmd;
    cmd.stamp = this->now();
    cmd.longitudinal.acceleration   = static_cast<float>(accel_cmd);
    cmd.lateral.steering_tire_angle = static_cast<float>(steer_cmd);
    pub_cmd_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "ref_v=%.2f cur_v=%.2f Ld=%.2f accel=%.2f steer=%.3f ey=%.2f",
      ref_v, cur_v, Ld, accel_cmd, steer_cmd, ey);
  }

  void warmupKick() {
    if (kick_count_ >= WARMUP_KICK_CYCLES) return;
    AckermannControlCommand cmd;
    cmd.stamp = this->now();
    cmd.longitudinal.acceleration   = 0.5f;
    cmd.lateral.steering_tire_angle = 0.0f;
    pub_cmd_->publish(cmd);
    ++kick_count_;
  }

  // ===== Utils =====
  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double sqr(double x){ return x*x; }

  static double dist2(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b){
    return sqr(a.x-b.x) + sqr(a.y-b.y) + sqr(a.z-b.z);
  }

  static double lateralError(const geometry_msgs::msg::Point& base_pos,
                             double base_yaw,
                             const geometry_msgs::msg::Point& tgt_pos) {
    const double dx = tgt_pos.x - base_pos.x;
    const double dy = tgt_pos.y - base_pos.y;
    const double c = std::cos(base_yaw), s = std::sin(base_yaw);
    const double x_local =  c * dx + s * dy; // 前
    const double y_local = -s * dx + c * dy; // 左+
    (void)x_local;
    return y_local;
  }

  // 最近傍点（値コピーで返す）
  static std::optional<TrajectoryPoint>
  nearestPoint(const Trajectory::SharedPtr& traj, const geometry_msgs::msg::Point& p) {
    if (!traj || traj->points.empty()) return std::nullopt;
    double best = std::numeric_limits<double>::infinity();
    const TrajectoryPoint* best_pt = nullptr;
    for (const auto& tp : traj->points) {
      const double d2 = dist2(tp.pose.position, p);
      if (d2 < best) { best = d2; best_pt = &tp; }
    }
    if (!best_pt) return std::nullopt;
    return *best_pt;
  }

  // Ld 先のターゲット点を「区間内線形補間」で生成
  static TrajectoryPoint
  forwardPoint(const Trajectory& traj,
               const geometry_msgs::msg::Point& p,
               double lookahead_dist,
               const TrajectoryPoint& fallback)
  {
    if (traj.points.empty()) return fallback;

    // 最近傍 index
    size_t idx_near = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < traj.points.size(); ++i) {
      const double d2 = dist2(traj.points[i].pose.position, p);
      if (d2 < best) { best = d2; idx_near = i; }
    }

    // idx_near→前方へ累積距離。Ld を越える区間で線形補間
    double acc = 0.0;
    for (size_t i = idx_near; i + 1 < traj.points.size(); ++i) {
      const auto& A = traj.points[i].pose.position;
      const auto& B = traj.points[i+1].pose.position;
      const double seg = std::hypot(B.x - A.x, B.y - A.y);
      if (acc + seg >= lookahead_dist && seg > 1e-6) {
        const double r = (lookahead_dist - acc) / seg; // 0..1
        TrajectoryPoint out = traj.points[i];          // ベースは A の属性
        out.pose.position.x = A.x + r * (B.x - A.x);
        out.pose.position.y = A.y + r * (B.y - A.y);
        out.pose.position.z = A.z + r * (B.z - A.z);
        // A→B の方位を付与
        geometry_msgs::msg::Quaternion q{};
        const double yaw = std::atan2(B.y - A.y, B.x - A.x);
        q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw * 0.5); q.w = std::cos(yaw * 0.5);
        out.pose.orientation = q;
        // 速度は B を採用（必要なら補間も可）
        out.longitudinal_velocity_mps = traj.points[i+1].longitudinal_velocity_mps;
        return out;
      }
      acc += seg;
    }
    return traj.points.back();
  }

private:
  // IO
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<Odometry>::SharedPtr   sub_odom_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  // latest inputs
  Trajectory::SharedPtr traj_;
  Odometry::SharedPtr   odom_;

  // params
  double speed_kp_{1.0};
  double lookahead_min_{2.2}, lookahead_k_{0.75};
  double wheel_base_{1.087};
  double max_steer_rad_{0.64};
  bool   invert_steer_{false};
  double ref_v_cap_{6.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCController>());
  rclcpp::shutdown();
  return 0;
}
