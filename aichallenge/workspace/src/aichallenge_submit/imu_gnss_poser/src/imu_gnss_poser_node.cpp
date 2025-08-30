#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ImuGnssPoser : public rclcpp::Node
{
public:
    ImuGnssPoser() : Node("imu_gnss_poser")
    {
        // ★★★ 改善点：パラメータの宣言 ★★★
        position_covariance_ = this->declare_parameter<double>("position_covariance", 0.1);
        orientation_covariance_ = this->declare_parameter<double>("orientation_covariance", 10.0);
        gnss_orientation_trust_threshold_ = this->declare_parameter<double>("gnss_orientation_trust_threshold", 1.0);


        const auto rv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        const auto rt_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/imu_gnss_poser/pose_with_covariance", rv_qos);
        pub_initial_pose_3d_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/initial_pose3d", rt_qos);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/localization/twist_with_covariance", rv_qos,
            std::bind(&ImuGnssPoser::twist_callback, this, std::placeholders::_1));
        sub_gnss_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", rv_qos,
            std::bind(&ImuGnssPoser::gnss_callback, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensing/imu/imu_raw", rv_qos,
            std::bind(&ImuGnssPoser::imu_callback, this, std::placeholders::_1));
    }

private:

    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        
        auto current_gnss_pose = *msg;

        // ★★★ 改善点：より洗練されたロジック ★★★
        bool use_imu_orientation = false;

        // 1. GNSSの向き情報が明らかに無効な場合
        if (std::isnan(current_gnss_pose.pose.pose.orientation.x) ||
            (current_gnss_pose.pose.pose.orientation.x == 0 &&
             current_gnss_pose.pose.pose.orientation.y == 0 &&
             current_gnss_pose.pose.pose.orientation.z == 0 &&
             current_gnss_pose.pose.pose.orientation.w == 0))
        {
            use_imu_orientation = true;
        }

        // 2. GNSS自身が報告する向きの共分散が大きい場合も、IMUの向きを採用
        // (GNSSドライバがヨーの共分散を適切に設定している場合に有効)
        const double yaw_covariance = current_gnss_pose.pose.covariance[5*6 + 5];
        if (yaw_covariance > gnss_orientation_trust_threshold_) {
            use_imu_orientation = true;
        }

        if (use_imu_orientation && imu_received_) {
            current_gnss_pose.pose.pose.orientation = imu_msg_.orientation;
            
            // IMUの向きを使ったことを共分散に反映
            current_gnss_pose.pose.covariance[3*6 + 3] = orientation_covariance_; // Roll
            current_gnss_pose.pose.covariance[4*6 + 4] = orientation_covariance_; // Pitch
            current_gnss_pose.pose.covariance[5*6 + 5] = orientation_covariance_; // Yaw
        }

        // 位置の共分散はパラメータ値で上書き (GNSSの測位精度はある程度一定と仮定)
        current_gnss_pose.pose.covariance[0*6 + 0] = position_covariance_; // X
        current_gnss_pose.pose.covariance[1*6 + 1] = position_covariance_; // Y
        current_gnss_pose.pose.covariance[2*6 + 2] = position_covariance_; // Z


        pub_pose_->publish(current_gnss_pose);
        
        // EKFの初期化が済んでいない場合、最初の信頼できるポーズを初期位置として一度だけパブリッシュ
        if (!is_ekf_initialized_ && (use_imu_orientation || yaw_covariance < gnss_orientation_trust_threshold_)) {
            pub_initial_pose_3d_->publish(current_gnss_pose);
        }
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = *msg;
        imu_received_ = true;
    }

    void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr)
    {
        // EKFが一度でも動作を開始したら、初期位置の再設定は行わない
        is_ekf_initialized_ = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_3d_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    sensor_msgs::msg::Imu imu_msg_;
    bool is_ekf_initialized_ = {false};
    bool imu_received_ = {false};

    // ★★★ 改善点：パラメータ用メンバ変数 ★★★
    double position_covariance_;
    double orientation_covariance_;
    double gnss_orientation_trust_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuGnssPoser>());
    rclcpp::shutdown();
    return 0;
}