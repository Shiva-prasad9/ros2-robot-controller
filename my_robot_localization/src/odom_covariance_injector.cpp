#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

/**
 * CovarianceInjector
 *
 * Gazebo's diff drive and IMU plugins publish zero covariance on all
 * fields. The EKF treats zero covariance as perfect measurements,
 * causing numerical instability and physically impossible estimates.
 *
 * This node subscribes to both /odom and /imu/data_raw, injects
 * realistic covariance values, and republishes on:
 *   /odom_with_covariance  — for EKF odometry input
 *   /imu/data              — for EKF IMU input
 *
 * Covariance values derived from sensor noise model:
 *   Odom position x, y : 0.1   m²
 *   Odom yaw           : 0.05  rad²
 *   Odom velocity x    : 0.01  m²/s²
 *   IMU yaw            : 0.02  rad²
 *   IMU gyro           : 8.1e-05 rad²/s² (stddev 0.009)
 *   IMU accel          : 4.41e-04 m²/s⁴  (stddev 0.021)
 */
class CovarianceInjector : public rclcpp::Node
{
public:
  CovarianceInjector() : Node("covariance_injector")
  {
    // ── Odometry ────────────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&CovarianceInjector::odom_callback, this,
                std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom_with_covariance", 10);

    // ── IMU ─────────────────────────────────────────────────
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 10,
      std::bind(&CovarianceInjector::imu_callback, this,
                std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", 10);

    RCLCPP_INFO(this->get_logger(),
      "CovarianceInjector started. "
      "/odom -> /odom_with_covariance | "
      "/imu/data_raw -> /imu/data");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto out = *msg;

    // 6x6 row-major. Diagonal: [0]=x [7]=y [14]=z [21]=roll [28]=pitch [35]=yaw
    out.pose.covariance.fill(0.0);
    out.pose.covariance[0]  = 0.1;   // x
    out.pose.covariance[7]  = 0.1;   // y
    out.pose.covariance[14] = 0.1;   // z
    out.pose.covariance[21] = 0.1;   // roll
    out.pose.covariance[28] = 0.1;   // pitch
    out.pose.covariance[35] = 0.05;  // yaw

    out.twist.covariance.fill(0.0);
    out.twist.covariance[0]  = 0.01;  // vx
    out.twist.covariance[7]  = 0.01;  // vy
    out.twist.covariance[14] = 0.01;  // vz
    out.twist.covariance[21] = 0.01;  // vroll
    out.twist.covariance[28] = 0.01;  // vpitch
    out.twist.covariance[35] = 0.01;  // vyaw

    odom_pub_->publish(out);
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto out = *msg;

    // 3x3 row-major. Diagonal: [0]=roll [4]=pitch [8]=yaw
    out.orientation_covariance.fill(0.0);
    out.orientation_covariance[0] = 0.05;    // roll  (unused in 2D)
    out.orientation_covariance[4] = 0.05;    // pitch (unused in 2D)
    out.orientation_covariance[8] = 0.02;    // yaw

    out.angular_velocity_covariance.fill(0.0);
    out.angular_velocity_covariance[0] = 8.1e-05;  // stddev 0.009²
    out.angular_velocity_covariance[4] = 8.1e-05;
    out.angular_velocity_covariance[8] = 8.1e-05;

    out.linear_acceleration_covariance.fill(0.0);
    out.linear_acceleration_covariance[0] = 4.41e-04;  // stddev 0.021²
    out.linear_acceleration_covariance[4] = 4.41e-04;
    out.linear_acceleration_covariance[8] = 4.41e-04;

    imu_pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CovarianceInjector>());
  rclcpp::shutdown();
  return 0;
}