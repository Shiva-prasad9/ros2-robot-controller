#include "my_robot_controllers/impedance_controller.hpp"
#include <cmath>

/**
 * ── Constructor ────────────────────────────────────────────
 *
 * Sets up impedance parameters as tunable ROS2 params.
 * Default values give compliant but responsive behaviour:
 *   K=1.0  — moderate spring stiffness
 *   B=0.8  — good damping (prevents oscillation)
 *   M=0.1  — low virtual mass (responsive)
 */
ImpedanceController::ImpedanceController()
: Node("impedance_controller"),
  desired_x_(0.0),
  desired_y_(0.0),
  desired_theta_(0.0),
  current_x_(0.0),
  current_y_(0.0),
  current_theta_(0.0),
  current_linear_vel_(0.0),
  current_angular_vel_(0.0),
  first_odom_received_(false)
{
  // ── Declare impedance parameters ────────────────────────
  // Tunable at runtime:
  //   ros2 param set /impedance_controller k 2.0
  this->declare_parameter("k", 1.0);    // Stiffness
  this->declare_parameter("b", 0.8);    // Damping
  this->declare_parameter("m", 0.1);    // Virtual mass
  this->declare_parameter("max_linear_vel",  0.5);
  this->declare_parameter("max_angular_vel", 1.0);

  k_ = this->get_parameter("k").as_double();
  b_ = this->get_parameter("b").as_double();
  m_ = this->get_parameter("m").as_double();
  max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();

  RCLCPP_INFO(this->get_logger(),
    "Impedance params — K: %.2f  B: %.2f  M: %.2f", k_, b_, m_);

  // ── Subscribers ──────────────────────────────────────────
  desired_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::Pose2D>(
      "/desired_pose", 10,
      std::bind(&ImpedanceController::desiredPoseCallback,
                this, std::placeholders::_1));

  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ImpedanceController::odomCallback,
                this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────
  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(
      "/model/my_robot/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(),
    "Impedance Controller started!");
  RCLCPP_INFO(this->get_logger(),
    "Listening on /desired_pose, publishing to /model/my_robot/cmd_vel");
}

/**
 * ── Desired Pose Callback ──────────────────────────────────
 *
 * Stores the target position whenever a new goal arrives.
 * Pose2D contains: x (metres), y (metres), theta (radians)
 */
void ImpedanceController::desiredPoseCallback(
  const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
  desired_x_     = msg->x;
  desired_y_     = msg->y;
  desired_theta_ = msg->theta;

  RCLCPP_INFO(this->get_logger(),
    "New desired pose — x: %.2f  y: %.2f  theta: %.2f",
    desired_x_, desired_y_, desired_theta_);
}

/**
 * ── Odometry Callback — The Impedance Loop ────────────────
 *
 * Runs every time /odom publishes (~50 Hz).
 *
 * Impedance Law:
 *   F_linear  = K*(x_error)  - B*(current_linear_vel)
 *   F_angular = K*(th_error) - B*(current_angular_vel)
 *
 * Where x_error is distance to goal projected onto
 * robot's forward direction, and th_error is the
 * angular difference to the goal heading.
 *
 * The M term (virtual mass) is simplified here —
 * it acts as a smoothing factor on the output.
 */
void ImpedanceController::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // ── Extract current position from odometry ───────────────
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // Convert quaternion to yaw angle (theta)
  // Quaternion: (x, y, z, w) → yaw = atan2(2*qw*qz, 1-2*qz²)
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  current_theta_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

  // ── Extract current velocity ─────────────────────────────
  current_linear_vel_  = msg->twist.twist.linear.x;
  current_angular_vel_ = msg->twist.twist.angular.z;

  if (!first_odom_received_) {
    first_odom_received_ = true;
    return;
  }

  // ── Compute position errors ──────────────────────────────
  // Distance error: how far are we from goal?
  double dx = desired_x_ - current_x_;
  double dy = desired_y_ - current_y_;
  double distance_error = std::sqrt(dx * dx + dy * dy);

  // Angle to goal: direction we need to face
  double angle_to_goal = std::atan2(dy, dx);

  // Heading error: difference between where we face
  // and where we need to go
  double heading_error = normalizeAngle(
    angle_to_goal - current_theta_);

  // Final orientation error: difference between desired
  // theta and current theta (for when we reach the goal)
  double theta_error = normalizeAngle(
    desired_theta_ - current_theta_);

  // ── Apply Impedance Law ──────────────────────────────────
  //
  // Linear force: spring pulls toward goal, damper resists motion
  // Only move forward if roughly facing the goal (|heading| < 90°)
  double linear_force = 0.0;
  if (std::abs(heading_error) < M_PI / 2.0) {
    linear_force = k_ * distance_error
                 - b_ * current_linear_vel_;
  }

  // Angular force: spring rotates toward goal angle,
  // damper resists current rotation
  double angular_force = k_ * heading_error
                       - b_ * current_angular_vel_;

  // ── Virtual mass smoothing ───────────────────────────────
  // M acts as low-pass filter — prevents jerky commands
  // Output = Force / Mass (F = ma → a = F/m → v += a*dt)
  double linear_output  = linear_force  / (m_ + 1.0);
  double angular_output = angular_force / (m_ + 1.0);

  // ── Safety clamp ─────────────────────────────────────────
  linear_output  = std::clamp(linear_output,
                              -max_linear_vel_, max_linear_vel_);
  angular_output = std::clamp(angular_output,
                              -max_angular_vel_, max_angular_vel_);

  // ── Stop if close enough to goal ────────────────────────
  // Dead zone prevents constant micro-corrections
  if (distance_error < 0.05) {
    // Close to position — just correct orientation
    linear_output = 0.0;
    angular_output = k_ * theta_error - b_ * current_angular_vel_;
    angular_output = std::clamp(angular_output,
                                -max_angular_vel_, max_angular_vel_);

    // Fully stop if orientation is also correct
    if (std::abs(theta_error) < 0.05) {
      linear_output  = 0.0;
      angular_output = 0.0;
      RCLCPP_INFO_THROTTLE(this->get_logger(),
        *this->get_clock(), 2000,
        "Goal reached! Position error: %.3f  Theta error: %.3f",
        distance_error, theta_error);
    }
  }

  // ── Publish velocity command ─────────────────────────────
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_output;
  cmd.angular.z = angular_output;
  cmd_vel_pub_->publish(cmd);

  // ── Debug logging ────────────────────────────────────────
  RCLCPP_DEBUG(this->get_logger(),
    "Pos error: %.2f  Head error: %.2f  "
    "Lin out: %.2f  Ang out: %.2f",
    distance_error, heading_error,
    linear_output, angular_output);
}

/**
 * ── Normalize Angle ────────────────────────────────────────
 *
 * Wraps angle to [-pi, pi] range.
 * Example: 270° becomes -90°, 370° becomes 10°
 * Critical for angular error calculations.
 */
double ImpedanceController::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * ── Main Entry Point ──────────────────────────────────────
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImpedanceController>());
  rclcpp::shutdown();
  return 0;
}