#include "my_robot_controllers/pid_controller.hpp"

/**
 * ── Constructor ────────────────────────────────────────────
 *
 * Called once when the node starts. Sets up:
 *   - ROS2 parameters (kp, ki, kd) — tunable without recompiling
 *   - Subscribers and publisher
 *   - Initial state of all PID variables
 */
PidController::PidController()
: Node("pid_controller"),
  linear_integral_(0.0),
  angular_integral_(0.0),
  prev_linear_error_(0.0),
  prev_angular_error_(0.0),
  desired_linear_x_(0.0),
  desired_angular_z_(0.0),
  first_odom_received_(false)
{
  // ── Declare ROS2 parameters ──────────────────────────────
  // These can be tuned at runtime without recompiling:
  //   ros2 param set /pid_controller kp 1.5
  this->declare_parameter("kp", 1.0);   // Proportional gain
  this->declare_parameter("ki", 0.1);   // Integral gain
  this->declare_parameter("kd", 0.05);  // Derivative gain

  kp_ = this->get_parameter("kp").as_double();
  ki_ = this->get_parameter("ki").as_double();
  kd_ = this->get_parameter("kd").as_double();

  RCLCPP_INFO(this->get_logger(),
    "PID gains — Kp: %.2f  Ki: %.2f  Kd: %.2f", kp_, ki_, kd_);

  // ── Subscribers ──────────────────────────────────────────
  // QoS depth=10 means buffer up to 10 messages if processing is slow
  desired_vel_sub_ =
    this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_desired", 10,
      std::bind(&PidController::desiredVelCallback, this,
                std::placeholders::_1));

  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PidController::odomCallback, this,
                std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────
  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(
      "/model/my_robot/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "PID Controller node started!");
  RCLCPP_INFO(this->get_logger(),
    "Listening on /cmd_vel_desired, publishing to /cmd_vel");
}

/**
 * ── Desired Velocity Callback ──────────────────────────────
 *
 * Called when operator publishes to /cmd_vel_desired.
 * We simply store the desired values — the PID calculation
 * happens in odomCallback where we know the actual velocity.
 */
void PidController::desiredVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  desired_linear_x_  = msg->linear.x;
  desired_angular_z_ = msg->angular.z;

  RCLCPP_DEBUG(this->get_logger(),
    "Desired velocity — linear: %.2f  angular: %.2f",
    desired_linear_x_, desired_angular_z_);
}

/**
 * ── Odometry Callback — The PID Loop ──────────────────────
 *
 * Called every time /odom publishes (~50 Hz).
 * This is where the full PID calculation runs:
 *
 *   error     = desired - actual
 *   P         = kp * error
 *   I         = ki * integral(error * dt)
 *   D         = kd * (error - prev_error) / dt
 *   output    = P + I + D
 */
void PidController::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // ── Extract actual velocity from odometry ────────────────
  double actual_linear_x  = msg->twist.twist.linear.x;
  double actual_angular_z = msg->twist.twist.angular.z;

  // ── Compute dt (time since last callback) ────────────────
  rclcpp::Time current_time = this->now();

  // Skip PID on first message — no dt yet
  if (!first_odom_received_) {
    prev_time_          = current_time;
    first_odom_received_ = true;
    return;
  }

  double dt = (current_time - prev_time_).seconds();

  // Guard against zero or negative dt (clock issues)
  if (dt <= 0.0) {
    return;
  }

  // ── Compute errors ───────────────────────────────────────
  // How far off are we from the desired velocity?
  double linear_error  = desired_linear_x_  - actual_linear_x;
  double angular_error = desired_angular_z_ - actual_angular_z;

  // ── P term — Proportional ────────────────────────────────
  // Larger error = larger correction. Immediate reaction.
  double linear_p  = kp_ * linear_error;
  double angular_p = kp_ * angular_error;

  // ── I term — Integral ────────────────────────────────────
  // Accumulates error over time. Fixes steady-state drift.
  // Without I term, robot might settle slightly below target.
  linear_integral_  += linear_error  * dt;
  angular_integral_ += angular_error * dt;

  double linear_i  = ki_ * linear_integral_;
  double angular_i = ki_ * angular_integral_;

  // ── D term — Derivative ──────────────────────────────────
  // Rate of change of error. Dampens oscillation.
  // If error is rapidly shrinking, D reduces the output.
  double linear_d  = kd_ * (linear_error  - prev_linear_error_)  / dt;
  double angular_d = kd_ * (angular_error - prev_angular_error_) / dt;

  // ── Total PID output ─────────────────────────────────────
  double linear_output  = linear_p  + linear_i  + linear_d;
  double angular_output = angular_p + angular_i + angular_d;

  // ── Clamp output to safe velocity limits ─────────────────
  // Prevents PID from commanding dangerously high speeds
  linear_output  = std::clamp(linear_output,  -1.0, 1.0);
  angular_output = std::clamp(angular_output, -1.0, 1.0);

  // ── Publish corrected velocity ───────────────────────────
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_output;
  cmd.angular.z = angular_output;
  cmd_vel_pub_->publish(cmd);

  // ── Log every 50 callbacks (~1 second) ───────────────────
  RCLCPP_DEBUG(this->get_logger(),
    "Linear  — desired: %.2f  actual: %.2f  error: %.2f  output: %.2f",
    desired_linear_x_, actual_linear_x, linear_error, linear_output);
  RCLCPP_DEBUG(this->get_logger(),
    "Angular — desired: %.2f  actual: %.2f  error: %.2f  output: %.2f",
    desired_angular_z_, actual_angular_z, angular_error, angular_output);

  // ── Save state for next iteration ───────────────────────
  prev_linear_error_  = linear_error;
  prev_angular_error_ = angular_error;
  prev_time_          = current_time;
}

/**
 * ── Main Entry Point ──────────────────────────────────────
 *
 * Standard ROS2 node startup:
 *   1. Initialise ROS2
 *   2. Create node
 *   3. Spin (keep running, processing callbacks)
 *   4. Shutdown cleanly
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidController>());
  rclcpp::shutdown();
  return 0;
}