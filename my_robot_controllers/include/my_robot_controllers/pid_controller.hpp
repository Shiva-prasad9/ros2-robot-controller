#ifndef MY_ROBOT_CONTROLLERS__PID_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__PID_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * PidController Node
 *
 * Subscribes to:
 *   /cmd_vel_desired  — target velocity (from operator or planner)
 *   /odom             — actual velocity (from Gazebo diff drive)
 *
 * Publishes to:
 *   /cmd_vel          — PID-corrected velocity command
 *
 * The PID loop runs every time a new /odom message arrives.
 * This ensures the correction rate matches the sensor rate.
 */
class PidController : public rclcpp::Node
{
public:
  /**
   * Constructor — initialises node, parameters, pubs/subs
   */
  PidController();

private:
  // ── Callbacks ───────────────────────────────────────────

  /**
   * Called when a new desired velocity arrives.
   * Simply stores it — actual PID runs in odom callback.
   */
  void desiredVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * Called every time /odom publishes (50 Hz).
   * This is where the PID calculation happens:
   *   1. Extract actual velocity from odometry
   *   2. Compute error = desired - actual
   *   3. Compute P, I, D terms
   *   4. Publish corrected /cmd_vel
   */
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  // ── ROS2 interfaces ─────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_pub_;

  // ── PID gains (tunable via ROS2 parameters) ─────────────
  double kp_;   // Proportional: reacts to current error
  double ki_;   // Integral:     corrects persistent drift
  double kd_;   // Derivative:   smooths sudden changes

  // ── PID state variables ──────────────────────────────────
  double linear_integral_;    // Accumulated linear error
  double angular_integral_;   // Accumulated angular error
  double prev_linear_error_;  // Previous linear error (for D term)
  double prev_angular_error_; // Previous angular error (for D term)

  // ── Desired velocity (set by /cmd_vel_desired) ───────────
  double desired_linear_x_;   // Target forward speed (m/s)
  double desired_angular_z_;  // Target turn rate (rad/s)

  // ── Time tracking (needed to compute dt for I and D) ────
  rclcpp::Time prev_time_;
  bool first_odom_received_;  // Guards against dt=0 on first callback
};

#endif  // MY_ROBOT_CONTROLLERS__PID_CONTROLLER_HPP_