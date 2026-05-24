#ifndef MY_ROBOT_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * ImpedanceController Node
 *
 * Models the robot as a virtual mass-spring-damper system.
 * Instead of directly commanding velocity, it computes the
 * "force" needed to move the robot to the desired position,
 * considering virtual stiffness (K), damping (B), and mass (M).
 *
 * Subscribes to:
 *   /desired_pose  — target position (x, y, theta)
 *   /odom          — actual position and velocity
 *
 * Publishes to:
 *   /cmd_vel       — velocity command derived from impedance law
 *
 * Impedance Law:
 *   F = K*(x_d - x) + B*(v_d - v) + M*(a_d - a)
 *
 * For simplicity we assume v_d = 0 and a_d = 0 (hold position)
 * So: F = K*position_error - B*current_velocity
 */
class ImpedanceController : public rclcpp::Node
{
public:
  /**
   * Constructor — sets up parameters, pubs/subs
   */
  ImpedanceController();

private:
  // ── Callbacks ───────────────────────────────────────────

  /**
   * Called when a new desired pose arrives.
   * Stores target x, y, theta for impedance calculation.
   */
  void desiredPoseCallback(
    const geometry_msgs::msg::Pose2D::SharedPtr msg);

  /**
   * Called every time /odom publishes (~50 Hz).
   * Extracts current position and velocity, then:
   *   1. Computes position error
   *   2. Applies impedance law: F = K*error - B*velocity
   *   3. Converts force to velocity command
   *   4. Publishes /cmd_vel
   */
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * Helper: Normalize angle to [-pi, pi]
   * Prevents angle wrapping issues (e.g. 359° vs -1°)
   */
  double normalizeAngle(double angle);

  // ── ROS2 interfaces ─────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr  desired_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;

  // ── Impedance parameters (tunable via ROS2 params) ──────
  double k_;  // Stiffness  — spring force toward desired pose
  double b_;  // Damping    — resistance to current velocity
  double m_;  // Virtual mass — resistance to acceleration

  // ── Desired pose (set by /desired_pose topic) ────────────
  double desired_x_;      // Target X position (metres)
  double desired_y_;      // Target Y position (metres)
  double desired_theta_;  // Target heading (radians)

  // ── Current state (from /odom) ───────────────────────────
  double current_x_;
  double current_y_;
  double current_theta_;
  double current_linear_vel_;   // Current forward speed
  double current_angular_vel_;  // Current turn rate

  // ── Velocity limits (safety clamp) ──────────────────────
  double max_linear_vel_;
  double max_angular_vel_;

  // ── Guard for first odom message ────────────────────────
  bool first_odom_received_;
};

#endif  // MY_ROBOT_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_