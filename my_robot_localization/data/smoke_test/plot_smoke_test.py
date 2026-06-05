#!/usr/bin/env python3
"""
EKF Smoke Test — Plot Results
Reads the smoke_test bag and generates four plots demonstrating
EKF sensor fusion performance.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # WSL2 — no display, save to file
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages

BAG_PATH = "./smoke_test/smoke_test_0.mcap"

# ── Data containers ───────────────────────────────────────────────
odom = {"t": [], "x": [], "y": [], "vx": [], "vyaw": []}
filt = {"t": [], "x": [], "y": [], "vx": [], "vyaw": [],
        "cov_x": [], "cov_y": [], "cov_yaw": []}
imu  = {"t": [], "yaw_rate": []}

print("Reading bag...")

for schema, channel, message in read_ros2_messages(BAG_PATH):
    t = message.log_time / 1e9  # nanoseconds to seconds

    if channel.topic == "/odom":
        msg = message.ros_msg
        odom["t"].append(t)
        odom["x"].append(msg.pose.pose.position.x)
        odom["y"].append(msg.pose.pose.position.y)
        odom["vx"].append(msg.twist.twist.linear.x)
        odom["vyaw"].append(msg.twist.twist.angular.z)

    elif channel.topic == "/odometry/filtered":
        msg = message.ros_msg
        filt["t"].append(t)
        filt["x"].append(msg.pose.pose.position.x)
        filt["y"].append(msg.pose.pose.position.y)
        filt["vx"].append(msg.twist.twist.linear.x)
        filt["vyaw"].append(msg.twist.twist.angular.z)
        filt["cov_x"].append(msg.pose.covariance[0])    # x variance
        filt["cov_y"].append(msg.pose.covariance[7])    # y variance
        filt["cov_yaw"].append(msg.pose.covariance[35]) # yaw variance

    elif channel.topic == "/imu/data":
        msg = message.ros_msg
        imu["t"].append(t)
        imu["yaw_rate"].append(msg.angular_velocity.z)

print(f"Loaded: odom={len(odom['t'])} filt={len(filt['t'])} imu={len(imu['t'])}")

# Normalize timestamps to start at 0
t0 = min(odom["t"][0], filt["t"][0], imu["t"][0])
for d in [odom, filt, imu]:
    d["t"] = [t - t0 for t in d["t"]]

# ── Convert to numpy ──────────────────────────────────────────────
for d in [odom, filt, imu]:
    for k in d:
        d[k] = np.array(d[k])

# ── Plot ──────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("EKF Smoke Test — IMU + Odometry Fusion\n"
             "robot_localization ekf_node | ROS2 Jazzy | Gazebo Harmonic",
             fontsize=13, fontweight='bold')

# Plot 1 — XY trajectory
ax = axes[0, 0]
ax.plot(odom["x"], odom["y"], 'b-', alpha=0.4, linewidth=1, label="Raw /odom")
ax.plot(filt["x"], filt["y"], 'r-', linewidth=2, label="EKF /odometry/filtered")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Robot Trajectory")
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

# Plot 2 — Linear velocity comparison
ax = axes[0, 1]
ax.plot(odom["t"], odom["vx"], 'b-', alpha=0.4, linewidth=1, label="Raw /odom vx")
ax.plot(filt["t"], filt["vx"], 'r-', linewidth=2, label="EKF filtered vx")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Linear Velocity X (m/s)")
ax.set_title("Linear Velocity — Raw vs Filtered")
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 3 — EKF covariance convergence
ax = axes[1, 0]
ax.plot(filt["t"], filt["cov_x"],   'r-', linewidth=2, label="Var(x)")
ax.plot(filt["t"], filt["cov_y"],   'b-', linewidth=2, label="Var(y)")
ax.plot(filt["t"], filt["cov_yaw"], 'g-', linewidth=2, label="Var(yaw)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Variance")
ax.set_title("EKF Covariance Convergence\n(filter confidence increasing as variance drops)")
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_yscale('log')  # log scale shows convergence clearly

# Plot 4 — Yaw rate: IMU vs EKF
ax = axes[1, 1]
ax.plot(imu["t"],  imu["yaw_rate"],  'b-', alpha=0.4, linewidth=1, label="IMU raw vyaw")
ax.plot(filt["t"], filt["vyaw"],     'r-', linewidth=2, label="EKF filtered vyaw")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Yaw Rate (rad/s)")
ax.set_title("Yaw Rate — IMU vs EKF Filtered")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
output = "./smoke_test_results.png"
plt.savefig(output, dpi=150, bbox_inches='tight')
print(f"Saved: {output}")