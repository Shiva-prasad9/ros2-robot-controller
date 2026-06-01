# Changelog

## v1.0.0 — Phase 1 Complete
- URDF differential drive robot
- Gazebo Harmonic simulation  
- PID velocity controller (C++)
- Impedance position controller (C++)
- Master bringup launch file
- Professional README with Mermaid diagrams

## v1.1.0 — State Estimation (in progress)
- IMU sensor link + Gazebo plugin added to URDF
- ROS-GZ bridge extended for /imu/data_raw topic
- my_robot_localization package with robot_localization EKF
- Fused odometry published on /odometry/filtered

## v1.2.0 — SLAM Mapping (planned)
- LIDAR sensor addition to URDF
- slam_toolbox integration
- Nav2 basic navigation