# Changelog

## v1.0.0 — Phase 1 Complete
- URDF differential drive robot
- Gazebo Harmonic simulation  
- PID velocity controller (C++)
- Impedance position controller (C++)
- Master bringup launch file
- Professional README with Mermaid diagrams


## v1.1.0 — State Estimation (feature/imu-ekf)

### Added
- IMU sensor link and joint in URDF (parented to `chassis_link`)
- Gazebo IMU sensor plugin with MEMS noise model (MPU-6050 class)
- Custom world SDF with explicit sensor system plugins for Gazebo Harmonic
- ROS-GZ bridge extended with `/imu/data_raw` (GZ→ROS unidirectional)
- `my_robot_localization` package — EKF state estimation via `robot_localization`
- `CovarianceInjector` C++ node — fixes zero-covariance from Gazebo plugins
- EKF fusing `/odom_with_covariance` + `/imu/data` → `/odometry/filtered`
- Smoke test plot demonstrating trajectory smoothing and covariance convergence

### Fixed
- Gazebo `empty.sdf` does not load sensor systems — replaced with custom world
- Diff drive and IMU plugins publish zero covariance — fixed via injector node
- Process noise matrix off-diagonal shift causing filter divergence

### Architecture
```
Gazebo IMU → /imu/data_raw → CovarianceInjector → /imu/data ──────┐
                                                                    ├─► ekf_node → /odometry/filtered
Gazebo Odom → /odom → CovarianceInjector → /odom_with_covariance ──┘
```

## v1.2.0 — SLAM Mapping (planned)
- LIDAR sensor addition to URDF
- slam_toolbox integration
- Nav2 basic navigation