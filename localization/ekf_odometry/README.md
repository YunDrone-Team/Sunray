# IMU EKF Odometry Package

ROS1 package for high-frequency IMU-based odometry with EKF fusion.

## Overview

This package provides a node that:
1. Subscribes to IMU data (`/livox/imu`) and low-frequency odometry (`/Odometry`)
2. Performs IMU pre-integration to generate high-frequency odometry
3. Uses Extended Kalman Filter (EKF) to fuse IMU odometry with low-frequency odometry
4. Publishes high-frequency fused odometry on `/sunray/imu_odometry`

## Features

- **IMU Pre-integration**: Integrates IMU measurements to estimate position, velocity, and orientation
- **Extended Kalman Filter**: Fuses high-frequency IMU data with low-frequency odometry measurements
- **Bias Estimation**: Estimates and corrects for IMU accelerometer and gyroscope biases
- **Configurable**: Easy parameter configuration via ROS parameters or YAML file

## Dependencies

- ROS Noetic (or Melodic)
- Eigen3
- Standard ROS packages: roscpp, sensor_msgs, nav_msgs, geometry_msgs, tf2

## Building

```bash
cd ~/catkin_ws
catkin_make --pkg imu_ekf_odometry
# or
catkin build imu_ekf_odometry
```

## Usage

### Launch the node

```bash
roslaunch imu_ekf_odometry imu_ekf_odometry.launch
```

### Topics

**Subscribed Topics:**
- `/livox/imu` (sensor_msgs/Imu): IMU data from Livox Mid360
- `/Odometry` (nav_msgs/Odometry): Low-frequency odometry from FAST-LIO

**Published Topics:**
- `/sunray/imu_odometry` (nav_msgs/Odometry): High-frequency fused odometry

### Parameters

See [config/imu_ekf_odometry.yaml](config/imu_ekf_odometry.yaml) for parameter descriptions.

Key parameters:
- `imu_topic`: Input IMU topic name
- `odom_topic`: Input odometry topic name
- `imu_odom_topic`: Output high-frequency odometry topic name
- `gravity_magnitude`: Gravity magnitude in m/s² (default: -9.81)
- `publish_tf`: Whether to publish TF transforms (default: false)

## Algorithm Details

### IMU Pre-integration

The IMU pre-integration module integrates angular velocity and linear acceleration:
- **Position**: p(t+dt) = p(t) + v(t)*dt + 0.5*a(t)*dt²
- **Velocity**: v(t+dt) = v(t) + a(t)*dt
- **Orientation**: q(t+dt) = q(t) ⊗ exp(ω*dt)

Where a(t) is the acceleration in world frame after bias removal and gravity compensation.

### Extended Kalman Filter

State vector (15 DOF error state):
- Position (3D)
- Velocity (3D)
- Rotation error (3D)
- Accelerometer bias (3D)
- Gyroscope bias (3D)

The EKF performs:
1. **Prediction**: Propagate state using IMU pre-integration
2. **Update**: Correct state using low-frequency odometry measurements

## Author

Created for the Sunray mapping project.

## License

MIT License
