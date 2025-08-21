# TODO

## ROS
- Add simulated Wheel Odometry
- Publish robot twist along with odometry from state estimation
- Use robot twist for simulated Lidar and IMU
- Build a basic URDF for rover (diff-drive for now, can worry about rocker-bogie later)
- Define a reference trajectory for tuning kalman filter (circle trajectory)
- Debug simulated IMU causing NaN state values
- Start designing a controller for trajectory tracking (PIC, MPC, LQR, MPPI)
- Once robot is done, setup Gazebo and replace simulated Lidar with Gazebo Lidar

## Rover Design
- Finish Rocker-Bogie Suspension design with simple wheels (no steering)
- Figure out how to convert OnShape CAD model to URDF for visualization

## Electronics
- Figure out sensor selection
  - Wheel Encoders: ...
  - IMU: BNO055
  - Lidar: RPLIDAR A1
  - Camera: ...
- Figure out (closed loop) motors and motor controller that can easily talk to RPI5