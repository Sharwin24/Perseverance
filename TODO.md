# ROS

## State Estimation
- Add simulated Wheel Odometry
- Publish robot twist along with odometry from state estimation
- Define a reference trajectory for tuning kalman filter (circle trajectory)

## Sensors
- Use robot twist for simulated Lidar and IMU
- Debug simulated IMU causing NaN state values

## Communications
- ROS node for interfacing with Feather MCU over SPI

## Motion Control
- Start designing a controller for trajectory tracking (PIC, MPC, LQR, MPPI)
- Ackermann Steering Planner for translating Twist commands to wheel velocities and steering angles
- Utilize AckermannDrive messages for sending wheel commands around ROS ecosystem

## Robot Model
- Build a basic URDF for rover (diff-drive for now, can worry about rocker-bogie later by locking the differential as a fixed joint)
- Setup Gazebo and replace simulated Lidar with Gazebo Lidar

# Rover Design
- Figure out how to convert OnShape CAD model to URDF for visualization
- Mounting Electronics
  - Feather MCU
  - IMU
  - Lidar
  - Camera
  - Battery Monitor

# Electronics
- Battery Management System (BMS) Design
  - Monitor (Adafruit LC709203F)
  - 12V PD device for charging (Adafruit 5807)
  - 12V 3S Lipo Charger
  - 12V 3S Lipo Battery
  - 12V to 5V/5A Buck Converter (for Raspberry Pi 5)

# Prototyping
- Interactive pygame simulation for testing translation from geometry_msgs/Twist to wheel velocities and steering angles.
- Translate steering planner from Python to C++
