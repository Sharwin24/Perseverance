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
  - Implement a simple Pure Pursuit controller for testing
- Ackermann Steering Planner for translating Twist commands to wheel velocities and steering angles
- Utilize AckermannDrive messages for sending wheel commands around ROS ecosystem
- Create a ROS node to handle motion control logic:
  - Subscribe to goal pose and current robot state
  - Plan a wheel velocity and steering angle trajectory
  - Publish wheel commands to the appropriate topic (later be consumed by comms node for sending to MCU)

## Robot Model
- Setup Gazebo and replace simulated Lidar with Gazebo Lidar

# Rover Design
- Mounting Electronics
  - Feather MCU
  - IMU
  - Lidar
  - Camera
  - Battery Monitor

## Electronics
- Battery Management System (BMS) Design
  - Monitor (Adafruit LC709203F)
  - 12V PD device for charging (Adafruit 5807)
  - 12V 3S Lipo Charger
  - 12V 3S Lipo Battery
  - 12V to 5V/5A Buck Converter (for Raspberry Pi 5)

# Prototyping
- Translate steering planner from Python to C++
