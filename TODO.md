# ROS
- Add simulated Wheel Odometry
- Publish robot twist along with odometry from state estimation
- Use robot twist for simulated Lidar and IMU
- Build a basic URDF for rover (diff-drive for now, can worry about rocker-bogie later by locking the differential as a fixed joint)
- Define a reference trajectory for tuning kalman filter (circle trajectory)
- Debug simulated IMU causing NaN state values
- Start designing a controller for trajectory tracking (PIC, MPC, LQR, MPPI)
- Once robot is done, setup Gazebo and replace simulated Lidar with Gazebo Lidar
- ROS node for interfacing with Feather MCU over SPI
- ROS node for implementing kinematics for rover, translate Twist commands to wheel velocities and steering angles

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
