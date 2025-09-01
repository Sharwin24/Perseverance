# ROS
- Add simulated Wheel Odometry
- Publish robot twist along with odometry from state estimation
- Use robot twist for simulated Lidar and IMU
- Build a basic URDF for rover (diff-drive for now, can worry about rocker-bogie later)
- Define a reference trajectory for tuning kalman filter (circle trajectory)
- Debug simulated IMU causing NaN state values
- Start designing a controller for trajectory tracking (PIC, MPC, LQR, MPPI)
- Once robot is done, setup Gazebo and replace simulated Lidar with Gazebo Lidar

# Rover Design
- Figure out how to convert OnShape CAD model to URDF for visualization
- Finalize rover dimensions

# Electronics
- Teensy 4.0 Development
  - Handle motor control for both drive and steering motors
  - Define interface between Teensy 4.0 and Raspberry Pi 5 (over SPI/UART/CAN)
- PCB for motor drivers and Teensy 4.0