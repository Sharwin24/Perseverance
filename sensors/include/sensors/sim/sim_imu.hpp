#ifndef SENSORS_SIM_IMU_HPP
#define SENSORS_SIM_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <cmath>

class SimulatedIMU : public rclcpp::Node {
public:
  SimulatedIMU();

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
  rclcpp::TimerBase::SharedPtr timer;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

  double timestep;
  double period;
};

#endif // SENSORS_SIM_IMU_HPP
