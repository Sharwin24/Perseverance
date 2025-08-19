#ifndef SENSORS_SIMULATED_IMU_HPP
#define SENSORS_SIMULATED_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <cmath>

class SimulatedIMU : public rclcpp::Node {
public:
  SimulatedIMU() : Node("simulated_imu") {
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("sensors/raw/imu", 10);
    double freq = this->declare_parameter<double>("sensor_frequency", 100.0);
    freq = this->get_parameter("sensor_frequency", freq);
    this->period_ = 1.0 / freq;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / freq)),
      std::bind(&SimulatedIMU::publish_imu, this));
    t_ = 0.0;
  }

private:
  void publish_imu() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    // Simulate some motion: e.g., sinusoidal angular velocity and acceleration
    msg.linear_acceleration.x = 0.5 * std::sin(t_);
    msg.linear_acceleration.y = 0.5 * std::cos(t_);
    msg.linear_acceleration.z = -9.81; // gravity
    msg.angular_velocity.z = 0.05 * std::sin(0.5 * t_);
    // Optionally fill orientation (quaternion)
    msg.orientation.w = 1.0; msg.orientation.x = 0.0; msg.orientation.y = 0.0; msg.orientation.z = 0.0;
    pub_->publish(msg);
    t_ += this->period_;
  }
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
  double period_;
};

#endif // SENSORS_SIMULATED_IMU_HPP
