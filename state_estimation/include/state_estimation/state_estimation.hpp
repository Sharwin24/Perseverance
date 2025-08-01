#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "kalman_filter.hpp"

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();
  ~StateEstimator() = default;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;

  rclcpp::TimerBase::SharedPtr timer;

  KalmanFilter kalmanFilter;
};

#endif // !STATE_ESTIMATION_HPP