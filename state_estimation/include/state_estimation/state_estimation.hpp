#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mutex>
#include <memory>

#include "kalman_filter.hpp"

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();
  ~StateEstimator() = default;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  void updateState(RobotState newState) const { this->kalmanFilter->updateState(newState.vec()); }

  geometry_msgs::msg::Quaternion yaw2Quaternion(const double yaw);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<KalmanFilter> kalmanFilter;

  // Cache of the latest IMU message for use in the timer
  std::mutex imuMutex;
  sensor_msgs::msg::Imu::SharedPtr lastImuMsg;
};

#endif // !STATE_ESTIMATION_HPP