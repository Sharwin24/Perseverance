#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <memory>
#include <mutex>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

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

  nav_msgs::msg::Odometry createOdomMessage(const RobotState& state);

  geometry_msgs::msg::TransformStamped createOdomToBaseLinkTF(const RobotState& state);


  // State Estimate (Odometry)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
  // IMU Data Subscription
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
  // Wheel Odometry Subscription
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<KalmanFilter> kalmanFilter;

  // Cache of the latest IMU message for use in the timer
  sensor_msgs::msg::Imu::SharedPtr lastImuMsg;
  std::mutex imuMutex;

  // Static TF broadcaster for map -> odom
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

  // TF broadcaster for odom -> base_link
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
};

#endif // !STATE_ESTIMATION_HPP
