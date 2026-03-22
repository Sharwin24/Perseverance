#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <memory>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "base_kalman_filter.hpp"
#include "perseverance_kalman_filter.hpp"
#include "perseverance_sensor_adapters.hpp"

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();
  ~StateEstimator() = default;

private:
  // EKF instance
  std::unique_ptr<PerseveranceEKF> kalmanFilter;

  // ── Sensor adapters ────────────────────────────────────────────────────────
  // Heap-allocated and shared with the filter. The node holds typed shared_ptrs
  // to call update() directly; the filter holds base-class shared_ptrs for
  // populateMeasurementVector() / populateControlInput().
  std::shared_ptr<ImuAdapter> imuAdapter;
  std::shared_ptr<WheelSpeedAdapter> wheelSpeedAdapter;
  std::shared_ptr<SteeringAdapter> frontSteeringAdapter;
  std::shared_ptr<SteeringAdapter> rearSteeringAdapter;

  // ── ROS Subscriptions ──────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheelSpeedSub;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr frontSteeringSub;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr rearSteeringSub;

  // ── Timer ──────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer;

  void timerCallback();

  // ── TF ────────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  // ── Publishers ────────────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;

  // ── Helpers ───────────────────────────────────────────────────────────────
  geometry_msgs::msg::Quaternion yaw2Quaternion(double yaw);
  nav_msgs::msg::Odometry createOdomMessage(const PerseveranceEKF::StateVector& state);
  geometry_msgs::msg::TransformStamped createOdomToBaseFootprintTF(const PerseveranceEKF::StateVector& state);
  geometry_msgs::msg::TransformStamped createBaseLinkCenteredToBaseFootprintTF();
};

#endif // !STATE_ESTIMATION_HPP
