/// @file state_estimation.cpp
/// @brief State Estimation for mobile robot using Kalman Filter to fuse IMU and Odometry data.
///
/// PARAMETERS:
///
/// PUBLISHERS:
///
/// SUBSCRIBERS:
///
/// SERVICES:
///
/// CLIENTS:

#include "state_estimation.hpp"

const std::string IMUTopic = "/sensors/raw/imu";
const std::string ODOMTopic = "/sensors/raw/odom";

StateEstimator::StateEstimator() : Node("state_estimator") {
  RCLCPP_INFO(this->get_logger(), "State Estimator Node has been initialized.");

  // Declare parameters
  double timer_freq = this->declare_parameter("timer_frequency", 100.0); // [Hz]
  double initial_x = this->declare_parameter("initial_x", 0.0);
  double initial_y = this->declare_parameter("initial_y", 0.0);
  double initial_theta = this->declare_parameter("initial_theta", 0.0);
  double initial_vx = this->declare_parameter("initial_vx", 0.0);
  double initial_vy = this->declare_parameter("initial_vy", 0.0);
  double initial_omega = this->declare_parameter("initial_omega", 0.0);
  // Get parameters from yaml config file
  timer_freq = this->get_parameter("timer_frequency").as_double();
  initial_x = this->get_parameter("initial_x").as_double();
  initial_y = this->get_parameter("initial_y").as_double();
  initial_theta = this->get_parameter("initial_theta").as_double();
  initial_vx = this->get_parameter("initial_vx").as_double();
  initial_vy = this->get_parameter("initial_vy").as_double();
  initial_omega = this->get_parameter("initial_omega").as_double();
  // Raw Sensor Data (Subscribers)
  const auto rawDataQoS = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  // Filtered Sensor Data (Publishers)
  const auto filteredDataQoS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Initialize subscriptions
  this->imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
    IMUTopic, rawDataQoS, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
  this->odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
    ODOMTopic, rawDataQoS, std::bind(&StateEstimator::odomCallback, this, std::placeholders::_1));

  this->currentState = RobotState(
    initial_x, initial_y, initial_theta,
    initial_vx, initial_vy, initial_omega
  );
  RCLCPP_INFO(this->get_logger(), "Initial state set to: [%f, %f, %f, %f, %f, %f]",
    initial_x, initial_y, initial_theta, initial_vx, initial_vy, initial_omega
  );
  // Initialize covariance
  this->covariance.setStateCovariance(0.05, 0.05, 0.01, 0.1, 0.1, 0.0825);
  this->covariance.setProcessNoiseCovariance(0.005, 0.005, 0.001, 0.075, 0.075, 0.005);
  this->covariance.setOdomMeasurementNoiseCovariance(1.0, 1.0, 0.05);
  this->covariance.setImuMeasurementNoiseCovariance(0.0075);

  // Initialize timer for periodic tasks
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq),
    std::bind(&StateEstimator::timerCallback, this)
  );
}

void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "IMU data received.");
}

void StateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Odometry data received.");
}

void StateEstimator::timerCallback() {
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}