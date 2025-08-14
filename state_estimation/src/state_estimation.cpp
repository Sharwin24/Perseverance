/// @file state_estimation.cpp
/// @brief State Estimation for mobile robot using Kalman Filter to fuse IMU and Odometry data
///
/// PARAMETERS:
///   timer_frequency (float64) : Frequency of the timer for updating the state estimate [Hz]
///
/// PUBLISHERS:
///   /sensors/filtered/imu (sensor_msgs::msg::Imu): Filtered IMU data
///   /sensors/filtered/odom (nav_msgs::msg::Odometry): Filtered Odometry data
///
/// SUBSCRIBERS:
///   /sensors/raw/imu (sensor_msgs::msg::Imu): Raw IMU data
///   /sensors/raw/odom (nav_msgs::msg::Odometry): Raw Odometry data
///
/// SERVICES:
///
/// CLIENTS:

#include "state_estimation.hpp"
#include "kalman_filter.hpp"

const std::string IMUTopic = "/sensors/raw/imu";
const std::string ODOMTopic = "/sensors/raw/odom";

StateEstimator::StateEstimator() : Node("state_estimator") {
  RCLCPP_INFO(this->get_logger(), "State Estimator Node has been initialized.");

  // Declare parameters
  double timer_freq = this->declare_parameter("timer_frequency", 100.0); // [Hz]
  double initial_x = this->declare_parameter("initial_x", 0.0); // [m]
  double initial_y = this->declare_parameter("initial_y", 0.0); // [m]
  double initial_theta = this->declare_parameter("initial_theta", 0.0); // [rad]
  double initial_vx = this->declare_parameter("initial_vx", 0.0); // [m/s]
  double initial_vy = this->declare_parameter("initial_vy", 0.0); // [m/s]
  double initial_omega = this->declare_parameter("initial_omega", 0.0); // [rad/s]
  double wheel_base = this->declare_parameter("wheel_base", 1.0); // [m]
  double wheel_radius = this->declare_parameter("wheel_radius", 0.05); // [m]
  double track_width = this->declare_parameter("track_width", 0.6); // [m]
  // Get parameters from yaml config file
  timer_freq = this->get_parameter("timer_frequency").as_double();
  initial_x = this->get_parameter("initial_x").as_double();
  initial_y = this->get_parameter("initial_y").as_double();
  initial_theta = this->get_parameter("initial_theta").as_double();
  initial_vx = this->get_parameter("initial_vx").as_double();
  initial_vy = this->get_parameter("initial_vy").as_double();
  initial_omega = this->get_parameter("initial_omega").as_double();
  wheel_base = this->get_parameter("wheel_base").as_double();
  wheel_radius = this->get_parameter("wheel_radius").as_double();
  track_width = this->get_parameter("track_width").as_double();
  // Raw Sensor Data (Subscribers)
  const auto rawDataQoS = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  // Filtered Sensor Data (Publishers)
  // const auto filteredDataQoS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Initialize subscriptions
  this->imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
    IMUTopic, rawDataQoS, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
  this->odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
    ODOMTopic, rawDataQoS, std::bind(&StateEstimator::odomCallback, this, std::placeholders::_1));


  RobotState initialState(
    initial_x, initial_y, initial_theta,
    initial_vx, initial_vy, initial_omega
  );

  RCLCPP_INFO(this->get_logger(), "Initial state set to: [x=%f, y=%f, theta=%f, vx=%f, vy=%f, omega=%f]",
    initial_x, initial_y, initial_theta, initial_vx, initial_vy, initial_omega
  );

  // Initialize covariance
  Covariance cov;
  cov.setStateCovariance(0.05, 0.05, 0.01, 0.1, 0.1, 0.0825);
  cov.setProcessNoiseCovariance(0.005, 0.005, 0.001, 0.075, 0.075, 0.005);
  cov.setOdomMeasurementNoiseCovariance(1.0, 1.0, 0.05);
  cov.setImuMeasurementNoiseCovariance(0.0075);

  RCLCPP_INFO(this->get_logger(), "State Covariance (P): diag[%f, %f, %f, %f, %f, %f]",
    cov.stateCovariance(0, 0), cov.stateCovariance(1, 1),
    cov.stateCovariance(2, 2), cov.stateCovariance(3, 3),
    cov.stateCovariance(4, 4), cov.stateCovariance(5, 5)
  );

  RCLCPP_INFO(this->get_logger(), "Process Noise Covariance (Q): diag[%f, %f, %f, %f, %f, %f]",
    cov.processNoiseCovariance(0, 0), cov.processNoiseCovariance(1, 1),
    cov.processNoiseCovariance(2, 2), cov.processNoiseCovariance(3, 3),
    cov.processNoiseCovariance(4, 4), cov.processNoiseCovariance(5, 5)
  );

  RCLCPP_INFO(this->get_logger(), "Odom Measurement Noise Covariance (R_odom): diag[%f, %f, %f]",
    cov.odomMeasurementNoiseCovariance(0, 0),
    cov.odomMeasurementNoiseCovariance(1, 1),
    cov.odomMeasurementNoiseCovariance(2, 2)
  );

  RCLCPP_INFO(this->get_logger(), "IMU Measurement Noise Covariance (R_imu): diag[%f]",
    cov.imuMeasurementNoiseCovariance(0, 0)
  );

  // Create Robot Constants
  RobotConstants robotConstants(wheel_base, wheel_radius, track_width);

  // Setup Kalman Filter
  this->kalmanFilter = KalmanFilter(
    robotConstants,
    PredictionModel::DYNAMIC, KinematicModel::ROCKER_BOGIE,
    cov, initialState
  );

  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq),
    std::bind(&StateEstimator::timerCallback, this)
  );
}

void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Save the measurement
  const double Z = msg->angular_velocity.z; // [rad/s]
  const auto H = this->kalmanFilter.imuMeasurementModel();
  const auto X = this->kalmanFilter.stateVector();
  // Calculate the innovation
  const auto Y = Z - (H * X);
  // Calculate the Kalman Gain [K] using the innovation covariance [S]
  const auto P = this->kalmanFilter.processNoiseCovariance();
  const auto R_imu = this->kalmanFilter.imuMeasurementNoiseCovariance();
  const auto S = H * P * H.transpose() + R_imu;
  const auto K = P * H.transpose() * S.inverse();
  // Update the state estimate and covariance
  const auto X_new = X + K * Y;
  const auto P_new = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  this->kalmanFilter.updateState(X_new);
  this->kalmanFilter.updateProcessNoiseCovariance(P_new);
}

void StateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Save measurement
  const auto odom = msg->pose.pose.position;
  const auto quat = msg->pose.pose.orientation;
  /// TODO: Convert quaternion to yaw angle [Euler Z]
  const Eigen::Quaternion orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  const double theta = orientation.toRotationMatrix().eulerAngles(0, 1, 2).z();
  const auto Z = Eigen::Vector<double, 3>({odom.x, odom.y, theta});
  const auto H = this->kalmanFilter.odomMeasurementModel();
  const auto X = this->kalmanFilter.stateVector();
  // Calculate the innovation
  const auto Y = Z - H * X;
  // Calculate the Kalman Gain [k] using the innovation covariance [S]
  const auto P = this->kalmanFilter.processNoiseCovariance();
  const auto R_odom = this->kalmanFilter.odomMeasurementNoiseCovariance();
  const auto S = H * P * H.transpose() + R_odom;
  const auto K = P * H.transpose() * S.inverse();
  // Update the state estimate and covariance
  const auto X_new = X + K * Y;
  const auto P_new = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  this->kalmanFilter.updateState(X_new);
  this->kalmanFilter.updateProcessNoiseCovariance(P_new);
}

void StateEstimator::timerCallback() {
  // Run prediction model to update the current state estimate
  switch (this->kalmanFilter.getPredictionModel()) {
  case PredictionModel::DYNAMIC: {
    // Predict using the dynamic model
    const auto imu = this->imuSubscription->get_last_message();
    if (imu) {
      this->kalmanFilter.predictDynamicModel(*imu);
    }
    break;
  }
  case PredictionModel::KINEMATIC: {
    // Predict using the kinematic model
    const auto kinematicParams = KinematicModelInput(); // TODO: Get actual kinematic parameters
    const long timestamp = this->now().seconds();
    this->kalmanFilter.predictKinematicModel(kinematicParams, timestamp);
    break;
  }
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}