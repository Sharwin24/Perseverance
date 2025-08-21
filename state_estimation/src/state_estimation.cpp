/// @file state_estimation.cpp
/// @brief State Estimation for mobile robot using Kalman Filter to fuse IMU and Odometry data
///
/// PARAMETERS:
///   timer_frequency (float64) : Frequency of the timer for updating the state estimate [Hz]
///   initial_x       (float64) : Initial X position [m]
///   initial_y       (float64) : Initial Y position [m]
///   initial_theta   (float64) : Initial yaw/orientation [rad]
///   initial_vx      (float64) : Initial X velocity [m/s]
///   initial_vy      (float64) : Initial Y velocity [m/s]
///   initial_omega   (float64) : Initial angular velocity (yaw rate) [rad/s]
///   wheel_base      (float64) : Wheel-to-wheel distance along the robot Y-axis [m]
///   wheel_radius    (float64) : Wheel radius [m]
///   track_width     (float64) : Back-to-front wheel distance along the robot X-axis [m]
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
const std::string RobotStateTopic = "/state_estimation/odom";

StateEstimator::StateEstimator() : Node("state_estimator") {
  RCLCPP_INFO(this->get_logger(), "State Estimator Node has been initialized.");

  // Declare parameters
  double timer_freq = this->declare_parameter("timer_frequency", 10.0); // [Hz]
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

  // Sensor Data QoS (Subscribers)
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  // Initialize subscriptions
  this->imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
    IMUTopic, qos, std::bind(&StateEstimator::imuCallback, this, std::placeholders::_1));
  this->odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
    ODOMTopic, qos, std::bind(&StateEstimator::odomCallback, this, std::placeholders::_1));
  // Initialize Odom (RobotState) Publisher
  this->odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(RobotStateTopic, qos);

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

  RCLCPP_INFO(this->get_logger(), "State Covariance (P): diag[x=%f, y=%f, theta=%f, vx=%f, vy=%f, omega=%f]",
    cov.stateCovariance(0, 0), cov.stateCovariance(1, 1),
    cov.stateCovariance(2, 2), cov.stateCovariance(3, 3),
    cov.stateCovariance(4, 4), cov.stateCovariance(5, 5)
  );

  RCLCPP_INFO(this->get_logger(), "Process Noise Covariance (Q): diag[%f, %f, %f, %f, %f, %f]",
    cov.processNoiseCovariance(0, 0), cov.processNoiseCovariance(1, 1),
    cov.processNoiseCovariance(2, 2), cov.processNoiseCovariance(3, 3),
    cov.processNoiseCovariance(4, 4), cov.processNoiseCovariance(5, 5)
  );

  RCLCPP_INFO(this->get_logger(), "Odom Measurement Noise Covariance (R_odom): diag[x=%f, y=%f, theta=%f]",
    cov.odomMeasurementNoiseCovariance(0, 0),
    cov.odomMeasurementNoiseCovariance(1, 1),
    cov.odomMeasurementNoiseCovariance(2, 2)
  );

  RCLCPP_INFO(this->get_logger(), "IMU Measurement Noise Covariance (R_imu): diag[omega=%f]",
    cov.imuMeasurementNoiseCovariance(0, 0)
  );

  // Create Robot Constants
  RobotConstants robotConstants(wheel_base, wheel_radius, track_width);

  // Setup Kalman Filter
  this->kalmanFilter = std::make_unique<KalmanFilter>(
    this->get_clock(),
    robotConstants,
    PredictionModel::DYNAMIC, KinematicModel::DIFF_DRIVE,
    cov, initialState
  );

  // Initialize static TF broadcaster and publish map -> odom using initial state
  this->staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  {
    geometry_msgs::msg::TransformStamped mapToOdom;
    mapToOdom.header.stamp = this->now();
    mapToOdom.header.frame_id = "map";
    mapToOdom.child_frame_id = "odom";
    // Place odom in the map frame at the initial robot pose (x,y,theta)
    mapToOdom.transform.translation.x = initial_x;
    mapToOdom.transform.translation.y = initial_y;
    mapToOdom.transform.translation.z = 0.0;
    const auto q = this->yaw2Quaternion(initial_theta);
    mapToOdom.transform.rotation = q;
    this->staticTfBroadcaster->sendTransform(mapToOdom);
  }

  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq),
    std::bind(&StateEstimator::timerCallback, this)
  );
}

void StateEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Cache the latest IMU message
  {
    std::lock_guard<std::mutex> lock(this->imuMutex);
    this->lastImuMsg = msg;
  }
  // Save the measurement
  const double Z = msg->angular_velocity.z; // [rad/s]
  const auto& H = this->kalmanFilter->getIMUMeasurementModel();
  const auto& X = this->kalmanFilter->getStateVector();
  // Calculate the innovation
  const auto Y = Z - (H * X);
  // Calculate the Kalman Gain [K] using the innovation covariance [S]
  const auto& P = this->kalmanFilter->getStateCovariance();
  const auto& R_imu = this->kalmanFilter->getIMUMeasurementNoiseCovariance();
  const auto S = H * P * H.transpose() + R_imu;
  const auto K = P * H.transpose() * S.inverse();
  // Update the state estimate and covariance
  const auto X_new = X + K * Y;
  const auto P_new = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  this->kalmanFilter->updateState(X_new);
  this->kalmanFilter->updateStateCovariance(P_new);
}

void StateEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Save measurement
  const auto odom = msg->pose.pose.position;
  const auto quat = msg->pose.pose.orientation;
  const Eigen::Quaterniond orientation = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
  const double yaw = orientation.toRotationMatrix().eulerAngles(0, 1, 2).z();
  const auto Z = Eigen::Vector<double, 3>({odom.x, odom.y, yaw});
  const auto H = this->kalmanFilter->getOdometryMeasurementModel();
  const auto X = this->kalmanFilter->getStateVector();
  // Calculate the innovation
  const auto Y = Z - H * X;
  // Calculate the Kalman Gain [k] using the innovation covariance [S]
  const auto P = this->kalmanFilter->getStateCovariance();
  const auto R_odom = this->kalmanFilter->getOdometryMeasurementNoiseCovariance();
  const auto S = H * P * H.transpose() + R_odom;
  const auto K = P * H.transpose() * S.inverse();
  // Update the state estimate and covariance
  const auto X_new = X + K * Y;
  const auto P_new = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  this->kalmanFilter->updateState(X_new);
  this->kalmanFilter->updateStateCovariance(P_new);
}

void StateEstimator::timerCallback() {
  // Run prediction model to update the current state estimate
  RobotState predictedState;
  switch (this->kalmanFilter->getPredictionModel()) {
  case PredictionModel::DYNAMIC: {
    // Predict using the dynamic model
    sensor_msgs::msg::Imu::SharedPtr imu;
    {
      std::lock_guard<std::mutex> lock(this->imuMutex);
      imu = this->lastImuMsg;
    }
    if (imu) {
      predictedState = this->kalmanFilter->predictDynamicModel(*imu);
    }
    break;
  }
  case PredictionModel::KINEMATIC: {
    // Predict using the kinematic model
    const auto kinematicParams = KinematicModelInput(); // TODO: Get actual kinematic parameters
    predictedState = this->kalmanFilter->predictKinematicModel(kinematicParams);
    break;
  }
  }

  // Update the new state
  this->updateState(predictedState);

  // Display the current Robot State Vector
  RCLCPP_INFO(this->get_logger(),
    "Current State Estimate: [x=%.3f [m], y=%.3f [m], theta=%.3f [rad], vx=%.3f [m/s], vy=%.3f [m/s], omega=%.3f [rad/s]]",
    predictedState.x, predictedState.y, predictedState.theta, predictedState.vx, predictedState.vy, predictedState.omega
  );
  // Create an Odom message to publish the current state
  nav_msgs::msg::Odometry odomMsg;
  odomMsg.header.frame_id = "odom";
  odomMsg.child_frame_id = "base_link";
  odomMsg.header.stamp = this->now();
  odomMsg.pose.pose.position.x = predictedState.x;
  odomMsg.pose.pose.position.y = predictedState.y;
  odomMsg.pose.pose.orientation = this->yaw2Quaternion(predictedState.theta);
  auto create_odom_cov_array = [&](const Eigen::Matrix<double, 6, 6>& P) {
    // Only copy x, y, theta covariance to the pose covariance array
    std::array<double, 36> arr{};
    arr[0] = P(0, 0);    // x
    arr[7] = P(1, 1);    // y
    arr[35] = P(2, 2);   // theta (yaw, rotation about Z)
    return arr;
  };
  auto create_twist_cov_array = [&](const Eigen::Matrix<double, 6, 6>& P) {
    // Only copy vx, vy, omega covariance to the twist covariance array
    std::array<double, 36> arr{};
    arr[0] = P(3, 3);    // vx
    arr[7] = P(4, 4);    // vy
    arr[35] = P(5, 5);    // omega (yaw rate)
    return arr;
  };
  odomMsg.pose.covariance = create_odom_cov_array(this->kalmanFilter->getStateCovariance());
  odomMsg.twist.twist.linear.x = predictedState.vx;
  odomMsg.twist.twist.linear.y = predictedState.vy;
  odomMsg.twist.twist.angular.z = predictedState.omega;
  odomMsg.twist.covariance = create_twist_cov_array(this->kalmanFilter->getStateCovariance());
  this->odomPublisher->publish(odomMsg);
}

geometry_msgs::msg::Quaternion StateEstimator::yaw2Quaternion(const double yaw) {
  // Convert yaw to quaternion
  const double half_yaw = yaw / 2.0;
  const double w = std::cos(half_yaw);
  const double z = std::sin(half_yaw);
  geometry_msgs::msg::Quaternion q;
  q.w = w;
  q.x = 0.0;
  q.y = 0.0;
  q.z = z;
  return q;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}