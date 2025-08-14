#include "kalman_filter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <array>

RobotState KalmanFilter::predictDynamicModel(const sensor_msgs::msg::Imu imu, const long timestamp) {
  const long imuTimestamp = imu.header.stamp.sec + imu.header.stamp.nanosec / 1e9;
  const long dt = imuTimestamp - this->previousPredictionTimeStamp;
  if (dt <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "IMU message has non-positive timestamp difference. Skipping prediction");
    return this->currentState; // No update if the timestamp is not valid
  }

  // Convert robot-frame acceleration to global-frame acceleration
  const double theta = this->currentState.theta;
  const double globalAccX = imu.linear_acceleration.x * std::cos(theta) - imu.linear_acceleration.y * std::sin(theta);
  const double globalAccY = imu.linear_acceleration.x * std::sin(theta) + imu.linear_acceleration.y * std::cos(theta);

  // Acquire State Transition matrix (F) and Control Input Model (B)
  const auto F = this->systemModel.getStateTransitionMatrix(dt);
  const auto B = this->systemModel.getControlInputModel(dt);

  // Establish Control Vector (U)
  const auto U = Eigen::Vector2d(globalAccX, globalAccY);

  // Predict the next state: x_pred = F * x + B * u
  Eigen::Vector<double, 6> predictedStateVector = F * this->currentState.vec() + B * U;
  assert(predictedStateVector.size() == 6 && "State vector must have 6 elements");
  RobotState predictedState(predictedStateVector);

  // Update the state covariance with the process noise and state transition matrix
  const auto P = this->covariance.stateCovariance;
  const auto Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = imuTimestamp;

  // Return the newly predicted state
  return predictedState;
}

RobotState KalmanFilter::predictKinematicModel(const KinematicModelInput& kinematicParams, const long timestamp) {
  switch (this->kinematicModel) {
  case KinematicModel::DIFF_DRIVE:
    return this->predictDiffDriveKinematicModel(kinematicParams.leftVelocity, kinematicParams.rightVelocity, timestamp);
  case KinematicModel::MECANUM:
    if (kinematicParams.wheelVelocities.size() != MECANUM_WHEEL_NAMES.size()) {
      throw std::invalid_argument(
        "Invalid wheel velocities vector size for MECANUM model: "
        + std::to_string(kinematicParams.wheelVelocities.size())
        + " (expected 4: {front_left, front_right, rear_left, rear_right}).");
    }
    return this->predictMecanumKinematicModel(kinematicParams.wheelVelocities, timestamp);
  case KinematicModel::ROCKER_BOGIE:
    if (kinematicParams.wheelVelocities.size() != ROCKER_BOGIE_WHEEL_NAMES.size() ||
      kinematicParams.wheelSteeringAngles.size() != ROCKER_BOGIE_WHEEL_NAMES.size()) {
      throw std::invalid_argument(
        "Invalid wheel velocities or steering angles map size for ROCKER_BOGIE model: "
        + std::to_string(kinematicParams.wheelVelocities.size()) + " (expected 6: {front_left, front_right, middle_left, middle_right, rear_left, rear_right}) and "
        + std::to_string(kinematicParams.wheelSteeringAngles.size()) + " (expected 6: {front_left, front_right, middle_left, middle_right, rear_left, rear_right}).");
    }
    return this->predictRockerBogieKinematicModel(kinematicParams.wheelVelocities, kinematicParams.wheelSteeringAngles, timestamp);
  default:
    throw std::runtime_error("Unknown kinematic model type.");
  }
}


RobotState KalmanFilter::predictDiffDriveKinematicModel(const double leftVelocity, const double rightVelocity, const long timestamp) {
  const double dt = timestamp - this->previousPredictionTimeStamp;
  if (dt <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Kinematic model prediction has non-positive timestamp difference. Skipping prediction");
    return this->currentState; // No update if the timestamp is not valid
  }

  // Calculate kinematic inputs V (forward) and W (angular)
  const auto V = (leftVelocity + rightVelocity) / 2.0; // Average velocity
  const auto W = (rightVelocity - leftVelocity) / this->robotConstants.wheelBase; // Angular velocity

  // Predict new state with non-linear state transition function
  const auto xOld = this->currentState.x;
  const auto yOld = this->currentState.y;
  const auto thetaOld = this->currentState.theta;

  // Update state using kinematic equations
  const double xNew = xOld + V * std::cos(thetaOld) * dt;
  const double yNew = yOld + V * std::sin(thetaOld) * dt;
  const double thetaNew = thetaOld + W * dt;
  const double vXNew = V * std::cos(thetaNew);
  const double vYNew = V * std::sin(thetaNew);
  RobotState predictedState(xNew, yNew, thetaNew, vXNew, vYNew, W);

  // Calculate the Jacobian (F)
  const auto F = this->systemModel.getStateTransitionMatrix(dt, V, W, thetaOld);

  // Update the state covariance with the process noise and state transition matrix
  const auto P = this->covariance.stateCovariance;
  const auto Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}

RobotState KalmanFilter::predictMecanumKinematicModel(const std::unordered_map<std::string, double>& wheelVelocities, const long timestamp) {
  const double dt = timestamp - this->previousPredictionTimeStamp;
  if (dt <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Kinematic model prediction has non-positive timestamp difference. Skipping prediction");
    return this->currentState; // No update if the timestamp is not valid
  }

  // Expected keys: front_left, front_right, rear_left, rear_right
  for (const auto& k : MECANUM_WHEEL_NAMES) {
    if (wheelVelocities.find(k) == wheelVelocities.end()) {
      throw std::invalid_argument(std::string("Missing wheel velocity key for MECANUM: ") + k);
    }
  }

  // Calculate the robot body twist
  const Eigen::Vector4d wheelVelocitiesVector(
    wheelVelocities.at("front_left"),
    wheelVelocities.at("front_right"),
    wheelVelocities.at("rear_left"),
    wheelVelocities.at("rear_right")
  );
  const Eigen::Vector3d bodyTwist = this->mecanumForwardKinematics() * wheelVelocitiesVector;
  // Convert to global frame
  const double thetaOld = this->currentState.theta;
  const double globalVX = bodyTwist(0) * std::cos(thetaOld) - bodyTwist(1) * std::sin(thetaOld);
  const double globalVY = bodyTwist(0) * std::sin(thetaOld) + bodyTwist(1) * std::cos(thetaOld);
  const double globalOmega = bodyTwist(2);
  // Predict new state
  const double xNew = this->currentState.x + globalVX * dt;
  const double yNew = this->currentState.y + globalVY * dt;
  const double thetaNew = this->currentState.theta + globalOmega * dt;
  RobotState predictedState(xNew, yNew, thetaNew, globalVX, globalVY, globalOmega);

  // Calculate the Jacobian (F)
  const auto F = this->systemModel.getStateTransitionMatrix(dt, 0, 0, thetaOld, globalVX, globalVY);

  // Update the state covariance with the process noise and state transition matrix
  const auto P = this->covariance.stateCovariance;
  const auto Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}

RobotState KalmanFilter::predictRockerBogieKinematicModel(const std::unordered_map<std::string, double>& wheelVelocities, const std::unordered_map<std::string, double>& wheelSteeringAngles, const long timestamp) {
  const double dt = timestamp - this->previousPredictionTimeStamp;
  if (dt <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Kinematic model prediction has non-positive timestamp difference. Skipping prediction");
    return this->currentState; // No update if the timestamp is not valid
  }

  // Expected velocity keys (6) and steering keys (6)
  for (const auto& k : ROCKER_BOGIE_WHEEL_NAMES) {
    if (wheelVelocities.find(k) == wheelVelocities.end()) {
      throw std::invalid_argument(std::string("Missing wheel velocity key for ROCKER_BOGIE: ") + k);
    }
    if (wheelSteeringAngles.find(k) == wheelSteeringAngles.end()) {
      throw std::invalid_argument(std::string("Missing wheel steering key for ROCKER_BOGIE: ") + k);
    }
  }

  // Calculate the robot body twist
  const Eigen::Vector3d bodyTwist = this->rockerBogieForwardKinematics(wheelVelocities, wheelSteeringAngles);
  // Convert to global frame
  const double thetaOld = this->currentState.theta;
  const double globalVX = bodyTwist(0) * std::cos(thetaOld) - bodyTwist(1) * std::sin(thetaOld);
  const double globalVY = bodyTwist(0) * std::sin(thetaOld) + bodyTwist(1) * std::cos(thetaOld);
  const double globalOmega = bodyTwist(2);

  // Predict new state
  const double xNew = this->currentState.x + globalVX * dt;
  const double yNew = this->currentState.y + globalVY * dt;
  const double thetaNew = this->currentState.theta + globalOmega * dt;
  RobotState predictedState(xNew, yNew, thetaNew, globalVX, globalVY, globalOmega);

  // Calculate the Jacobian (F)
  const auto F = this->systemModel.getStateTransitionMatrix(dt, 0, 0, thetaOld, globalVX, globalVY);

  // Update the state covariance with the process noise and state transition matrix
  const auto P = this->covariance.stateCovariance;
  const auto Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}
