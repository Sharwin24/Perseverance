#include "state_estimation/kalman_filter.hpp"

#include <array>

#include "rclcpp/rclcpp.hpp"

RobotState KalmanFilter::predictDynamicModel(const sensor_msgs::msg::Imu& imu) {
  const double dt = this->computeDeltaTime(imu.header.stamp);
  if (dt <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "IMU message has non-positive timestamp difference. Skipping prediction");
    return this->currentState; // No update if the timestamp is not valid
  }

  // Convert robot-frame acceleration to global-frame acceleration
  const double theta = this->currentState.theta;
  const double globalAccX = imu.linear_acceleration.x * std::cos(theta) - imu.linear_acceleration.y * std::sin(theta);
  const double globalAccY = imu.linear_acceleration.x * std::sin(theta) + imu.linear_acceleration.y * std::cos(theta);

  // Acquire State Transition matrix (F) and Control Input Model (B)
  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 2> B;
  this->systemModel.getStateTransitionMatrix(F, dt, 0, 0, 0, 0, 0);
  this->systemModel.getControlInputModel(dt, B);

  // Establish Control Vector (U)
  const auto& U = Eigen::Vector2d(globalAccX, globalAccY);

  // Predict the next state: x_pred = F * x + B * u
  Eigen::Vector<double, 6> predictedStateVector = F * this->currentState.vec() + B * U;
  assert(predictedStateVector.size() == 6 && "State vector must have 6 elements");
  RobotState predictedState(predictedStateVector);

  // Update the state covariance with the process noise and state transition matrix
  const auto& P = this->covariance.stateCovariance;
  const auto& Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = imu.header.stamp;

  // Return the newly predicted state
  return predictedState;
}

RobotState KalmanFilter::predictKinematicModel(const KinematicModelInput& kinematicParams) {
  switch (this->kinematicModel) {
  case KinematicModel::DIFF_DRIVE: {
    return this->predictDiffDriveKinematicModel(
      kinematicParams.leftVelocity, kinematicParams.rightVelocity,
      kinematicParams.timestamp
    );
  }
  case KinematicModel::MECANUM: {
    const std::array<double, 4> mecanumWheelVelocities = {
      kinematicParams.wheelVelocities[static_cast<size_t>(MecanumWheelID::MECANUM_FRONT_LEFT)],
      kinematicParams.wheelVelocities[static_cast<size_t>(MecanumWheelID::MECANUM_FRONT_RIGHT)],
      kinematicParams.wheelVelocities[static_cast<size_t>(MecanumWheelID::MECANUM_REAR_LEFT)],
      kinematicParams.wheelVelocities[static_cast<size_t>(MecanumWheelID::MECANUM_REAR_RIGHT)],
    };
    return this->predictMecanumKinematicModel(mecanumWheelVelocities, kinematicParams.timestamp);
  }
  case KinematicModel::ROCKER_BOGIE: {
    return this->predictRockerBogieKinematicModel(
      kinematicParams.wheelVelocities, kinematicParams.wheelSteeringAngles,
      kinematicParams.timestamp
    );
  }
  default: {
    throw std::runtime_error("Unknown kinematic model type.");
  }
  }
}


RobotState KalmanFilter::predictDiffDriveKinematicModel(
  const double leftVelocity, const double rightVelocity,
  const rclcpp::Time timestamp) {
  const double dt = this->computeDeltaTime(timestamp);
  if (dt <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("state_estimator"),
      "Kinematic model prediction has non-positive timestamp difference. Skipping prediction"
    );
    return this->currentState; // No update if the timestamp is not valid
  }

  // Calculate kinematic inputs V (forward) and W (angular)
  const double V = (leftVelocity + rightVelocity) / 2.0; // Average velocity
  const double W = (rightVelocity - leftVelocity) / this->robotConstants.wheelBase; // Angular velocity

  // Predict new state with non-linear state transition function
  const double xOld = this->currentState.x;
  const double yOld = this->currentState.y;
  const double thetaOld = this->currentState.theta;

  // Update state using kinematic equations
  const double xNew = xOld + V * std::cos(thetaOld) * dt;
  const double yNew = yOld + V * std::sin(thetaOld) * dt;
  const double thetaNew = thetaOld + W * dt;
  const double vXNew = V * std::cos(thetaNew);
  const double vYNew = V * std::sin(thetaNew);
  RobotState predictedState(xNew, yNew, thetaNew, vXNew, vYNew, W);

  // Calculate the Jacobian (F)
  Eigen::Matrix<double, 6, 6> F;
  this->systemModel.getStateTransitionMatrix(F, dt, V, W, thetaOld, 0, 0);

  // Update the state covariance with the process noise and state transition matrix
  const auto& P = this->covariance.stateCovariance;
  const auto& Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}

RobotState KalmanFilter::predictMecanumKinematicModel(
  const std::array<double, 4>& wheelVelocities,
  const rclcpp::Time timestamp) {
  const double dt = this->computeDeltaTime(timestamp);
  if (dt <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("state_estimator"),
      "Kinematic model prediction has non-positive timestamp difference. Skipping prediction"
    );
    return this->currentState; // No update if the timestamp is not valid
  }

  // Calculate the robot body twist
  Eigen::Map<const Eigen::Vector4d> wheelVelocitiesVector(wheelVelocities.data());
  const Eigen::Vector3d bodyTwist = this->mecanumForwardKinematicsMatrix * wheelVelocitiesVector;
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
  Eigen::Matrix<double, 6, 6> F;
  this->systemModel.getStateTransitionMatrix(F, dt, 0, 0, thetaOld, globalVX, globalVY);

  // Update the state covariance with the process noise and state transition matrix
  const auto& P = this->covariance.stateCovariance;
  const auto& Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}

RobotState KalmanFilter::predictRockerBogieKinematicModel(
  const std::array<double, 6>& wheelVelocities,
  const std::array<double, 6>& wheelSteeringAngles,
  const rclcpp::Time timestamp) {
  const double dt = this->computeDeltaTime(timestamp);
  if (dt <= 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("state_estimator"),
      "Kinematic model prediction has non-positive timestamp difference. Skipping prediction"
    );
    return this->currentState; // No update if the timestamp is not valid
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
  Eigen::Matrix<double, 6, 6> F;
  this->systemModel.getStateTransitionMatrix(F, dt, 0, 0, thetaOld, globalVX, globalVY);

  // Update the state covariance with the process noise and state transition matrix
  const auto& P = this->covariance.stateCovariance;
  const auto& Q = this->covariance.processNoiseCovariance;
  this->covariance.stateCovariance = F * P * F.transpose() + Q;

  // Update the previous prediction timestamp
  this->previousPredictionTimeStamp = timestamp;

  // Return the predicted state
  return predictedState;
}
