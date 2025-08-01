#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/imu.hpp>

struct RobotState {
  double x = 0.0; // X Position [m]
  double y = 0.0; // Y Position [m]
  double theta = 0.0; // Orientation [rad]
  double vx = 0.0; // X Velocity [m/s]
  double vy = 0.0; // Y Velocity [m/s]
  double omega = 0.0; // Angular Velocity [rad/s]

  RobotState() = default;
  RobotState(double x, double y, double theta, double vx, double vy, double omega)
    : x(x), y(y), theta(theta), vx(vx), vy(vy), omega(omega) {
    this->normalizeTheta();
  }
  RobotState(const Eigen::Vector<double, 6>& vec)
    : x(vec(0)), y(vec(1)), theta(vec(2)), vx(vec(3)), vy(vec(4)), omega(vec(5)) {
    this->normalizeTheta();
  }

  Eigen::Vector<double, 6> vec() const {
    return Eigen::Vector<double, 6>(x, y, theta, vx, vy, omega);
  }

  /**
   * @brief Normalize theta to the range [-pi, pi].
   *
   */
  void normalizeTheta() {
    theta = std::fmod(theta + M_PI, 2 * M_PI) - M_PI;
  }
};

struct Covariance {
  Eigen::Matrix<double, 6, 6> stateCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> processNoiseCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 3, 3> odomMeasurementNoiseCovariance = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 1, 1> imuMeasurementNoiseCovariance = Eigen::Matrix<double, 1, 1>::Identity();

  Covariance() = default;

  /**
   * @brief Set the uncertainties for the state covariance matrix.
   *
   * @param xUncertainty The uncertainty for the X position [m^2]
   * @param yUncertainty The uncertainty for the Y position [m^2]
   * @param thetaUncertainty The uncertainty for the orientation [rad^2]
   * @param vxUncertainty The uncertainty for the X velocity [m^2/s^2]
   * @param vyUncertainty The uncertainty for the Y velocity [m^2/s^2]
   * @param omegaUncertainty The uncertainty for the angular velocity [rad^2/s^2]
   */
  void setStateCovariance(double xUncertainty, double yUncertainty, double thetaUncertainty,
    double vxUncertainty, double vyUncertainty, double omegaUncertainty) {
    stateCovariance(0, 0) = xUncertainty; // X Position
    stateCovariance(1, 1) = yUncertainty; // Y Position
    stateCovariance(2, 2) = thetaUncertainty; // Orientation
    stateCovariance(3, 3) = vxUncertainty; // X Velocity
    stateCovariance(4, 4) = vyUncertainty; // Y Velocity
    stateCovariance(5, 5) = omegaUncertainty; // Angular Velocity
  }

  /**
   * @brief Set the process noise covariance matrix.
   *
   * @param xNoise The process noise for the X position [m^2]
   * @param yNoise The process noise for the Y position [m^2]
   * @param thetaNoise The process noise for the orientation [rad^2]
   * @param vxNoise The process noise for the X velocity [m^2/s^2]
   * @param vyNoise The process noise for the Y velocity [m^2/s^2]
   * @param omegaNoise The process noise for the angular velocity [rad^2/s^2]
   */
  void setProcessNoiseCovariance(double xNoise, double yNoise, double thetaNoise,
    double vxNoise, double vyNoise, double omegaNoise) {
    processNoiseCovariance(0, 0) = xNoise; // X Position
    processNoiseCovariance(1, 1) = yNoise; // Y Position
    processNoiseCovariance(2, 2) = thetaNoise; // Orientation
    processNoiseCovariance(3, 3) = vxNoise; // X Velocity
    processNoiseCovariance(4, 4) = vyNoise; // Y Velocity
    processNoiseCovariance(5, 5) = omegaNoise; // Angular Velocity
  }

  void setProcessNoiseCovariance(const Eigen::Matrix<double, 6, 6> P) {
    processNoiseCovariance = P;
  }

  /**
   * @brief Set the Odom Measurement Noise
   *
   * @param xNoise The measurement noise for the X position [m^2]
   * @param yNoise The measurement noise for the Y position [m^2]
   * @param thetaNoise The measurement noise for the orientation [rad^2]
   */
  void setOdomMeasurementNoiseCovariance(double xNoise, double yNoise, double thetaNoise) {
    odomMeasurementNoiseCovariance(0, 0) = xNoise; // X Position
    odomMeasurementNoiseCovariance(1, 1) = yNoise; // Y Position
    odomMeasurementNoiseCovariance(2, 2) = thetaNoise; // Orientation
  }

  /**
   * @brief Set the Imu Measurement Noise
   *
   * @param omegaNoise The measurement noise for the angular velocity [rad^2/s^2]
   */
  void setImuMeasurementNoiseCovariance(double omegaNoise) {
    imuMeasurementNoiseCovariance(0, 0) = omegaNoise; // Angular Velocity
  }
};

enum class PredictionModel {
  // Uses the dynamics of the physical system (acceleration) to predict the state
  DYNAMIC,
  // Uses the kinematic model of the diff-drive drivebase to predict the state
  KINEMATIC
};

struct SystemModel {
  Eigen::Matrix<double, 6, 6> stateTransitionMatrix = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 2> controlInputModel = Eigen::Matrix<double, 6, 2>::Zero();
  PredictionModel predModel = PredictionModel::DYNAMIC;

  SystemModel() = default;
  SystemModel(PredictionModel predModel) : predModel(predModel) {};

  Eigen::Matrix<double, 6, 6> getStateTransitionMatrix(const double dt, const double V = 0, const double W = 0, const double theta = 0) {
    switch (predModel) {
    case PredictionModel::DYNAMIC: {
      stateTransitionMatrix(0, 0) = 1.0;
      stateTransitionMatrix(1, 1) = 1.0;
      stateTransitionMatrix(2, 2) = 1.0;
      stateTransitionMatrix(3, 3) = 1.0;
      stateTransitionMatrix(4, 4) = 1.0;
      stateTransitionMatrix(5, 5) = 1.0;
      stateTransitionMatrix(0, 3) = dt; // dX/dVx
      stateTransitionMatrix(1, 4) = dt; // dY/dVy
      stateTransitionMatrix(2, 5) = dt; // dTheta/dOmega
    }
    case PredictionModel::KINEMATIC: {
      stateTransitionMatrix(0, 0) = 1.0; // dX/dx
      stateTransitionMatrix(0, 2) = -1.0 * V * std::sin(theta) * dt; // dX/dtheta
      stateTransitionMatrix(1, 1) = 1.0; // dY/dy
      stateTransitionMatrix(1, 2) = V * std::cos(theta) * dt; // dY/dtheta
      stateTransitionMatrix(2, 2) = 1.0; // dTheta/dtheta
      stateTransitionMatrix(3, 2) = -1.0 * V * std::sin(theta) * (W * dt); // dVx/dtheta
      stateTransitionMatrix(4, 2) = V * std::cos(theta) * (W * dt); // dVy/dtheta

    }
    }
    return stateTransitionMatrix;
  }

  Eigen::Matrix<double, 6, 2> getControlInputModel(const double dt) {
    const double dt2 = dt * dt;
    controlInputModel(0, 0) = 0.5 * (dt2); // dX/dax
    controlInputModel(1, 1) = 0.5 * (dt2); // dY/day
    controlInputModel(3, 0) = dt; // dVx/dax
    controlInputModel(4, 1) = dt; // dVy/day
    return controlInputModel;
  }
};

struct MeasurementModel {
  Eigen::Matrix<double, 3, 6> odom = Eigen::Matrix<double, 3, 6>::Zero();
  Eigen::Matrix<double, 1, 6> imu = Eigen::Matrix<double, 1, 6>::Zero();

  MeasurementModel() {
    odom(0, 0) = 1.0; // [x]
    odom(1, 1) = 1.0; // [y]
    odom(2, 2) = 1.0; // [theta]
    imu(0, 5) = 1.0; // [omega]
  }
};

class KalmanFilter {
public:
  KalmanFilter() = default;
  KalmanFilter(PredictionModel predModel, Covariance cov, RobotState initialState)
    : predictionModel(predModel), systemModel(SystemModel(predModel)), covariance(cov), currentState(initialState) {
  }

  RobotState predictDynamicModel(const sensor_msgs::msg::Imu imu, const double timestamp);

  RobotState predictKinematicModel(const double left_vel, const double right_vel, const double timestamp);

  Eigen::Matrix<double, 3, 6> odomMeasurementModel() const { return this->measurementModel.odom; }
  Eigen::Matrix<double, 1, 6> imuMeasurementModel() const { return this->measurementModel.imu; }
  Eigen::Matrix<double, 6, 6> stateCovariance() const { return this->covariance.stateCovariance; }
  Eigen::Matrix<double, 6, 6> processNoiseCovariance() const { return this->covariance.processNoiseCovariance; }
  Eigen::Matrix<double, 3, 3> odomMeasurementNoiseCovariance()  const { return this->covariance.odomMeasurementNoiseCovariance; }
  Eigen::Matrix<double, 1, 1> imuMeasurementNoiseCovariance() const { return this->covariance.imuMeasurementNoiseCovariance; }
  Eigen::Vector<double, 6> stateVector() const { return this->currentState.vec(); }

  void updateState(Eigen::Vector<double, 6> X) { this->currentState = RobotState(X); }
  void updateProcessNoiseCovariance(const Eigen::Matrix<double, 6, 6> P) {
    this->covariance.setProcessNoiseCovariance(P);
  }

private:
  // The type of prediction model to use for this filter
  PredictionModel predictionModel;

  // The state transition matrix F and the control input matrix B
  SystemModel systemModel;

  // Measurement models for the IMU and Odometry sensors
  MeasurementModel measurementModel;

  // Covariances relevant for both the system states and the sensors
  Covariance covariance;

  // The current robot's state estimation
  RobotState currentState;
};

#endif // !KALMAN_FILTER_HPP