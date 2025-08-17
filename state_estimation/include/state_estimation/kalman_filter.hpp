#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/imu.hpp>
#include <unordered_map>
#include <string>
#include <rclcpp/time.hpp>

// MECANUM wheel names (4)
constexpr std::array<const char*, 4> MECANUM_WHEEL_NAMES = {
  "front_left", "front_right", "rear_left", "rear_right"
};

// ROCKER_BOGIE wheel names (6)
constexpr std::array<const char*, 6> ROCKER_BOGIE_WHEEL_NAMES = {
  "front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"
};

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

  /**
   * @brief Convert the RobotState to an Eigen vector.
   *
   * @note Not an eigen-vector, a Vector type from the Eigen Library
   *
   * @return Eigen::Vector<double, 6> The 6x1 state vector [x, y, theta, vx, vy, omega]
   */
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
  // The 6x6 state covariance matrix (P), describing the uncertainty of the state estimate
  Eigen::Matrix<double, 6, 6> stateCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  // The 6x6 process noise covariance matrix (Q), describing the uncertainty in the process model
  Eigen::Matrix<double, 6, 6> processNoiseCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  // The 3x3 odometry measurement noise covariance matrix (R_odom), describing the uncertainty in the odometry measurements
  Eigen::Matrix<double, 3, 3> odomMeasurementNoiseCovariance = Eigen::Matrix<double, 3, 3>::Identity();
  // The 1x1 IMU measurement noise covariance matrix (R_imu), describing the uncertainty in the IMU measurements
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

enum class KinematicModel {
  // 2 wheel differential drive
  DIFF_DRIVE,
  // 4 wheel omni-directional mecanum drive
  MECANUM,
  // 6 wheel rocker-bogie drive with the 4 corner wheels capable of independent steering
  ROCKER_BOGIE
};

enum class PredictionModel {
  // Uses the dynamics of the physical system (acceleration) to predict the state
  DYNAMIC,
  // Uses the kinematic model of the drivebase to predict the state
  KINEMATIC
};

struct SystemModel {
  Eigen::Matrix<double, 6, 6> stateTransitionMatrix = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 2> controlInputModel = Eigen::Matrix<double, 6, 2>::Zero();
  PredictionModel predModel = PredictionModel::DYNAMIC;
  KinematicModel kinematicModel = KinematicModel::DIFF_DRIVE;

  SystemModel() = default;
  SystemModel(PredictionModel predModel, KinematicModel kinematicModel) : predModel(predModel), kinematicModel(kinematicModel) {}

  Eigen::Matrix<double, 6, 6> getStateTransitionMatrix(
    const double dt,
    const double V = 0, const double W = 0, const double theta = 0,
    const double vX = 0, const double vY = 0) {
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
      break;
    }
    case PredictionModel::KINEMATIC: {
      switch (kinematicModel) {
      case KinematicModel::DIFF_DRIVE: {
        stateTransitionMatrix(0, 0) = 1.0; // dX/dx
        stateTransitionMatrix(0, 2) = -1.0 * V * std::sin(theta) * dt; // dX/dtheta
        stateTransitionMatrix(1, 1) = 1.0; // dY/dy
        stateTransitionMatrix(1, 2) = V * std::cos(theta) * dt; // dY/dtheta
        stateTransitionMatrix(2, 2) = 1.0; // dTheta/dtheta
        stateTransitionMatrix(3, 2) = -1.0 * V * std::sin(theta) * (W * dt); // dVx/dtheta
        stateTransitionMatrix(4, 2) = V * std::cos(theta) * (W * dt); // dVy/dtheta
        break;
      }
      case KinematicModel::MECANUM: {
        stateTransitionMatrix(0, 2) = (-vX * std::sin(theta) - vY * std::cos(theta)) * dt; // dX/dtheta
        stateTransitionMatrix(1, 2) = (vX * std::cos(theta) - vY * std::sin(theta)) * dt; // dY/dtheta
        stateTransitionMatrix(0, 3) = dt; // dX/dVx
        stateTransitionMatrix(1, 4) = dt; // dY/dVy
        stateTransitionMatrix(2, 5) = dt; // dTheta/dOmega
        break;
      }
      case KinematicModel::ROCKER_BOGIE: {
        stateTransitionMatrix(0, 2) = (-vX * std::sin(theta) - vY * std::cos(theta)) * dt; // dX/dtheta
        stateTransitionMatrix(1, 2) = (vX * std::cos(theta) - vY * std::sin(theta)) * dt; // dY/dtheta
        stateTransitionMatrix(3, 2) = -vX * std::sin(theta) - vY * std::cos(theta); // dVx/dtheta
        stateTransitionMatrix(4, 2) = vX * std::cos(theta) - vY * std::sin(theta); // dVy/dtheta
        break;
      }
      default:
        throw std::runtime_error("Unknown kinematic model type");
      }
      break;
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

struct KinematicModelInput {
  // The velocity [rad/s] of the left wheel (DIFF_DRIVE)
  double leftVelocity;
  // The velocity [rad/s] of the right wheel (DIFF_DRIVE)
  double rightVelocity;
  // The velocities [rad/s] of the wheels (MECANUM, ROCKER_BOGIE)
  // MECANUM keys: {"front_left", "front_right", "rear_left", "rear_right"}
  // ROCKER_BOGIE keys: {"front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"}
  std::unordered_map<std::string, double> wheelVelocities;

  // The steering angles [rad] of the steerable wheels (ROCKER_BOGIE only)
  // ROCKER_BOGIE keys: {"front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"}
  std::unordered_map<std::string, double> wheelSteeringAngles;

  // Timestamp of the commanded motion
  rclcpp::Time timestamp;
};

struct RobotConstants {
  // Wheel to Wheel distance along the Robot Y-axis [m]
  double wheelBase;
  // Wheel radius [m]
  double wheelRadius;
  // The back wheel to front wheel distance along the Robot X-axis [m]
  double trackWidth;
  // Map from wheel name to location
  // MECANUM keys: {"front_left", "front_right", "rear_left", "rear_right"}
  // ROCKER_BOGIE keys: {"front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"}
  std::unordered_map<std::string, Eigen::Vector2d> wheelLocations;

  RobotConstants() = default;
  RobotConstants(const double wheelBase, const double wheelRadius, const double trackWidth)
    : wheelBase(wheelBase), wheelRadius(wheelRadius), trackWidth(trackWidth) {
    wheelLocations["front_left"] = Eigen::Vector2d(-trackWidth / 2.0, wheelBase / 2.0);
    wheelLocations["front_right"] = Eigen::Vector2d(trackWidth / 2.0, wheelBase / 2.0);
    wheelLocations["rear_left"] = Eigen::Vector2d(-trackWidth / 2.0, -wheelBase / 2.0);
    wheelLocations["rear_right"] = Eigen::Vector2d(trackWidth / 2.0, -wheelBase / 2.0);
  }
};
class KalmanFilter {
public:
  KalmanFilter() = default;
  KalmanFilter(RobotConstants robotConstants, PredictionModel predModel, KinematicModel kinematicModel, Covariance cov, RobotState initialState)
    : robotConstants(robotConstants), predictionModel(predModel), kinematicModel(kinematicModel),
    systemModel(SystemModel(predModel, kinematicModel)), covariance(cov), currentState(initialState) {}

  RobotState predictDynamicModel(const sensor_msgs::msg::Imu& imu);

  RobotState predictKinematicModel(const KinematicModelInput& kinematicParams);

  Eigen::Matrix<double, 3, 6> odomMeasurementModel() const { return this->measurementModel.odom; }
  Eigen::Matrix<double, 1, 6> imuMeasurementModel() const { return this->measurementModel.imu; }
  Eigen::Matrix<double, 6, 6> stateCovariance() const { return this->covariance.stateCovariance; }
  Eigen::Matrix<double, 6, 6> processNoiseCovariance() const { return this->covariance.processNoiseCovariance; }
  Eigen::Matrix<double, 3, 3> odomMeasurementNoiseCovariance()  const { return this->covariance.odomMeasurementNoiseCovariance; }
  Eigen::Matrix<double, 1, 1> imuMeasurementNoiseCovariance() const { return this->covariance.imuMeasurementNoiseCovariance; }
  Eigen::Vector<double, 6> stateVector() const { return this->currentState.vec(); }
  PredictionModel getPredictionModel() const { return this->predictionModel; }

  void updateState(Eigen::Vector<double, 6> X) { this->currentState = RobotState(X); }
  void updateProcessNoiseCovariance(const Eigen::Matrix<double, 6, 6> P) { this->covariance.setProcessNoiseCovariance(P); }

private:
  // Robot constants (e.g., wheel base, wheel radius, track width)
  RobotConstants robotConstants;
  // The type of prediction model to use for this filter
  PredictionModel predictionModel;
  // The type of kinematic model to use for this filter
  KinematicModel kinematicModel;
  // The state transition matrix F and the control input matrix B
  SystemModel systemModel;
  // Measurement models for the IMU and Odometry sensors
  MeasurementModel measurementModel;
  // Covariances relevant for both the system states and the sensors
  Covariance covariance;
  // The current robot's state estimation
  RobotState currentState;
  // The timestamp of the previous prediction [s]
  rclcpp::Time previousPredictionTimeStamp = rclcpp::Time(0);

  double computeDeltaTime(const rclcpp::Time currentTime) const {
    const double dtSeconds = (currentTime - this->previousPredictionTimeStamp).seconds();
    return dtSeconds;
  }

  Eigen::Matrix<double, 3, 4> mecanumForwardKinematics() const {
    // Calculate the forward kinematics matrix for a mecanum drive robot
    const double R = this->robotConstants.wheelRadius;
    const double L = this->robotConstants.trackWidth / 2.0;
    const double W = this->robotConstants.wheelBase / 2.0;
    return (R / 4) * Eigen::Matrix<double, 3, 4>{
      {-1 / (L + W), 1 / (L + W), 1 / (L + W), -1 / (L + W)},
      {1, 1, 1, 1},
      {-1, 1, -1, 1}
    };
  }

  Eigen::Vector4d mecanumInverseKinematics(const double vX, const double vY, const double omega) const {
    // Calculate wheel velocities from robot body velocities
    const double R = this->robotConstants.wheelRadius;
    const double L = this->robotConstants.trackWidth / 2.0;
    const double W = this->robotConstants.wheelBase / 2.0;

    return (1 / R) * Eigen::Vector4d{
      vY - vX - (L + W) * omega, // Front left wheel
      vY + vX + (L + W) * omega, // Front right wheel
      vY - vX + (L + W) * omega, // Rear left wheel
      vY + vX - (L + W) * omega  // Rear right wheel
    };
  }

  Eigen::Vector3d rockerBogieForwardKinematics(
    const std::unordered_map<std::string, double>& wheelVelocities,
    const std::unordered_map<std::string, double>& wheelSteeringAngles) const {
    // Calculate the forward kinematics for a rocker-bogie drive robot
    // wheelVelocities: [front left, front right, middle left, middle right, rear left, rear right]
    // wheelSteeringAngles: [front left, front right, rear left, rear right]
    for (const auto& k : ROCKER_BOGIE_WHEEL_NAMES) {
      if (wheelVelocities.find(k) == wheelVelocities.end()) {
        throw std::invalid_argument(std::string("Missing wheel velocity key for ROCKER_BOGIE: ") + k);
      }
      if (wheelSteeringAngles.find(k) == wheelSteeringAngles.end()) {
        throw std::invalid_argument(std::string("Missing wheel steering key for ROCKER_BOGIE: ") + k);
      }
    }

    // Initialize Jacobian matrices
    Eigen::Matrix<double, 12, 3> A = Eigen::Matrix<double, 12, 3>::Zero();
    Eigen::Matrix<double, 12, 1> B = Eigen::Matrix<double, 12, 1>::Zero();
    // Initialize row index and body velocity
    unsigned int row = 0;
    // Iterate over wheel names
    for (const auto& wheelName : ROCKER_BOGIE_WHEEL_NAMES) {
      // Get the wheel position, velocity, and steering angles
      const Eigen::Vector2d& wheelPos = this->robotConstants.wheelLocations.at(wheelName);
      const double wheelVel = wheelVelocities.at(wheelName);
      const double wheelSteer = wheelSteeringAngles.at(wheelName);

      // Populate A matrix for current wheel
      // v_ix = v_bx - omega_b * y_i
      A(row, 0) = 1;
      A(row, 1) = 0;
      A(row, 2) = -wheelPos.y();
      // v_iy = v_by + omega_b * x_i
      A(row + 1, 0) = 0;
      A(row + 1, 1) = 1;
      A(row + 1, 2) = wheelPos.x();

      // Populate B matrix for current wheel
      B(row, 0) = wheelVel * std::cos(wheelSteer);
      B(row + 1, 0) = wheelVel * std::sin(wheelSteer);

      // Move to the next wheel (2 rows per wheel)
      row += 2;
    }
    Eigen::Vector3d twist; // [vx, vy, omega]
    // QR Least Squares
    auto qr = A.colPivHouseholderQr();
    qr.setThreshold(1e-10);
    if (qr.rank() == 3) {
      twist = qr.solve(B);
    }
    else {
      // Fallback: Complete Orthogonal Decomposition (min-norm)
      twist = A.completeOrthogonalDecomposition().solve(B);
    }
    return twist;
  }

  RobotState predictDiffDriveKinematicModel(const double leftVelocity, const double rightVelocity, const rclcpp::Time timestamp);

  RobotState predictMecanumKinematicModel(const std::unordered_map<std::string, double>& wheelVelocities, const rclcpp::Time timestamp);

  RobotState predictRockerBogieKinematicModel(
    const std::unordered_map<std::string, double>& wheelVelocities,
    const std::unordered_map<std::string, double>& wheelSteeringAngles,
    const rclcpp::Time timestamp);
};

#endif // !KALMAN_FILTER_HPP