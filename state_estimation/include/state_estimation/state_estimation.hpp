#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Dense>

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
  }
  RobotState(const Eigen::Vector<double, 6>& vec)
    : x(vec(0)), y(vec(1)), theta(vec(2)), vx(vec(3)), vy(vec(4)), omega(vec(5)) {
  }

  Eigen::Vector<double, 6> vec() const {
    return Eigen::Vector<double, 6>(x, y, theta, vx, vy, omega);
  }
};

struct Covariance {
  Eigen::Matrix<double, 6, 6> stateCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> processNoiseCovariance = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 3, 3> odomMeasurementNoiseCovariance = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 1, 1> imuMeasurementNoiseCovariance = Eigen::Matrix<double, 1, 1>::Identity();

  Covariance() = default;
  Covariance(
    const Eigen::Matrix<double, 6, 6>& stateCov,
    const Eigen::Matrix<double, 6, 6>& processNoiseCov,
    const Eigen::Matrix<double, 3, 3>& odomMeasNoiseCov,
    const Eigen::Matrix<double, 1, 1>& imuMeasNoiseCov)
    : stateCovariance(stateCov),
    processNoiseCovariance(processNoiseCov),
    odomMeasurementNoiseCovariance(odomMeasNoiseCov),
    imuMeasurementNoiseCovariance(imuMeasNoiseCov) {
  }

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

class StateEstimator : public rclcpp::Node {
public:
  StateEstimator();
  ~StateEstimator() = default;

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription;

  rclcpp::TimerBase::SharedPtr timer;

  RobotState currentState;
  Covariance covariance;
};

#endif // !STATE_ESTIMATION_HPP