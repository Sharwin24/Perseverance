#ifndef ALPHA_BETA_FILTER_HPP
#define ALPHA_BETA_FILTER_HPP

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

/**
 * @struct AlphaBetaFilter
 * @brief Struct for storing alpha-beta filter parameters and state.
 *
 * This struct contains the parameters and state variables for an alpha-beta filter,
 * which is used to filter sensor data and for this struct specifically, a floating point value.
 */
struct AlphaBetaFilter {
  /**
   * @brief Alpha value for filter
   *
   * @note Determines how much of the residual error is used to update the estimate.
   *
   */
  double alpha;

  /**
   * @brief Beta value for filter
   *
   * @note Determines how much of the residual error is used to update the rate estimate.
   */
  double beta;
  double estimate; // Current estimate
  double rateEstimate; // Current rate estimate
  double previousEstimate = 0; // Previous estimate
  double previousRateEstimate = 0; // Previous rate estimate
  rclcpp::Time prevMeasureTime = rclcpp::Time(0); // Time of the previous measurement

  explicit AlphaBetaFilter(double alpha = 0.5, double beta = 0.1)
    : alpha(alpha), beta(beta), estimate(0), rateEstimate(0) {}

  /**
   * @brief Apply the Alpha-Beta filter with new data
   *
   * @param Z A measurement
   * @param timestamp The timestamp of the measurement
   */
  void applyFilter(double Z, rclcpp::Time timestamp) {
    // Handle first measurement case
    if (this->previousEstimate == 0 && this->previousRateEstimate == 0) {
      this->prevMeasureTime = timestamp;
      this->previousEstimate = Z;
      this->estimate = Z; // Use current measurement as the initial estimate
      return; // Skip prediction for first measurement
    }

    // Update Timestep and save the previous measurement time
    double dt = (timestamp - this->prevMeasureTime).seconds();
    this->prevMeasureTime = timestamp;

    // Prediction Step
    this->alpha = this->previousEstimate + (this->rateEstimate * dt);

    // Update Step
    double residual = Z - this->estimate;
    this->estimate += this->alpha * residual;
    this->rateEstimate += this->beta * (residual / dt);

    // Update the filter state
    this->previousRateEstimate = this->rateEstimate;
    this->previousEstimate = this->estimate;
  }
};

struct IMUABFilter {
  AlphaBetaFilter linear_accel_x;
  AlphaBetaFilter linear_accel_y;
  AlphaBetaFilter linear_accel_z;
  AlphaBetaFilter angular_vel_x;
  AlphaBetaFilter angular_vel_y;
  AlphaBetaFilter angular_vel_z;
  AlphaBetaFilter orientation_x;
  AlphaBetaFilter orientation_y;
  AlphaBetaFilter orientation_z;

  IMUABFilter(double linAccelAlpha = 0.5, double linAccelBeta = 0.1,
    double angVelAlpha = 0.5, double angVelBeta = 0.1,
    double orientAlpha = 0.5, double orientBeta = 0.1)
    : linear_accel_x(linAccelAlpha, linAccelBeta),
    linear_accel_y(linAccelAlpha, linAccelBeta),
    linear_accel_z(linAccelAlpha, linAccelBeta),
    angular_vel_x(angVelAlpha, angVelBeta),
    angular_vel_y(angVelAlpha, angVelBeta),
    angular_vel_z(angVelAlpha, angVelBeta),
    orientation_x(orientAlpha, orientBeta),
    orientation_y(orientAlpha, orientBeta),
    orientation_z(orientAlpha, orientBeta) {}

  sensor_msgs::msg::Imu update(const sensor_msgs::msg::Imu& imu_msg) {
    this->linear_accel_x.applyFilter(imu_msg.linear_acceleration.x, imu_msg.header.stamp);
    this->linear_accel_y.applyFilter(imu_msg.linear_acceleration.y, imu_msg.header.stamp);
    this->linear_accel_z.applyFilter(imu_msg.linear_acceleration.z, imu_msg.header.stamp);
    this->angular_vel_x.applyFilter(imu_msg.angular_velocity.x, imu_msg.header.stamp);
    this->angular_vel_y.applyFilter(imu_msg.angular_velocity.y, imu_msg.header.stamp);
    this->angular_vel_z.applyFilter(imu_msg.angular_velocity.z, imu_msg.header.stamp);
    this->orientation_x.applyFilter(imu_msg.orientation.x, imu_msg.header.stamp);
    this->orientation_y.applyFilter(imu_msg.orientation.y, imu_msg.header.stamp);
    this->orientation_z.applyFilter(imu_msg.orientation.z, imu_msg.header.stamp);
    sensor_msgs::msg::Imu filtered_msg = imu_msg; // Create a copy of the original message
    filtered_msg.linear_acceleration.x = this->linear_accel_x.estimate;
    filtered_msg.linear_acceleration.y = this->linear_accel_y.estimate;
    filtered_msg.linear_acceleration.z = this->linear_accel_z.estimate;
    filtered_msg.angular_velocity.x = this->angular_vel_x.estimate;
    filtered_msg.angular_velocity.y = this->angular_vel_y.estimate;
    filtered_msg.angular_velocity.z = this->angular_vel_z.estimate;
    filtered_msg.orientation.x = this->orientation_x.estimate;
    filtered_msg.orientation.y = this->orientation_y.estimate;
    filtered_msg.orientation.z = this->orientation_z.estimate;
    return filtered_msg;
  }
};

struct MagABFilter {
  AlphaBetaFilter magnetic_field_x;
  AlphaBetaFilter magnetic_field_y;
  AlphaBetaFilter magnetic_field_z;

  explicit MagABFilter(double magFieldAlpha = 0.5, double magFieldBeta = 0.1)
    : magnetic_field_x(magFieldAlpha, magFieldBeta),
    magnetic_field_y(magFieldAlpha, magFieldBeta),
    magnetic_field_z(magFieldAlpha, magFieldBeta) {}

  sensor_msgs::msg::MagneticField update(const sensor_msgs::msg::MagneticField& mag_msg) {
    this->magnetic_field_x.applyFilter(mag_msg.magnetic_field.x, mag_msg.header.stamp);
    this->magnetic_field_y.applyFilter(mag_msg.magnetic_field.y, mag_msg.header.stamp);
    this->magnetic_field_z.applyFilter(mag_msg.magnetic_field.z, mag_msg.header.stamp);
    sensor_msgs::msg::MagneticField filtered_msg = mag_msg; // Create a copy of the original message
    filtered_msg.magnetic_field.x = this->magnetic_field_x.estimate;
    filtered_msg.magnetic_field.y = this->magnetic_field_y.estimate;
    filtered_msg.magnetic_field.z = this->magnetic_field_z.estimate;
    return filtered_msg;
  }
};

struct TempABFilter {
  AlphaBetaFilter temperature;

  explicit TempABFilter(double tempAlpha = 0.5, double tempBeta = 0.1)
    : temperature(tempAlpha, tempBeta) {}

  sensor_msgs::msg::Temperature update(const sensor_msgs::msg::Temperature& temp_msg) {
    this->temperature.applyFilter(temp_msg.temperature, temp_msg.header.stamp);
    sensor_msgs::msg::Temperature filtered_msg = temp_msg; // Create a copy of the original message
    filtered_msg.temperature = this->temperature.estimate;
    return filtered_msg;
  }
};

#endif // !ALPHA_BETA_FILTER_HPP
