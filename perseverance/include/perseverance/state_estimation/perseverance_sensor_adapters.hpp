#pragma once

#include <mutex>

#include "base_kalman_filter.hpp"
#include "perseverance_kalman_filter.hpp"

// =============================================================================
// ImuAdapter
// =============================================================================

/**
 * @brief Feeds IMU measurements into the EKF.
 *
 * Owned measurement slot:
 *   Z[kMeasOmega] ← yaw rate from the gyroscope [rad/s]
 *
 * Owned control input slot:
 *   U[kAccel] ← longitudinal linear acceleration [m/s²]
 *
 * Call update() from the IMU subscriber callback:
 *   imuAdapter.update(msg->angular_velocity.z, msg->linear_acceleration.x);
 */
class ImuAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  ImuAdapter() = default;

  /**
   * @brief Cache the latest IMU reading.
   * @param omega  Yaw rate from the gyroscope [rad/s]
   * @param accel  Longitudinal (+X) linear acceleration [m/s²]
   */
  void update(double omega, double accel) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedOmega = omega;
    cachedAccel = accel;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[PerseveranceEKF::kMeasOmega] = cachedOmega;
  }

  void populateControlInput(ControlInput& controlInput) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    controlInput[PerseveranceEKF::kAccel] = cachedAccel;
  }

  std::vector<int> ownedMeasurementIndices() const override {
    return {PerseveranceEKF::kMeasOmega};
  }

  std::vector<int> ownedControlIndices() const override {
    return {PerseveranceEKF::kAccel};
  }

private:
  mutable std::mutex dataLock;
  double cachedAccel{0.0};
  double cachedOmega{0.0};
};


// =============================================================================
// WheelSpeedAdapter
// =============================================================================

/**
 * @brief Feeds wheel odometry linear speed into the EKF.
 *
 * Owned measurement slot:
 *   Z[kMeasV] ← longitudinal body speed [m/s]
 *
 * Call update() from the wheel speed subscriber callback:
 *   wheelSpeedAdapter.update(msg->twist.linear.x);
 */
class WheelSpeedAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  WheelSpeedAdapter() = default;

  /**
   * @brief Cache the latest wheel speed reading.
   * @param speed  Longitudinal (+X) body speed [m/s]
   */
  void update(double speed) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedSpeed = speed;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[PerseveranceEKF::kMeasV] = cachedSpeed;
  }

  void populateControlInput(ControlInput& /*controlInput*/) const override {}

  std::vector<int> ownedMeasurementIndices() const override {
    return {PerseveranceEKF::kMeasV};
  }

  std::vector<int> ownedControlIndices() const override {
    return {};
  }

private:
  mutable std::mutex dataLock;
  double cachedSpeed{0.0};
};


// =============================================================================
// SteeringAdapter
// =============================================================================

/**
 * @brief Feeds a single steering angle measurement into the EKF.
 *
 * Instantiate once for front steering (kMeasDeltaF) and once for rear
 * steering (kMeasDeltaR). The measIndex constructor argument determines
 * which measurement slot is owned.
 *
 * Call update() from the steering subscriber callback:
 *   steeringAdapter.update(msg->drive.steering_angle);
 */
class SteeringAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  /**
   * @param measurementIndex  The measurement vector index this adapter owns:
   *                          PerseveranceEKF::kMeasDeltaF for front steering,
   *                          PerseveranceEKF::kMeasDeltaR for rear  steering.
   */
  explicit SteeringAdapter(PerseveranceEKF::MeasurementIndex measurementIndex)
    : measurementIndex(measurementIndex) {}

  /**
   * @brief Cache the latest steering angle.
   * @param angle  Steering angle [rad]
   */
  void update(double angle) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedAngle = angle;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[measurementIndex] = cachedAngle;
  }

  void populateControlInput(ControlInput& /*controlInput*/) const override {}

  std::vector<int> ownedMeasurementIndices() const override {
    return {measurementIndex};
  }

  std::vector<int> ownedControlIndices() const override {
    return {};
  }

private:
  mutable std::mutex dataLock;
  PerseveranceEKF::MeasurementIndex measurementIndex;
  double cachedAngle{0.0};
};
