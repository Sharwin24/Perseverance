#ifndef PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_SENSOR_ADAPTERS_HPP
#define PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_SENSOR_ADAPTERS_HPP

#include <mutex>

#include "base_kalman_filter.hpp"
#include "perseverance_kalman_filter.hpp"

// =============================================================================
// ImuAdapter
// =============================================================================

/**
 * @brief Feeds IMU measurements into the EKF.
 *
 * Owned measurement slots:
 *   Z[kMeasOmega] ← yaw rate from the gyroscope [rad/s]
 *   Z[kMeasTheta] ← fused yaw angle from the IMU orientation quaternion [rad]
 *
 * Owned control input slot:
 *   U[kAccel] ← longitudinal linear acceleration [m/s²]
 *
 * Call update() from the IMU subscriber callback:
 *   imuAdapter.update(msg->angular_velocity.z, msg->linear_acceleration.x,
 *                     tf2::getYaw(msg->orientation));
 */
class ImuAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  ImuAdapter() = default;

  /**
   * @brief Cache the latest IMU reading.
   * @param omega  Yaw rate from the gyroscope [rad/s]
   * @param accel  Longitudinal (+X) linear acceleration [m/s²]
   * @param yaw    Fused heading angle from the orientation quaternion [rad]
   */
  void update(double omega, double accel, double yaw) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedOmega = omega;
    cachedAccel = accel;
    cachedYaw = yaw;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[PerseveranceEKF::kMeasOmega] = cachedOmega;
    Z[PerseveranceEKF::kMeasTheta] = cachedYaw;
  }

  void populateControlInput(ControlInput& controlInput) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    controlInput[PerseveranceEKF::kAccel] = cachedAccel;
  }

  std::vector<int> ownedMeasurementIndices() const override {
    return {PerseveranceEKF::kMeasOmega, PerseveranceEKF::kMeasTheta};
  }

  std::vector<int> ownedControlIndices() const override {
    return {PerseveranceEKF::kAccel};
  }

private:
  mutable std::mutex dataLock;
  double cachedAccel{0.0};
  double cachedOmega{0.0};
  double cachedYaw{0.0};
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
 * @brief Feeds a single steering angle measurement and steering rate control input into the EKF.
 *
 * Instantiate once for front steering (kMeasDeltaF, kDeltaFRate) and once for rear
 * steering (kMeasDeltaR, kDeltaRRate). The constructor arguments determine which
 * measurement and control slots are owned.
 *
 * The steering rate is computed via finite-difference each time update() is called.
 * If no new data arrived since the last EKF tick, the control output is 0.0 to avoid
 * emitting a stale rate.
 *
 * Call update() from the steering subscriber callback:
 *   steeringAdapter.update(msg->drive.steering_angle, dt);
 */
class SteeringAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  /**
   * @param measurementIndex  The measurement vector index this adapter owns:
   *                          PerseveranceEKF::kMeasDeltaF for front steering,
   *                          PerseveranceEKF::kMeasDeltaR for rear  steering.
   * @param controlIndex      The control input index this adapter owns:
   *                          PerseveranceEKF::kDeltaFRate for front steering,
   *                          PerseveranceEKF::kDeltaRRate for rear  steering.
   */
  explicit SteeringAdapter(
    PerseveranceEKF::MeasurementIndex measurementIndex,
    PerseveranceEKF::ControlIndex controlIndex)
    : measurementIndex(measurementIndex), controlIndex(controlIndex) {}

  /**
   * @brief Cache the latest steering angle and compute the steering rate.
   * @param angle  Steering angle [rad]
   * @param dt     Time elapsed since last call [s]. If <= 0, rate computation is skipped.
   */
  void update(double angle, double dt) {
    std::lock_guard<std::mutex> lock(dataLock);
    if (dt > 0.0) {
      cachedRate = (angle - cachedAngle) / dt;
    }
    cachedAngle = angle;
    newDataAvailable = true;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[measurementIndex] = cachedAngle;
  }

  void populateControlInput(ControlInput& controlInput) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    controlInput[controlIndex] = newDataAvailable ? cachedRate : 0.0;
    newDataAvailable = false;
  }

  std::vector<int> ownedMeasurementIndices() const override {
    return {measurementIndex};
  }

  std::vector<int> ownedControlIndices() const override {
    return {controlIndex};
  }

private:
  mutable std::mutex dataLock;
  PerseveranceEKF::MeasurementIndex measurementIndex;
  PerseveranceEKF::ControlIndex controlIndex;
  double cachedAngle{0.0};
  double cachedRate{0.0};
  mutable bool newDataAvailable{false};
};


// =============================================================================
// OdomPoseAdapter
// =============================================================================

/**
 * @brief Dead-reckoning position adapter that integrates px and py from wheel odometry.
 *
 * Owned measurement slots:
 *   Z[kMeasPx] ← integrated x position [m]
 *   Z[kMeasPy] ← integrated y position [m]
 *
 * Uses the same bicycle kinematic model as the EKF motion model:
 *   β  = atan2(L_r·tan(δ_f) + L_f·tan(δ_r), L)
 *   ω  = v·(tan(δ_f) - tan(δ_r)) / L
 *   θ += ω·dt
 *   px += v·cos(θ + β)·dt
 *   py += v·sin(θ + β)·dt
 *
 * Note: θ is tracked internally for integration only and is not exposed to the EKF
 * measurement vector (kMeasTheta is owned by ImuHeadingAdapter).
 *
 * Call updateSpeed() from the wheel speed callback, updateFrontSteering() and
 * updateRearSteering() from the respective steering callbacks. Integration is
 * triggered on each updateSpeed() call.
 */
class OdomPoseAdapter : public SensorAdapter<MeasurementDimension, ControlDimension> {
public:
  /**
   * @param geometry  Rover geometry (axle distances, steering limits)
   */
  explicit OdomPoseAdapter(const PerseveranceEKF::RoverGeometry& geometry)
    : geometry(geometry) {}

  /**
   * @brief Cache the latest wheel speed and integrate position.
   * @param speed  Longitudinal body speed [m/s]
   * @param dt     Time since last call [s]. If <= 0, integration is skipped.
   */
  void updateSpeed(double speed, double dt) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedSpeed = speed;
    if (dt > 0.0) {
      integratePose(dt);
    }
  }

  /**
   * @brief Cache the latest front steering angle.
   * @param angle  Front steering angle [rad]
   */
  void updateFrontSteering(double angle) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedDeltaF = angle;
  }

  /**
   * @brief Cache the latest rear steering angle.
   * @param angle  Rear steering angle [rad]
   */
  void updateRearSteering(double angle) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedDeltaR = angle;
  }

  /**
   * @brief Reset the integrated pose. Call after construction to match the EKF initial state.
   * @param initialPx     Initial x position [m]
   * @param initialPy     Initial y position [m]
   * @param initialTheta  Initial heading [rad]
   */
  void resetPose(double initialPx, double initialPy, double initialTheta) {
    std::lock_guard<std::mutex> lock(dataLock);
    cachedPx = initialPx;
    cachedPy = initialPy;
    cachedTheta = initialTheta;
  }

  void populateMeasurementVector(MeasurementVector& Z) const override {
    std::lock_guard<std::mutex> lock(dataLock);
    Z[PerseveranceEKF::kMeasPx] = cachedPx;
    Z[PerseveranceEKF::kMeasPy] = cachedPy;
  }

  void populateControlInput(ControlInput& /*controlInput*/) const override {}

  std::vector<int> ownedMeasurementIndices() const override {
    return {PerseveranceEKF::kMeasPx, PerseveranceEKF::kMeasPy};
  }

  std::vector<int> ownedControlIndices() const override {
    return {};
  }

private:
  // Integrate position one step forward. MUST be called under dataLock.
  void integratePose(double dt) {
    const auto [nextPx, nextPy, nextTheta] = PerseveranceEKF::integratePose(
      PerseveranceEKF::StateVector{cachedPx, cachedPy, cachedTheta, cachedSpeed, cachedDeltaF, cachedDeltaR, 0.0},
      dt, geometry
    );
    cachedPx = nextPx;
    cachedPy = nextPy;
    cachedTheta = nextTheta;
  }

  mutable std::mutex dataLock;
  PerseveranceEKF::RoverGeometry geometry;
  double cachedSpeed{0.0};
  double cachedDeltaF{0.0};
  double cachedDeltaR{0.0};
  double cachedPx{0.0};
  double cachedPy{0.0};
  double cachedTheta{0.0};
};

#endif  // PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_SENSOR_ADAPTERS_HPP
