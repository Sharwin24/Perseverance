#pragma once

#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

// =============================================================================
// SensorAdapter<M, U>
// =============================================================================

/**
 * @brief Abstract, ROS-free base for a sensor that contributes to the EKF
 *        measurement vector Z and/or control input U.
 *
 * Templated on M (measurement dimension) and U (control input dimension).
 * Each concrete adapter caches the latest reading from its sensor and exposes:
 *   - populateMeasurementVector() — writes owned measurement slots into Z
 *   - populateControlInput()      — writes owned control slots into U
 *
 * The owning ROS node is responsible for creating the ROS subscription and
 * calling the concrete adapter's typed update() method directly.
 *
 * ── How to implement a new sensor adapter ────────────────────────────────────
 *
 *  1. Inherit from SensorAdapter<M, U> with the same M and U as the filter.
 *
 *  2. Add a typed non-virtual update() method that accepts only the values
 *     this adapter needs (e.g., update(double omega, double accel)).
 *     Cache the values under a mutex.
 *
 *  3. Implement populateMeasurementVector(MeasurementVector& Z) and/or
 *     populateControlInput(ControlInput& U):
 *     Write cached values into the correct slots of Z or U.
 *     Leave all non-owned indices untouched.
 *
 *  4. In the ROS node subscription callback:
 *     Call adapter.update(<extracted scalars from msg>) directly.
 *
 * Example:
 *
 *   class GyroAdapter : public SensorAdapter<7, 3> {
 *   public:
 *     void update(double omega) {
 *       std::lock_guard lock(dataLock);
 *       cachedOmega = omega;
 *     }
 *     void populateMeasurementVector(MeasurementVector& Z) const override {
 *       std::lock_guard lock(dataLock);
 *       Z[PerseveranceEKF::kMeasOmega] = cachedOmega;
 *     }
 *     void populateControlInput(ControlInput&) const override {}
 *   private:
 *     mutable std::mutex mutex_;
 *     double omega_{0.0};
 *   };
 *
 *   // In the ROS node:
 *   imu_sub_ = node.create_subscription<Imu>(topic, qos,
 *     [this](Imu::SharedPtr msg) {
 *       gyroAdapter_.update(msg->angular_velocity.z);
 *     });
 */
template <int M, int U>
class SensorAdapter {
public:
  using MeasurementVector = Eigen::Matrix<double, M, 1>;
  using ControlInput = Eigen::Matrix<double, U, 1>;

  virtual ~SensorAdapter() = default;

  SensorAdapter(const SensorAdapter&) = delete;
  SensorAdapter& operator=(const SensorAdapter&) = delete;

  /**
   * @brief Write this adapter's owned measurement slots into Z.
   *
   * Called by BaseKalmanFilter::buildMeasurementVector(). Must leave all
   * non-owned indices of Z untouched.
   *
   * @param Z  Measurement vector to populate (M×1), initialised to zero by caller.
   */
  virtual void populateMeasurementVector(MeasurementVector& Z) const = 0;

  /**
   * @brief Write this adapter's owned control input slots into U.
   *
   * Called by BaseKalmanFilter::buildControlInput(). Must leave all non-owned
   * indices of U untouched.
   *
   * @param U  Control input vector to populate (U×1), initialised to zero by caller.
   */
  virtual void populateControlInput(ControlInput& controlInput) const = 0;

  /**
   * @brief Return the measurement vector indices this adapter owns.
   * @return Indices into [0, M). Empty if this adapter owns no measurement slots.
   */
  virtual std::vector<int> ownedMeasurementIndices() const = 0;

  /**
   * @brief Return the control input indices this adapter owns.
   * @return Indices into [0, U). Empty if this adapter owns no control input slots.
   */
  virtual std::vector<int> ownedControlIndices() const = 0;

protected:
  SensorAdapter() = default;
};


// =============================================================================
// BaseKalmanFilter<N, M, U>
// =============================================================================

/**
 * @brief Abstract base class for an Extended Kalman Filter (EKF).
 *
 * Templated on the three fundamental dimensions:
 *   N - State vector size        (x: N×1,  P: N×N,  F: N×N,  Q: N×N)
 *   M - Measurement vector size  (z: M×1,  H: M×N,  R: M×M,  K: N×M)
 *   U - Control input size       (u: U×1,  B: N×U)
 *
 * The sensor adapter set is passed at construction time as a
 * std::vector<std::shared_ptr<SensorAdapter<M,U>>>, allowing any number
 * of adapters without encoding the count in the type.
 *
 * Usage:
 *   class MyEKF : public BaseKalmanFilter<6, 3, 2> { ... };
 */
template <int N, int M, int U>
class BaseKalmanFilter {
public:
  // -------------------------------------------------------------------------
  // Eigen type aliases — sized by template parameters for clarity at call sites
  // -------------------------------------------------------------------------
  using StateVector = Eigen::Matrix<double, N, 1>;
  using StateCovariance = Eigen::Matrix<double, N, N>;
  using StateTransition = Eigen::Matrix<double, N, N>;
  using ProcessNoise = Eigen::Matrix<double, N, N>;
  using ControlInput = Eigen::Matrix<double, U, 1>;
  using ControlMatrix = Eigen::Matrix<double, N, U>;
  using MeasurementVector = Eigen::Matrix<double, M, 1>;
  using MeasurementModel = Eigen::Matrix<double, M, N>;
  using MeasurementCovariance = Eigen::Matrix<double, M, M>;
  using KalmanGain = Eigen::Matrix<double, N, M>;
  using AdapterList = std::vector<std::shared_ptr<SensorAdapter<M, U>>>;

  /**
   * @brief Result of checkAdapterCoverage().
   *
   * complete is true only when every measurement index [0, M) and every
   * control input index [0, U) is claimed by at least one adapter.
   * The two vectors name the gaps when complete is false.
   */
  struct SensorAdapterCoverageResult {
    bool complete;
    std::vector<int> uncoveredMeasurementIndices;
    std::vector<int> uncoveredControlIndices;
  };

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------

  /**
   * @brief Construct the filter with its set of sensor adapters.
   *
   * @param adapters  List of shared_ptr<SensorAdapter<M,U>>. Each element
   *                  must be non-null and must outlive the filter.
   */
  explicit BaseKalmanFilter(AdapterList adapters)
    : sensorAdapters(std::move(adapters)) {}

  virtual ~BaseKalmanFilter() = default;

  BaseKalmanFilter(const BaseKalmanFilter&) = delete;
  BaseKalmanFilter& operator=(const BaseKalmanFilter&) = delete;

  // -------------------------------------------------------------------------
  // Filter Initialization
  // -------------------------------------------------------------------------

  /**
   * @brief Initialize the filter with a prior state estimate and covariance.
   * @param initialState               x₀: initial state estimate  (N×1)
   * @param initialStateCovariance     P₀: initial state covariance (N×N)
   * @param processNoise               Q:  process noise covariance  (N×N)
   * @param measurementCovariance      R:  measurement noise covariance (M×M)
   */
  void initialize(const StateVector& initialState,
    const StateCovariance& initialStateCovariance,
    const ProcessNoise& processNoise,
    const MeasurementCovariance& measurementCovariance) {
    currentState = initialState;
    P = initialStateCovariance;
    Q = processNoise;
    R = measurementCovariance;
    initialized = true;
  }

  // -------------------------------------------------------------------------
  // EKF Predict Step
  // -------------------------------------------------------------------------

  /**
   * @brief Propagate the state and covariance forward using the motion model.
   *
   *   x⁻ = f(x, u)
   *   P⁻ = F·P·Fᵀ + Q
   *
   * @param control_input  u: control input vector (U×1)
   */
  void predict(const ControlInput& controlInput) {
    // Linearize motion model around current estimate
    const StateTransition F = computeF(currentState, controlInput);

    // Propagate state through the nonlinear motion model
    currentState = motionModel(currentState, controlInput);

    // Propagate covariance through the linearized model
    P = F * P * F.transpose() + Q;
  }

  // -------------------------------------------------------------------------
  // EKF Update Step
  // -------------------------------------------------------------------------

  /**
   * @brief Correct the predicted state using a new measurement.
   *
   *   K  = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
   *   x  = x⁻ + K·(z - h(x⁻))
   *   P  = (I - K·H)·P⁻
   *
   * @param measurement  z: raw measurement vector (M×1)
   */
  void update(const MeasurementVector& measurement) {
    // Linearize measurement model around the predicted state
    // M×N Jacobian
    const MeasurementModel H = computeH(currentState);

    // Innovation covariance [MxM] S = H·P⁻·Hᵀ + R
    const MeasurementCovariance S = H * P * H.transpose() + R;

    // Kalman gain [N×M] K = P⁻·Hᵀ·S⁻¹
    // ldlt() gives a numerically stable Cholesky decomposition for the solve
    const KalmanGain K = P * H.transpose() * S.ldlt().solve(
      MeasurementCovariance::Identity());

    // Innovation [Mx1]: difference between actual measurement and predicted measurement
    const MeasurementVector innovation = measurement - measurementModel(currentState);

    // State update
    currentState = currentState + K * innovation;

    // Covariance update — simplified form (I - KH)·P⁻
    const StateCovariance I = StateCovariance::Identity();
    P = (I - K * H) * P;
  }

  // -------------------------------------------------------------------------
  // State Accessors
  // -------------------------------------------------------------------------
  const StateVector& getState()                  const { return currentState; }
  const StateCovariance& getStateCovariance()    const { return P; }
  const ProcessNoise& getProcessNoise()          const { return Q; }
  const MeasurementCovariance& getMeasurementNoise() const { return R; }
  bool isInitialized()                           const { return initialized; }

  // -------------------------------------------------------------------------
  // Sensor Adapter Access
  // -------------------------------------------------------------------------

  /**
   * @brief Collect measurements from all adapters into a single Z vector.
   *
   * Calls populateMeasurementVector() on each adapter in array order.
   * Each adapter writes only its owned indices; all others remain zero.
   *
   * @return Populated measurement vector z (M×1).
   */
  MeasurementVector buildMeasurementVector() const {
    MeasurementVector Z = MeasurementVector::Zero();
    for (const auto& adapter : sensorAdapters) {
      adapter->populateMeasurementVector(Z);
    }
    return Z;
  }

  /**
   * @brief Collect control inputs from all adapters into a single U vector.
   *
   * Calls populateControlInput() on each adapter in array order.
   * Each adapter writes only its owned indices; all others remain zero.
   *
   * @return Populated control input vector u (U×1).
   */
  ControlInput buildControlInput() const {
    ControlInput controlInput = ControlInput::Zero();
    for (const auto& adapter : sensorAdapters) {
      adapter->populateControlInput(controlInput);
    }
    return controlInput;
  }

  /**
   * @brief Check that the adapter set collectively covers all M measurement
   *        indices and all U control input indices.
   *
   * @return SensorAdapterCoverageResult with complete=true if every index is claimed by at
   *         least one adapter, or complete=false with the uncovered indices listed.
   */
  SensorAdapterCoverageResult checkAdapterCoverage() const {
    std::vector<bool> measCovered(M, false);
    std::vector<bool> ctrlCovered(U, false);

    for (const auto& adapter : sensorAdapters) {
      for (int idx : adapter->ownedMeasurementIndices()) {
        measCovered[idx] = true;
      }
      for (int idx : adapter->ownedControlIndices()) {
        ctrlCovered[idx] = true;
      }
    }

    SensorAdapterCoverageResult result{true, {}, {}};
    for (int i = 0; i < M; ++i) {
      if (!measCovered[i]) {
        result.complete = false;
        result.uncoveredMeasurementIndices.push_back(i);
      }
    }
    for (int i = 0; i < U; ++i) {
      if (!ctrlCovered[i]) {
        result.complete = false;
        result.uncoveredControlIndices.push_back(i);
      }
    }
    return result;
  }

protected:
  // -------------------------------------------------------------------------
  // Pure Virtual Interface — derived classes define the system model here
  // -------------------------------------------------------------------------

  virtual StateVector motionModel(const StateVector& state, const ControlInput& controlInput) const = 0;

  virtual MeasurementVector measurementModel(const StateVector& state) const = 0;

  virtual StateTransition computeF(const StateVector& state, const ControlInput& controlInput) const = 0;

  virtual MeasurementModel computeH(const StateVector& state) const = 0;

  StateVector currentState = StateVector::Zero();          // Filtered State Estimate (N×1)
  StateCovariance P = StateCovariance::Identity();         // State Estimate Covariance (N×N)
  ProcessNoise Q = ProcessNoise::Zero();                   // Process Noise Covariance (N×N)
  MeasurementCovariance R = MeasurementCovariance::Zero(); // Measurement Noise Covariance (M×M)

private:
  bool initialized = false;
  AdapterList sensorAdapters;
};
