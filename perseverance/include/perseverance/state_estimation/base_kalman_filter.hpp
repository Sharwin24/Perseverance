#pragma once

#include <eigen3/Eigen/Dense>

/**
 * @brief Abstract base class for an Extended Kalman Filter (EKF).
 *
 * Templated on the three fundamental dimensions of any EKF:
 *   N - State vector size        (x: N×1,  P: N×N,  F: N×N,  Q: N×N)
 *   M - Measurement vector size  (z: M×1,  H: M×N,  R: M×M,  K: N×M)
 *   U - Control input size       (u: U×1,  B: N×U)
 *
 * Derived classes must implement the nonlinear motion and measurement models
 * (motionModel, measurementModel) and their Jacobians (computeF, computeH).
 *
 * Usage:
 *   class MyEKF : public BaseKalmanFilter  <6, 3, 2> { ... };
 */
template <int N, int M, int U>
class BaseKalmanFilter {
public:
  // -------------------------------------------------------------------------
  // Eigen type aliases — sized by template parameters for clarity at call sites
  // -------------------------------------------------------------------------
  using StateVector = Eigen::Matrix<double, N, 1>;           // x:  N×1
  using StateCovariance = Eigen::Matrix<double, N, N>;       // P:  N×N
  using StateTransition = Eigen::Matrix<double, N, N>;       // F:  N×N  (Jacobian of f)
  using ProcessNoise = Eigen::Matrix<double, N, N>;          // Q:  N×N
  using ControlInput = Eigen::Matrix<double, U, 1>;          // u:  U×1
  using ControlMatrix = Eigen::Matrix<double, N, U>;         // B:  N×U
  using MeasurementVector = Eigen::Matrix<double, M, 1>;     // z:  M×1
  using MeasurementModel = Eigen::Matrix<double, M, N>;      // H:  M×N  (Jacobian of h)
  using MeasurementCovariance = Eigen::Matrix<double, M, M>; // R:  M×M
  using KalmanGain = Eigen::Matrix<double, N, M>;            // K:  N×M

  // -------------------------------------------------------------------------
  // Constructor / Destructor
  // -------------------------------------------------------------------------
  BaseKalmanFilter() = default;
  virtual ~BaseKalmanFilter() = default;

  // Non-copyable — state should be owned by a single filter instance
  BaseKalmanFilter(const BaseKalmanFilter&) = delete;
  BaseKalmanFilter& operator=(const BaseKalmanFilter&) = delete;

  // -------------------------------------------------------------------------
  // Filter Initialization
  // -------------------------------------------------------------------------

  /**
   * @brief Initialize the filter with a prior state estimate and covariance.
   * @param initialState  x₀: initial state estimate                     (N×1)
   * @param initialStateCovariance    P₀: initial state covariance       (N×N)
   * @param processNoise  Q:  process noise covariance                   (N×N)
   * @param measurementCovariance     R:  measurement noise covariance   (M×M)
   */
  void initialize(const StateVector& initialState,
    const StateCovariance& initialStateCovariance,
    const ProcessNoise& processNoise,
    const MeasurementCovariance& measurementCovariance) {
    currentState = initialState;
    P = initialStateCovariance;
    Q = processNoise;
    R = measurementCovariance;
    _initialized = true;
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
  void predict(const ControlInput& control_input) {
    // Linearize motion model around current estimate
    const StateTransition F = computeF(currentState, control_input);

    // Propagate state through the nonlinear motion model
    currentState = motionModel(currentState, control_input);

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
   *   P  = (I - K·H)·P⁻            [simplified form]
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
  const StateVector& getState()      const { return currentState; }
  const StateCovariance& getStateCovariance() const { return P; }
  const ProcessNoise& getProcessNoise() const { return Q; }
  const MeasurementCovariance& getMeasurementNoise() const { return R; }
  bool isInitialized() const { return _initialized; }
  int getStateDimension() const { return N; }
  int getMeasurementDimension() const { return M; }
  int getControlDimension() const { return U; }

protected:
  // -------------------------------------------------------------------------
  // Pure Virtual Interface — derived classes define the system model here
  // -------------------------------------------------------------------------

  /**
   * @brief Nonlinear motion model f(x, u).
   *
   * Propagates the state forward one timestep.
   * Used directly in the predict step to advance the state estimate.
   *
   * @param state          Current state estimate x     (N×1)
   * @param control_input  Control input u              (U×1)
   * @return               Predicted next state x⁻      (N×1)
   */
  virtual StateVector motionModel(const StateVector& state,
    const ControlInput& control_input) const = 0;

  /**
   * @brief Nonlinear measurement model h(x).
   *
   * Maps a state to the expected measurement space.
   * Used in the update step to compute the innovation z - h(x⁻).
   *
   * @param state  Predicted state estimate x⁻  (N×1)
   * @return       Expected measurement ẑ        (M×1)
   */
  virtual MeasurementVector measurementModel(const StateVector& state) const = 0;

  /**
   * @brief Jacobian of the motion model ∂f/∂x evaluated at (x, u).
   *
   * Linearizes f around the current estimate for covariance propagation.
   * F = ∂f/∂x |_(x̂, u)
   *
   * @param state          State to linearize around     (N×1)
   * @param control_input  Control input u               (U×1)
   * @return               State transition Jacobian F   (N×N)
   */
  virtual StateTransition computeF(const StateVector& state,
    const ControlInput& control_input) const = 0;

  /**
   * @brief Jacobian of the measurement model ∂h/∂x evaluated at x⁻.
   *
   * Linearizes h around the predicted state for the Kalman gain computation.
   * H = ∂h/∂x |_(x̂⁻)
   *
   * @param state  Predicted state to linearize around   (N×1)
   * @return       Measurement Jacobian H                (M×N)
   */
  virtual MeasurementModel computeH(const StateVector& state) const = 0;

  // -------------------------------------------------------------------------
  // Filter State (accessible to derived classes for custom initialization)
  // -------------------------------------------------------------------------
  StateVector currentState = StateVector::Zero();          // x:  N×1  current estimate
  StateCovariance P = StateCovariance::Identity();         // P:  N×N  estimate uncertainty
  ProcessNoise Q = ProcessNoise::Zero();                   // Q:  N×N  process noise
  MeasurementCovariance R = MeasurementCovariance::Zero(); // R:  M×M  measurement noise

private:
  bool _initialized = false;
};
