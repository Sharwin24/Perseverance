#ifndef PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_KALMAN_FILTER_HPP
#define PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_KALMAN_FILTER_HPP

#include "base_kalman_filter.hpp"
#include <array>
#include <cmath>
#include <stdexcept>

constexpr int StateDimension = 7;  // [px, py, theta, v, delta_f, delta_r, omega]
constexpr int ControlDimension = 3;  // [accel_x, delta_f_rate, delta_r_rate]
constexpr int MeasurementDimension = 7;  // [px, py, theta, v, delta_f, delta_r, omega]

/**
 * @brief EKF for a 6-wheel Perseverance-style rover with front and rear Ackermann steering.
 *
 * ── Wheel Layout ─────────────────────────────────────────────────────────────
 *
 *        ┌──────────────────────────────┐
 *        │ [FL] ←centerToFrontAxle→ [FR]│   ← Front axle  (steerable, δ_f)
 *        │                              │
 *        │ [ML]   (fixed, δ=0) [MR]     │   ← Mid   axle  (fixed, parallel to body)
 *        │                              │
 *        │ [RL] ←centerToRearAxle→ [RR] │   ← Rear  axle  (steerable, δ_r)
 *        └──────────────────────────────┘
 *
 *   FL/FR: steerable front wheels       (virtual single front wheel at angle δ_f)
 *   ML/MR: fixed mid wheels             (constrained to straight-ahead, no steering state)
 *   RL/RR: steerable rear wheels        (virtual single rear wheel at angle δ_r)
 *
 *   All 6 wheels are assumed to be driven (independent wheel drive).
 *   The mid wheels act as a passive kinematic constraint on the ICR — they do not
 *   need an explicit steering state since δ_mid = 0 always.
 *
 * ── Coordinate Frame ─────────────────────────────────────────────────────────
 *
 *   Origin of the body frame is at the geometric center between all three axles.
 *
 *        ┌────────────────────────────────────────────────────────┐
 *        │    centerToFrontAxle  = center to front axle       [m] │
 *        │    centerToRearAxle  = center to rear axle         [m] │
 *        │    L    = centerToFrontAxle + centerToRearAxle     [m] │
 *        │    track = lateral distance between left/right wheels  │
 *        └────────────────────────────────────────────────────────┘
 *
 * ── Kinematic Model ──────────────────────────────────────────────────────────
 *
 *   With both front and rear axles steerable, the instantaneous center of
 *   rotation (ICR) is determined by both steering angles simultaneously.
 *   The combined yaw rate is:
 *
 *     ω = v · (tan(δ_f) - tan(δ_r)) / L
 *
 *   The sign convention: positive δ steers left (counterclockwise from above).
 *   Rear wheels steer opposite to front for minimum-radius turns (like a forklift),
 *   or same direction for crabbing (lateral translation).
 *
 *   Velocity of the body center projected onto the world frame:
 *     ẋ  = v · cos(θ + β)
 *     ẏ  = v · sin(θ + β)
 *   where the slip angle β ≈ 0 for a symmetric configuration (centerToFrontAxle = centerToRearAxle), and:
 *     β  = arctan( (L_r·tan(δ_f) + L_f·tan(δ_r)) / L )
 *
 *   Discrete-time Euler integration over dt:
 *     px_next = px + v·cos(θ + β)·dt
 *     py_next = py + v·sin(θ + β)·dt
 *     θ_next  = θ  + ω·dt
 *     v_next  = v  + a·dt
 *     δ_f_next = clamp(δ_f + δ_f_rate·dt,  ±maxSteeringFront)
 *     δ_r_next = clamp(δ_r + δ_r_rate·dt,  ±maxSteeringRear)
 *     ω_next  = ω   (modeled as a random walk — updated directly by IMU gyro)
 *
 * ── State Vector (N = 7) ─────────────────────────────────────────────────────
 *   x = [ px,  py,  θ,  v,  δ_f,  δ_r,  ω ]ᵀ
 *         │    │    │   │   │     │     └── yaw rate (from gyro)       [rad/s]
 *         │    │    │   │   │     └──────── rear  steering angle       [rad]
 *         │    │    │   │   └────────────── front steering angle       [rad]
 *         │    │    │   └────────────────── longitudinal speed         [m/s]
 *         │    │    └────────────────────── heading (yaw) angle        [rad]
 *         │    └─────────────────────────── y position in world frame  [m]
 *         └──────────────────────────────── x position in world frame  [m]
 *
 *   ω is included as a state so IMU gyro measurements can directly correct it
 *   without passing through the kinematic chain. It provides a cross-check against
 *   the kinematic yaw rate v·(tan(δ_f)-tan(δ_r))/L in the measurement residual.
 *
 * ── Control Input (U = 3) ────────────────────────────────────────────────────
 *   u = [ a,  δ_f_rate,  δ_r_rate ]ᵀ
 *         │   │          └── rear  steering rate command               [rad/s]
 *         │   └──────────── front steering rate command                [rad/s]
 *         └──────────────── longitudinal acceleration command          [m/s²]
 *
 * ── Measurement Vector (M = 7) ───────────────────────────────────────────────
 *   z = [ px,  py,  θ,  v,  δ_f,  δ_r,  ω ]ᵀ
 *   Assumes: GPS (px, py), IMU magnetometer (θ), wheel odometry (v),
 *            front steering encoder (δ_f), rear steering encoder (δ_r),
 *            IMU gyroscope (ω).
 *   Unavailable sensors: set corresponding R diagonal entry to a large value (1e6).
 *
 * ── State Jacobian F = ∂f/∂x ─────────────────────────────────────────────────
 *
 *   Let:  β  = atan2(L_r·tan(δ_f) + L_f·tan(δ_r),  L)   [body slip angle]
 *         θ' = θ + β
 *
 *   F = I₇ with the following nonzero off-diagonal entries:
 *
 *         px  py   θ        v              δ_f                    δ_r                    ω
 *   px  [  1   0  -v·sin(θ')·dt   cos(θ')·dt   -v·sin(θ')·∂β/∂δ_f·dt  -v·sin(θ')·∂β/∂δ_r·dt  0 ]
 *   py  [  0   1   v·cos(θ')·dt   sin(θ')·dt    v·cos(θ')·∂β/∂δ_f·dt   v·cos(θ')·∂β/∂δ_r·dt  0 ]
 *   θ   [  0   0   1              ∂ω/∂v·dt       ∂ω/∂δ_f·dt             ∂ω/∂δ_r·dt             dt]
 *   v   [  0   0   0              1              0                       0                       0 ]
 *   δ_f [  0   0   0              0              1                       0                       0 ]
 *   δ_r [  0   0   0              0              0                       1                       0 ]
 *   ω   [  0   0   0              0              0                       0                       1 ]
 *
 *   Key partial derivatives:
 *     ∂ω/∂v     =  (tan(δ_f) - tan(δ_r)) / L
 *     ∂ω/∂δ_f   =   v / (L·cos²(δ_f))
 *     ∂ω/∂δ_r   =  -v / (L·cos²(δ_r))
 *     ∂β/∂δ_f   =  (centerToRearAxle / cos²(δ_f)) / (L² + (L_r·tan(δ_f) + L_f·tan(δ_r))²) · L
 *     ∂β/∂δ_r   =  (centerToFrontAxle / cos²(δ_r)) / (L² + (L_r·tan(δ_f) + L_f·tan(δ_r))²) · L
 */
class PerseveranceEKF : public BaseKalmanFilter<StateDimension, MeasurementDimension, ControlDimension> {
public:
  // -------------------------------------------------------------------------
  // Index enumerations to avoid magic number indexing
  // -------------------------------------------------------------------------

  enum StateIndex {
    kPx = 0,
    kPy = 1,
    kTheta = 2,
    kV = 3,
    kDeltaF = 4,   // front steering angle
    kDeltaR = 5,   // rear  steering angle
    kOmega = 6    // yaw rate (gyro state)
  };

  enum ControlIndex {
    kAccel = 0,
    kDeltaFRate = 1,  // front steering rate
    kDeltaRRate = 2   // rear  steering rate
  };

  enum MeasurementIndex {
    kMeasPx = 0,
    kMeasPy = 1,
    kMeasTheta = 2,
    kMeasV = 3,
    kMeasDeltaF = 4,
    kMeasDeltaR = 5,
    kMeasOmega = 6
  };

  static StateIndex stateIndexFromInt(int i) {
    if (i < 0 || i >= StateDimension) throw std::out_of_range("Invalid state index");
    return static_cast<StateIndex>(i);
  }

  static ControlIndex controlIndexFromInt(int i) {
    if (i < 0 || i >= ControlDimension) throw std::out_of_range("Invalid control index");
    return static_cast<ControlIndex>(i);
  }

  static MeasurementIndex measIndexFromInt(int i) {
    if (i < 0 || i >= MeasurementDimension) throw std::out_of_range("Invalid measurement index");
    return static_cast<MeasurementIndex>(i);
  }

  static std::string getStateIndexName(int i) { return getStateIndexName(stateIndexFromInt(i)); }
  static std::string getStateIndexName(StateIndex i) {
    switch (i) {
    case PerseveranceEKF::kPx: return "px";
    case PerseveranceEKF::kPy: return "py";
    case PerseveranceEKF::kTheta: return "theta";
    case PerseveranceEKF::kV: return "v";
    case PerseveranceEKF::kDeltaF: return "delta_f";
    case PerseveranceEKF::kDeltaR: return "delta_r";
    case PerseveranceEKF::kOmega: return "omega";
    default: return "unknown";
    }
  }

  static std::string getControlIndexName(int i) { return getControlIndexName(controlIndexFromInt(i)); }
  static std::string getControlIndexName(ControlIndex i) {
    switch (i) {
    case PerseveranceEKF::kAccel:      return "accel";
    case PerseveranceEKF::kDeltaFRate: return "delta_f_rate";
    case PerseveranceEKF::kDeltaRRate: return "delta_r_rate";
    default: return "unknown";
    }
  }

  static std::string getMeasurementIndexName(int i) { return getMeasurementIndexName(measIndexFromInt(i)); }
  static std::string getMeasurementIndexName(MeasurementIndex i) {
    switch (i) {
    case PerseveranceEKF::kMeasPx:    return "px";
    case PerseveranceEKF::kMeasPy:    return "py";
    case PerseveranceEKF::kMeasTheta: return "theta";
    case PerseveranceEKF::kMeasV:     return "v";
    case PerseveranceEKF::kMeasDeltaF: return "delta_f";
    case PerseveranceEKF::kMeasDeltaR: return "delta_r";
    case PerseveranceEKF::kMeasOmega: return "omega";
    default: return "unknown";
    }
  }

  // -------------------------------------------------------------------------
  // Vehicle geometry parameters
  // -------------------------------------------------------------------------
  struct RoverGeometry {
    double centerToFrontAxle;      // body center to front axle distance  [m]
    double centerToRearAxle;       // body center to rear  axle distance  [m]
    double maxSteeringFront;       // front steering angle physical limit [rad]
    double maxSteeringRear;        // rear  steering angle physical limit [rad]
    double baseToWheelHeight;      // nominal height of the base from the wheels [m]
    double wheelRadius;            // radius of the wheels [m]

    // L = centerToFrontAxle + centerToRearAxle
    double getWheelbase() const { return centerToFrontAxle + centerToRearAxle; }

    // The base to ground distance when the rover's differential suspension is at its nominal position
    double baseToGround() const { return baseToWheelHeight + wheelRadius; }

    RoverGeometry() = delete;
    RoverGeometry(
      double frontWheelBase, double rearWheelBase,
      double baseToWheelHeight, double wheelRadius,
      double maxSteeringFront, double maxSteeringRear)
      : centerToFrontAxle(frontWheelBase), centerToRearAxle(rearWheelBase),
      maxSteeringFront(maxSteeringFront), maxSteeringRear(maxSteeringRear),
      baseToWheelHeight(baseToWheelHeight), wheelRadius(wheelRadius) {}
  };

  /**
   * @brief Construct the rover EKF.
   * @note The user must still call initialize() with the initial state and covariance before using the filter.
   * @param roverGeometry  Vehicle geometry (axle distances, steering limits)
   * @param deltaTimeStep  Nominal integration timestep [s]
   * @param adapters       Array of 4 sensor adapters (ImuAdapter, WheelSpeedAdapter,
   *                       front SteeringAdapter, rear SteeringAdapter)
   */
  PerseveranceEKF(const RoverGeometry& roverGeometry, double deltaTimeStep, AdapterList adapters)
    : BaseKalmanFilter<7, 7, 3>(std::move(adapters)), geometry(roverGeometry), dt(deltaTimeStep) {
    // Validate parameters
    if (geometry.centerToFrontAxle <= 0.0 || geometry.centerToRearAxle <= 0.0)
      throw std::invalid_argument("Axle distances centerToFrontAxle and centerToRearAxle must be positive.");
    if (dt <= 0.0)
      throw std::invalid_argument("Timestep dt must be positive.");
    if (geometry.maxSteeringFront <= 0.0 || geometry.maxSteeringRear <= 0.0)
      throw std::invalid_argument("Steering limits must be positive.");
  }

  void setDt(double newDt) { dt = newDt; }
  RoverGeometry getGeometry() const { return geometry; }

  // =========================================================================
  // Public Static Kinematic Helpers
  // =========================================================================

  /**
   * @brief Compute the combined yaw rate from both steerable axles.
   *   ω = v · (tan(δ_f) - tan(δ_r)) / L
   */
  static double computeYawRate(double v, double deltaF, double deltaR, const RoverGeometry& geometry) {
    return v * (std::tan(deltaF) - std::tan(deltaR)) / geometry.getWheelbase();
  }

  /**
   * @brief Compute the body slip angle β from both steering angles.
   *   β = atan( (L_r·tan(δ_f) + L_f·tan(δ_r)) / L )
   */
  static double computeSlipAngle(double deltaF, double deltaR, const RoverGeometry& geometry) {
    const double slipNumerator =
      geometry.centerToRearAxle * std::tan(deltaF) +
      geometry.centerToFrontAxle * std::tan(deltaR);
    return std::atan2(slipNumerator, geometry.getWheelbase());
  }

  /**
   * @brief Integrate position one step forward using the rover bicycle kinematics.
   *
   * Computes the next (px, py, theta) given the current values, speed, steering
   * angles, and timestep. This is the position/heading sub-step of the full
   * motion model and can be called from outside the EKF (e.g. OdomPoseAdapter).
   *
   * @param currentState Current state vector [px, py, theta, v, delta_f, delta_r, omega]
   * @param dt      Integration timestep [s]
   * @param geometry  Rover geometry (axle distances)
   * @return Array of {nextPx, nextPy, nextTheta}
   */
  static std::array<double, 3> integratePose(const StateVector& currentState, double dt, const RoverGeometry& geometry) {
    const double beta = computeSlipAngle(currentState[kDeltaF], currentState[kDeltaR], geometry);
    const double omega = computeYawRate(currentState[kV], currentState[kDeltaF], currentState[kDeltaR], geometry);
    const double nextPx = currentState[kPx] + currentState[kV] * std::cos(currentState[kTheta] + beta) * dt;
    const double nextPy = currentState[kPy] + currentState[kV] * std::sin(currentState[kTheta] + beta) * dt;
    const double nextTheta = currentState[kTheta] + omega * dt;
    return {nextPx, nextPy, nextTheta};
  }

protected:
  // =========================================================================
  // Motion Model  f(x, u)
  // =========================================================================

  /**
   * @brief Kinematic motion model for the 6-wheel rover.
   *
   * Both front and rear axle steering angles contribute to the yaw rate and
   * to the body slip angle β. The mid fixed wheels impose no additional state —
   * their constraint is implicitly satisfied by the ICR geometry.
   */
  StateVector motionModel(const StateVector& state, const ControlInput& controlInput) const override {
    const double px = state[kPx];
    const double py = state[kPy];
    const double theta = state[kTheta];
    const double v = state[kV];
    const double deltaF = state[kDeltaF];
    const double deltaR = state[kDeltaR];

    const double accel = controlInput[kAccel];
    const double deltaFRate = controlInput[kDeltaFRate];
    const double deltaRRate = controlInput[kDeltaRRate];

    // Combined yaw rate from both steerable axles
    const double omega = computeYawRate(v, deltaF, deltaR, geometry);

    // Body slip angle β: nonzero when centerToFrontAxle ≠ centerToRearAxle or in crab/pivot configurations
    const double beta = computeSlipAngle(deltaF, deltaR, geometry);

    const double effectiveHeading = theta + beta;  // effective velocity heading

    StateVector nextState;
    nextState[kPx] = px + v * std::cos(effectiveHeading) * dt;
    nextState[kPy] = py + v * std::sin(effectiveHeading) * dt;
    nextState[kTheta] = theta + omega * dt;
    nextState[kV] = v + accel * dt;
    nextState[kDeltaF] = std::clamp(deltaF + deltaFRate * dt, -geometry.maxSteeringFront, geometry.maxSteeringFront);
    nextState[kDeltaR] = std::clamp(deltaR + deltaRRate * dt, -geometry.maxSteeringRear, geometry.maxSteeringRear);
    nextState[kOmega] = omega;  // propagate kinematic yaw rate; corrected by gyro update

    return nextState;
  }

  // =========================================================================
  // Measurement Model  h(x)
  // =========================================================================

  /**
   * @brief Direct state observation: h(x) = x, so H = I₇.
   *
   * All seven states are directly observable by the assumed sensor suite.
   * Disable unavailable sensors by setting R[i,i] to a large value (e.g. 1e6).
   */
  MeasurementVector measurementModel(const StateVector& state) const override {
    return state;  // h(x) = x  →  H = I₇
  }

  // =========================================================================
  // State Transition Jacobian  F = ∂f/∂x
  // =========================================================================

  /**
   * @brief Analytically derived Jacobian of the 6-wheel rover motion model.
   *
   * Linearizes the nonlinear contributions of θ, v, δ_f, δ_r on the
   * position and heading propagation. See class-level docstring for full layout.
   */
  StateTransition computeF(const StateVector& state, const ControlInput& /*controlInput*/) const override {
    const double theta = state[kTheta];
    const double v = state[kV];
    const double deltaF = state[kDeltaF];
    const double deltaR = state[kDeltaR];

    const double tanDeltaF = std::tan(deltaF);
    const double tanDeltaR = std::tan(deltaR);
    const double cos2DeltaF = std::cos(deltaF) * std::cos(deltaF);
    const double cos2DeltaR = std::cos(deltaR) * std::cos(deltaR);

    const double beta = computeSlipAngle(deltaF, deltaR, geometry);
    const double effectiveHeading = theta + beta;
    const double cosEff = std::cos(effectiveHeading);
    const double sinEff = std::sin(effectiveHeading);

    // ── Partial derivatives of yaw rate ω w.r.t. states ──────────────────
    // ω = v·(tan(δ_f) - tan(δ_r)) / L
    const double& L = geometry.getWheelbase();
    const double dOmegaDv = (tanDeltaF - tanDeltaR) / L;
    const double dOmegaDdeltaF = v / (L * cos2DeltaF);
    const double dOmegaDdeltaR = -v / (L * cos2DeltaR);

    // ── Partial derivatives of slip angle β w.r.t. δ_f and δ_r ──────────
    // β = atan2(L_r·tan(δ_f) + L_f·tan(δ_r), L)
    // Let num = L_r·tan(δ_f) + L_f·tan(δ_r)
    // ∂β/∂δ_f = (centerToRearAxle/cos²(δ_f)) / (L² + num²) · 1
    // ∂β/∂δ_r = (centerToFrontAxle/cos²(δ_r)) / (L² + num²) · 1
    // (the L in the atan2 denominator is the constant, derivative of atan(num/L)
    //  w.r.t. num is L/(L²+num²))
    const double slipNumerator = geometry.centerToRearAxle * tanDeltaF + geometry.centerToFrontAxle * tanDeltaR;
    const double atanDenom = L * L + slipNumerator * slipNumerator;

    const double dBetaDdeltaF = (geometry.centerToRearAxle / cos2DeltaF) * L / atanDenom;
    const double dBetaDdeltaR = (geometry.centerToFrontAxle / cos2DeltaR) * L / atanDenom;

    // ── Partial derivatives of position through the chain rule ────────────
    // px_next = px + v·cos(θ+β)·dt
    // ∂px/∂θ   = -v·sin(θ+β)·dt
    // ∂px/∂v   =  cos(θ+β)·dt
    // ∂px/∂δ_f = -v·sin(θ+β)·∂β/∂δ_f·dt   (β depends on δ_f)
    // ∂px/∂δ_r = -v·sin(θ+β)·∂β/∂δ_r·dt
    const double dPxDtheta = -v * sinEff * dt;
    const double dPxDv = cosEff * dt;
    const double dPxDdeltaF = -v * sinEff * dBetaDdeltaF * dt;
    const double dPxDdeltaR = -v * sinEff * dBetaDdeltaR * dt;

    const double dPyDtheta = v * cosEff * dt;
    const double dPyDv = sinEff * dt;
    const double dPyDdeltaF = v * cosEff * dBetaDdeltaF * dt;
    const double dPyDdeltaR = v * cosEff * dBetaDdeltaR * dt;

    // ── Assemble F ────────────────────────────────────────────────────────
    StateTransition F = StateTransition::Identity();

    // Position rows
    F(kPx, kTheta) = dPxDtheta;
    F(kPx, kV) = dPxDv;
    F(kPx, kDeltaF) = dPxDdeltaF;
    F(kPx, kDeltaR) = dPxDdeltaR;

    F(kPy, kTheta) = dPyDtheta;
    F(kPy, kV) = dPyDv;
    F(kPy, kDeltaF) = dPyDdeltaF;
    F(kPy, kDeltaR) = dPyDdeltaR;

    // Heading row: θ_next = θ + ω(v, δ_f, δ_r)·dt
    F(kTheta, kV) = dOmegaDv * dt;
    F(kTheta, kDeltaF) = dOmegaDdeltaF * dt;
    F(kTheta, kDeltaR) = dOmegaDdeltaR * dt;
    F(kTheta, kOmega) = dt;  // θ also propagates through the stored ω state

    // Omega row: ω_next = ω(v, δ_f, δ_r) — same kinematic formula, not a random walk
    F(kOmega, kV) = dOmegaDv;
    F(kOmega, kDeltaF) = dOmegaDdeltaF;
    F(kOmega, kDeltaR) = dOmegaDdeltaR;
    F(kOmega, kOmega) = 0.0;  // ω is fully recomputed from v and δ, not self-propagating

    return F;
  }

  // =========================================================================
  // Measurement Jacobian  H = ∂h/∂x
  // =========================================================================

  /**
   * @brief Jacobian of the direct observation model h(x) = x.
   *
   * Since all states are directly observable, H = I₇.
   * Override both measurementModel and computeH together if adapting to a
   * partial sensor suite (e.g., no GPS → zero out rows kMeasPx and kMeasPy,
   * and set R(kMeasPx,kMeasPx) and R(kMeasPy,kMeasPy) to large values).
   */
  MeasurementModel computeH(const StateVector& /*state*/) const override {
    return MeasurementModel::Identity();  // H = I₇
  }

private:
  RoverGeometry geometry;    // vehicle geometry (axle distances, steering limits)
  double dt;                 // integration timestep [s]
};

#endif  // PERSEVERANCE_STATE_ESTIMATION_PERSEVERANCE_KALMAN_FILTER_HPP
