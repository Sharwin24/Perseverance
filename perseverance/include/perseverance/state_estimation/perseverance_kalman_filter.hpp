#pragma once

#include "base_kalman_filter.hpp"
#include <cmath>
#include <stdexcept>

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
class PerseveranceEKF : public BaseKalmanFilter<7, 7, 3> {
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

  enum MeasIndex {
    kMeasPx = 0,
    kMeasPy = 1,
    kMeasTheta = 2,
    kMeasV = 3,
    kMeasDeltaF = 4,
    kMeasDeltaR = 5,
    kMeasOmega = 6
  };

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
   */
  explicit PerseveranceEKF(const RoverGeometry& roverGeometry, double deltaTimeStep)
    : _geometry(roverGeometry), _dt(deltaTimeStep) {
    // Validate parameters
    if (_geometry.centerToFrontAxle <= 0.0 || _geometry.centerToRearAxle <= 0.0)
      throw std::invalid_argument("Axle distances centerToFrontAxle and centerToRearAxle must be positive.");
    if (_dt <= 0.0)
      throw std::invalid_argument("Timestep dt must be positive.");
    if (_geometry.maxSteeringFront <= 0.0 || _geometry.maxSteeringRear <= 0.0)
      throw std::invalid_argument("Steering limits must be positive.");
  }

  void setDt(double dt) { _dt = dt; }
  RoverGeometry getGeometry() const { return _geometry; }

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
  StateVector motionModel(const StateVector& state, const ControlInput& control) const override {
    const double px = state[kPx];
    const double py = state[kPy];
    const double theta = state[kTheta];
    const double v = state[kV];
    const double delta_f = state[kDeltaF];
    const double delta_r = state[kDeltaR];

    const double accel = control[kAccel];
    const double delta_f_rate = control[kDeltaFRate];
    const double delta_r_rate = control[kDeltaRRate];

    // Combined yaw rate from both steerable axles
    const double omega = computeYawRate(v, delta_f, delta_r);

    // Body slip angle β: nonzero when centerToFrontAxle ≠ centerToRearAxle or in crab/pivot configurations
    const double beta = computeSlipAngle(delta_f, delta_r);

    const double effectiveHeading = theta + beta;  // effective velocity heading

    StateVector next_state;
    next_state[kPx] = px + v * std::cos(effectiveHeading) * _dt;
    next_state[kPy] = py + v * std::sin(effectiveHeading) * _dt;
    next_state[kTheta] = theta + omega * _dt;
    next_state[kV] = v + accel * _dt;
    next_state[kDeltaF] = std::clamp(delta_f + delta_f_rate * _dt, -_geometry.maxSteeringFront, _geometry.maxSteeringFront);
    next_state[kDeltaR] = std::clamp(delta_r + delta_r_rate * _dt, -_geometry.maxSteeringRear, _geometry.maxSteeringRear);
    next_state[kOmega] = omega;  // propagate kinematic yaw rate; corrected by gyro update

    return next_state;
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
  StateTransition computeF(const StateVector& state, const ControlInput& /*control*/) const override {
    const double theta = state[kTheta];
    const double v = state[kV];
    const double delta_f = state[kDeltaF];
    const double delta_r = state[kDeltaR];

    const double tan_df = std::tan(delta_f);
    const double tan_dr = std::tan(delta_r);
    const double cos2_df = std::cos(delta_f) * std::cos(delta_f);
    const double cos2_dr = std::cos(delta_r) * std::cos(delta_r);

    const double beta = computeSlipAngle(delta_f, delta_r);
    const double effectiveHeading = theta + beta;
    const double cos_eff = std::cos(effectiveHeading);
    const double sin_eff = std::sin(effectiveHeading);

    // ── Partial derivatives of yaw rate ω w.r.t. states ──────────────────
    // ω = v·(tan(δ_f) - tan(δ_r)) / L
    const double& L = _geometry.getWheelbase();
    const double domega_dv = (tan_df - tan_dr) / L;
    const double domega_ddeltaf = v / (L * cos2_df);
    const double domega_ddeltar = -v / (L * cos2_dr);

    // ── Partial derivatives of slip angle β w.r.t. δ_f and δ_r ──────────
    // β = atan2(L_r·tan(δ_f) + L_f·tan(δ_r), L)
    // Let num = L_r·tan(δ_f) + L_f·tan(δ_r)
    // ∂β/∂δ_f = (centerToRearAxle/cos²(δ_f)) / (L² + num²) · 1
    // ∂β/∂δ_r = (centerToFrontAxle/cos²(δ_r)) / (L² + num²) · 1
    // (the L in the atan2 denominator is the constant, derivative of atan(num/L)
    //  w.r.t. num is L/(L²+num²))
    const double slip_num = _geometry.centerToRearAxle * tan_df + _geometry.centerToFrontAxle * tan_dr;
    const double atan_denom = L * L + slip_num * slip_num;

    const double dbeta_ddeltaf = (_geometry.centerToRearAxle / cos2_df) * L / atan_denom;
    const double dbeta_ddeltar = (_geometry.centerToFrontAxle / cos2_dr) * L / atan_denom;

    // ── Partial derivatives of position through the chain rule ────────────
    // px_next = px + v·cos(θ+β)·dt
    // ∂px/∂θ   = -v·sin(θ+β)·dt
    // ∂px/∂v   =  cos(θ+β)·dt
    // ∂px/∂δ_f = -v·sin(θ+β)·∂β/∂δ_f·dt   (β depends on δ_f)
    // ∂px/∂δ_r = -v·sin(θ+β)·∂β/∂δ_r·dt
    const double dpx_dtheta = -v * sin_eff * _dt;
    const double dpx_dv = cos_eff * _dt;
    const double dpx_ddeltaf = -v * sin_eff * dbeta_ddeltaf * _dt;
    const double dpx_ddeltar = -v * sin_eff * dbeta_ddeltar * _dt;

    const double dpy_dtheta = v * cos_eff * _dt;
    const double dpy_dv = sin_eff * _dt;
    const double dpy_ddeltaf = v * cos_eff * dbeta_ddeltaf * _dt;
    const double dpy_ddeltar = v * cos_eff * dbeta_ddeltar * _dt;

    // ── Assemble F ────────────────────────────────────────────────────────
    StateTransition F = StateTransition::Identity();

    // Position rows
    F(kPx, kTheta) = dpx_dtheta;
    F(kPx, kV) = dpx_dv;
    F(kPx, kDeltaF) = dpx_ddeltaf;
    F(kPx, kDeltaR) = dpx_ddeltar;

    F(kPy, kTheta) = dpy_dtheta;
    F(kPy, kV) = dpy_dv;
    F(kPy, kDeltaF) = dpy_ddeltaf;
    F(kPy, kDeltaR) = dpy_ddeltar;

    // Heading row: θ_next = θ + ω(v, δ_f, δ_r)·dt
    F(kTheta, kV) = domega_dv * _dt;
    F(kTheta, kDeltaF) = domega_ddeltaf * _dt;
    F(kTheta, kDeltaR) = domega_ddeltar * _dt;
    F(kTheta, kOmega) = _dt;  // θ also propagates through the stored ω state

    // Omega row: ω_next = ω(v, δ_f, δ_r) — same kinematic formula, not a random walk
    F(kOmega, kV) = domega_dv;
    F(kOmega, kDeltaF) = domega_ddeltaf;
    F(kOmega, kDeltaR) = domega_ddeltar;
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
  // =========================================================================
  // Internal Kinematic Helpers
  // =========================================================================

  /**
   * @brief Compute the combined yaw rate from both steerable axles.
   *   ω = v · (tan(δ_f) - tan(δ_r)) / L
   */
  double computeYawRate(double v, double delta_f, double delta_r) const {
    return v * (std::tan(delta_f) - std::tan(delta_r)) / _geometry.getWheelbase();
  }

  /**
   * @brief Compute the body slip angle β from both steering angles.
   *
   *   β = atan( (L_r·tan(δ_f) + L_f·tan(δ_r)) / L )
   *
   *   β = 0 when the rover drives straight or centerToFrontAxle=centerToRearAxle with δ_f=-δ_r (pure spin).
   *   β ≠ 0 in asymmetric crab or turning configurations.
   */
  double computeSlipAngle(double delta_f, double delta_r) const {
    const double slip_num = _geometry.centerToRearAxle * std::tan(delta_f) + _geometry.centerToFrontAxle * std::tan(delta_r);
    return std::atan2(slip_num, _geometry.getWheelbase());
  }

  RoverGeometry _geometry;    // vehicle geometry (axle distances, steering limits)
  double   _dt;          // integration timestep [s]
};
