/// @file state_estimation.cpp
/// @brief State Estimation for mobile robot using Kalman Filter to fuse IMU and Odometry data
/// @author Sharwin Patil
/// @date 2026-03-21
///
/// PARAMETERS:
///   timer_frequency (float64) : Frequency of the timer for updating the state estimate [Hz]
///   initial_x       (float64) : Initial X position [m]
///   initial_y       (float64) : Initial Y position [m]
///   wheel_radius    (float64) : Wheel radius [m]
///   front_wheel_base (float64) : Middle Wheel to Front Wheel distance along the Robot Y-axis [m]
///   rear_wheel_base  (float64) : Middle Wheel to Rear Wheel distance along the Robot Y-axis [m]
///   base_to_wheel_height (float64) : Height of the robot base to the wheel center [m]
///   max_steering_front (float64) : Front wheel max steering angle [rad]
///   max_steering_rear (float64) : Rear wheel max steering angle [
///
/// TF TREE:
///   map (static, published once at startup)
///    └─ odom                     ← fixed at z=0, placed at initial robot pose
///        └─ base_footprint       ← estimated by this node (2D ground-plane projection, z=0)
///             └─ base_link_centered  ← published by this node (same x,y/yaw, z=base_height)
///                  └─ base_link      ← original onshape-to-robot frame (from URDF fixed joint)
///                       └─ wheels, sensors, etc.
///
/// PUBLISHERS:
///   /state_estimation/odom (nav_msgs::msg::Odometry): Filtered robot state
///
/// SUBSCRIBERS:
///   /sensors/raw/imu                     (sensor_msgs/Imu)                     → ImuAdapter
///   /sensors/raw/wheel/speed             (geometry_msgs/TwistStamped)           → WheelSpeedAdapter
///   /sensors/raw/wheel/steering/front    (ackermann_msgs/AckermannDriveStamped) → SteeringAdapter (front)
///   /sensors/raw/wheel/steering/rear     (ackermann_msgs/AckermannDriveStamped) → SteeringAdapter (rear)

#include "state_estimation/state_estimation.hpp"

const char IMUTopic[] = "/sensors/raw/imu";
const char WheelSpeedTopic[] = "/sensors/raw/wheel/speed";
const char FrontSteeringTopic[] = "/sensors/raw/wheel/steering/front";
const char RearSteeringTopic[] = "/sensors/raw/wheel/steering/rear";
const char RobotStateTopic[] = "/state_estimation/odom";

StateEstimator::StateEstimator()
  : Node("state_estimator") {
  RCLCPP_INFO(this->get_logger(), "State Estimator Node has been initialized.");

  // ── Parameters ─────────────────────────────────────────────────────────────
  double timerFreq = this->declare_parameter("timer_frequency", 1.0); // [Hz]
  double initialX = this->declare_parameter("initial_x", 0.0); // [m]
  double initialY = this->declare_parameter("initial_y", 0.0); // [m]
  double initialTheta = this->declare_parameter("initial_theta", 0.0); // [rad]
  double frontWheelBase = this->declare_parameter("front_wheel_base", 0.125); // [m]
  double rearWheelBase = this->declare_parameter("rear_wheel_base", 0.125); // [m]
  double wheelRadius = this->declare_parameter("wheel_radius", 0.035); // [m]
  double baseToWheelHeight = this->declare_parameter("base_to_wheel_height", 0.045); // [m]
  double maxSteeringFront = this->declare_parameter("max_steering_front", M_PI / 6.0); // [rad]
  double maxSteeringRear = this->declare_parameter("max_steering_rear", M_PI / 6.0); // [rad]
  // Get parameters from yaml config file
  timerFreq = this->get_parameter("timer_frequency").as_double();
  initialX = this->get_parameter("initial_x").as_double();
  initialY = this->get_parameter("initial_y").as_double();
  initialTheta = this->get_parameter("initial_theta").as_double();
  frontWheelBase = this->get_parameter("front_wheel_base").as_double();
  rearWheelBase = this->get_parameter("rear_wheel_base").as_double();
  wheelRadius = this->get_parameter("wheel_radius").as_double();
  baseToWheelHeight = this->get_parameter("base_to_wheel_height").as_double();
  maxSteeringFront = this->get_parameter("max_steering_front").as_double();
  maxSteeringRear = this->get_parameter("max_steering_rear").as_double();

  // ── Publisher ───────────────────────────────────────────────────────────────
  const auto stateEstimateQoS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  this->odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>(RobotStateTopic, stateEstimateQoS);

  // ── Subscriptions → sensor adapters ────────────────────────────────────────
  // Each callback extracts the relevant scalar(s) from the ROS message and
  // forwards them to the corresponding ROS-free adapter via update().
  const auto sensorQoS = rclcpp::QoS(rclcpp::SensorDataQoS());

  // ── Sensor adapter construction ─────────────────────────────────────────────
  imuAdapter = std::make_shared<ImuAdapter>();
  wheelSpeedAdapter = std::make_shared<WheelSpeedAdapter>();
  frontSteeringAdapter = std::make_shared<SteeringAdapter>(PerseveranceEKF::kMeasDeltaF);
  rearSteeringAdapter = std::make_shared<SteeringAdapter>(PerseveranceEKF::kMeasDeltaR);

  imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
    IMUTopic, sensorQoS,
    [this](sensor_msgs::msg::Imu::SharedPtr msg) {
    imuAdapter->update(msg->angular_velocity.z, msg->linear_acceleration.x);
  });

  wheelSpeedSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    WheelSpeedTopic, sensorQoS,
    [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    wheelSpeedAdapter->update(msg->twist.linear.x);
  });

  frontSteeringSub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    FrontSteeringTopic, sensorQoS,
    [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    frontSteeringAdapter->update(msg->drive.steering_angle);
  });

  rearSteeringSub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    RearSteeringTopic, sensorQoS,
    [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    rearSteeringAdapter->update(msg->drive.steering_angle);
  });

  // ── Kalman Filter setup ────────────────────────────────────────────────────
  const double deltaTime = 1.0 / timerFreq;
  this->kalmanFilter = std::make_unique<PerseveranceEKF>(
    PerseveranceEKF::RoverGeometry(
      frontWheelBase, rearWheelBase,
      baseToWheelHeight, wheelRadius,
      maxSteeringFront, maxSteeringRear
    ),
    deltaTime,
    PerseveranceEKF::AdapterList{imuAdapter, wheelSpeedAdapter, frontSteeringAdapter, rearSteeringAdapter}
  );
  // TODO(Sharwin): Tune these covariances
  this->kalmanFilter->initialize(
    PerseveranceEKF::StateVector{initialX, initialY, initialTheta, 0.0, 0.0, 0.0, 0.0},
    PerseveranceEKF::StateCovariance::Identity() * 0.0001, // initial state covariance
    PerseveranceEKF::ProcessNoise::Identity() * 0.01, // process noise covariance
    PerseveranceEKF::MeasurementCovariance::Identity() * 0.1  // measurement noise covariance
  );

  // ── Sensor adapter coverage check ─────────────────────────────────────────
  const auto coverage = this->kalmanFilter->checkAdapterCoverage();
  if (coverage.complete) {
    RCLCPP_INFO(this->get_logger(), "Sensor adapter coverage: all measurement and control indices covered.");
  }
  else {
    for (int idx : coverage.uncoveredMeasurementIndices) {
      RCLCPP_WARN(
        this->get_logger(),
        "Sensor adapter coverage: measurement index %d (%s) is uncovered.", idx, PerseveranceEKF::getMeasurementIndexName(idx).c_str()
      );
    }
    for (int idx : coverage.uncoveredControlIndices) {
      RCLCPP_WARN(
        this->get_logger(),
        "Sensor adapter coverage: control index %d (%s) is uncovered.", idx, PerseveranceEKF::getControlIndexName(idx).c_str()
      );
    }
  }

  const auto& initialState = this->kalmanFilter->getState();
  RCLCPP_INFO(
    this->get_logger(),
    "Initial state: [x=%.3f m, y=%.3f m, theta=%.3f rad, vx=%.3f m/s, delta_f=%.3f rad, delta_r=%.3f rad, omega=%.3f rad/s]",
    initialState(PerseveranceEKF::StateIndex::kPx),
    initialState(PerseveranceEKF::StateIndex::kPy),
    initialState(PerseveranceEKF::StateIndex::kTheta),
    initialState(PerseveranceEKF::StateIndex::kV),
    initialState(PerseveranceEKF::StateIndex::kDeltaF),
    initialState(PerseveranceEKF::StateIndex::kDeltaR),
    initialState(PerseveranceEKF::StateIndex::kOmega)
  );

  const auto& P = this->kalmanFilter->getStateCovariance();
  RCLCPP_INFO(
    this->get_logger(),
    "State covariance P: diag[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
    P(PerseveranceEKF::StateIndex::kPx, PerseveranceEKF::StateIndex::kPx),
    P(PerseveranceEKF::StateIndex::kPy, PerseveranceEKF::StateIndex::kPy),
    P(PerseveranceEKF::StateIndex::kTheta, PerseveranceEKF::StateIndex::kTheta),
    P(PerseveranceEKF::StateIndex::kV, PerseveranceEKF::StateIndex::kV),
    P(PerseveranceEKF::StateIndex::kDeltaF, PerseveranceEKF::StateIndex::kDeltaF),
    P(PerseveranceEKF::StateIndex::kDeltaR, PerseveranceEKF::StateIndex::kDeltaR),
    P(PerseveranceEKF::StateIndex::kOmega, PerseveranceEKF::StateIndex::kOmega)
  );

  const auto& Q = this->kalmanFilter->getProcessNoise();
  RCLCPP_INFO(
    this->get_logger(),
    "Process noise Q:    diag[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
    Q(PerseveranceEKF::StateIndex::kPx, PerseveranceEKF::StateIndex::kPx),
    Q(PerseveranceEKF::StateIndex::kPy, PerseveranceEKF::StateIndex::kPy),
    Q(PerseveranceEKF::StateIndex::kTheta, PerseveranceEKF::StateIndex::kTheta),
    Q(PerseveranceEKF::StateIndex::kV, PerseveranceEKF::StateIndex::kV),
    Q(PerseveranceEKF::StateIndex::kDeltaF, PerseveranceEKF::StateIndex::kDeltaF),
    Q(PerseveranceEKF::StateIndex::kDeltaR, PerseveranceEKF::StateIndex::kDeltaR),
    Q(PerseveranceEKF::StateIndex::kOmega, PerseveranceEKF::StateIndex::kOmega)
  );

  // ── Static TF: map → odom ──────────────────────────────────────────────────
  // odom is fixed at z=0 with the initial robot heading, representing the
  // starting pose of the robot in the map frame. It never moves after startup.
  this->staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = initialX;
    map_to_odom.transform.translation.y = initialY;
    map_to_odom.transform.translation.z = 0.0;   // odom is always at ground level
    map_to_odom.transform.rotation = this->yaw2Quaternion(initialTheta);
    this->staticTfBroadcaster->sendTransform(map_to_odom);
  }

  // ── Dynamic TF broadcaster: odom → base_footprint ─────────────────────────
  // base_footprint is the 2D ground-plane projection of base_link_centered.
  // robot_state_publisher then handles base_footprint → base_link_centered
  // from the URDF fixed joint.
  this->tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(deltaTime),
    std::bind(&StateEstimator::timerCallback, this));

}

// ── Timer: predict + publish ──────────────────────────────────────────────────

void StateEstimator::timerCallback() {
  const PerseveranceEKF::ControlInput U = this->kalmanFilter->buildControlInput();
  const PerseveranceEKF::MeasurementVector Z = this->kalmanFilter->buildMeasurementVector();

  this->kalmanFilter->predict(U);
  this->kalmanFilter->update(Z);

  const PerseveranceEKF::StateVector& predictedState = this->kalmanFilter->getState();

  RCLCPP_DEBUG(
    this->get_logger(),
    "State: [x=%.3f m, y=%.3f m, theta=%.3f rad, V=%.3f m/s, delta_f=%.3f rad, delta_r=%.3f rad, omega=%.3f rad/s]",
    predictedState(PerseveranceEKF::StateIndex::kPx),
    predictedState(PerseveranceEKF::StateIndex::kPy),
    predictedState(PerseveranceEKF::StateIndex::kTheta),
    predictedState(PerseveranceEKF::StateIndex::kV),
    predictedState(PerseveranceEKF::StateIndex::kDeltaF),
    predictedState(PerseveranceEKF::StateIndex::kDeltaR),
    predictedState(PerseveranceEKF::StateIndex::kOmega)
  );

  this->odomPublisher->publish(this->createOdomMessage(predictedState));
  this->tfBroadcaster->sendTransform(this->createOdomToBaseFootprintTF(predictedState));
  this->tfBroadcaster->sendTransform(this->createBaseLinkCenteredToBaseFootprintTF());
}

// ── Helper methods ────────────────────────────────────────────────────────────

geometry_msgs::msg::Quaternion StateEstimator::yaw2Quaternion(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

nav_msgs::msg::Odometry StateEstimator::createOdomMessage(const PerseveranceEKF::StateVector& state) {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = this->now();
  // odom is the fixed reference frame; base_footprint is the moving child frame.
  // base_footprint is the 2D ground projection — robot_state_publisher bridges
  // it to base_link_centered (and onward to base_link) via the URDF fixed joints.
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";

  msg.pose.pose.position.x = state(PerseveranceEKF::StateIndex::kPx);
  msg.pose.pose.position.y = state(PerseveranceEKF::StateIndex::kPy);
  msg.pose.pose.position.z = 0.0;   // base_footprint is always at ground level
  msg.pose.pose.orientation = this->yaw2Quaternion(state(PerseveranceEKF::StateIndex::kTheta));

  msg.twist.twist.linear.x = state(PerseveranceEKF::StateIndex::kV);
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.angular.z = state(PerseveranceEKF::StateIndex::kOmega);

  const auto& P = this->kalmanFilter->getStateCovariance();

  std::array<double, 36> pose_cov{};
  pose_cov[0] = P(PerseveranceEKF::StateIndex::kPx, PerseveranceEKF::StateIndex::kPx);  // x
  pose_cov[7] = P(PerseveranceEKF::StateIndex::kPy, PerseveranceEKF::StateIndex::kPy);  // y
  pose_cov[35] = P(PerseveranceEKF::StateIndex::kTheta, PerseveranceEKF::StateIndex::kTheta);  // theta

  std::array<double, 36> twist_cov{};
  twist_cov[0] = P(PerseveranceEKF::StateIndex::kV, PerseveranceEKF::StateIndex::kV);  // vx
  twist_cov[7] = 0.0;  // vy (not estimated, so set to zero)
  twist_cov[35] = P(PerseveranceEKF::StateIndex::kOmega, PerseveranceEKF::StateIndex::kOmega);  // omega

  msg.pose.covariance = pose_cov;
  msg.twist.covariance = twist_cov;

  return msg;
}

geometry_msgs::msg::TransformStamped StateEstimator::createBaseLinkCenteredToBaseFootprintTF() {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "base_footprint";
  tf.child_frame_id = "base_link_centered";

  // base_link_centered shares the same x/y/yaw as base_footprint.
  // The z offset accounts for chassis height above ground (including terrain elevation),
  // which is tracked via base_height and the current estimated z position.
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = this->kalmanFilter->getGeometry().baseToGround();
  tf.transform.rotation = this->yaw2Quaternion(0.0);  // no rotation relative to base_footprint

  return tf;
}

geometry_msgs::msg::TransformStamped StateEstimator::createOdomToBaseFootprintTF(const PerseveranceEKF::StateVector& state) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  // odom is the parent (fixed) frame; base_footprint is the child (moving) frame.
  // base_footprint is always at z=0 — the height of the chassis above ground is
  // captured by the base_footprint → base_link_centered joint in the URDF.
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";

  tf.transform.translation.x = state(PerseveranceEKF::StateIndex::kPx);
  tf.transform.translation.y = state(PerseveranceEKF::StateIndex::kPy);
  tf.transform.translation.z = 0.0;   // base_footprint is always at ground level
  tf.transform.rotation = this->yaw2Quaternion(state(PerseveranceEKF::StateIndex::kTheta));

  return tf;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}
