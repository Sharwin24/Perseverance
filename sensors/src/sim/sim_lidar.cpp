#include "sim/sim_lidar.hpp"

SimulatedLidar::SimulatedLidar() : Node("simulated_lidar") {
  RCLCPP_INFO(this->get_logger(), "LiDAR Simulator Node has been initialized.");

  // Declare and get parameters for the simulation
  this->declare_parameter("room_width", 10.0);
  this->declare_parameter("room_height", 10.0);
  this->declare_parameter("robot_speed", 0.5);
  this->declare_parameter("robot_angular_speed", 0.5);
  this->declare_parameter("scan_publish_rate", 10.0);
  this->declare_parameter("scan_num_points", 360);

  roomWidth = this->get_parameter("room_width").as_double();
  roomHeight = this->get_parameter("room_height").as_double();
  robotSpeed = this->get_parameter("robot_speed").as_double();
  robotAngularSpeed = this->get_parameter("robot_angular_speed").as_double();
  scanPublishRate = this->get_parameter("scan_publish_rate").as_double();
  scanNumPoints = this->get_parameter("scan_num_points").as_int();

  // Initialize the static transform broadcaster
  this->staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  {
    // Publish the transform from the base_link to the sensor link
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "lidar_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.3;
    transformStamped.transform.rotation.w = 1.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    this->staticTfBroadcaster->sendTransform(transformStamped);
  }

  // Initialize robot state
  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;
  lastUpdateTime = this->now();

  // Initialize TF broadcaster
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create publisher for the laser scan
  scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

  // Create a timer for the main simulation loop
  simulationTimer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / scanPublishRate),
    std::bind(&SimulatedLidar::simulationStep, this));
}

void SimulatedLidar::simulationStep() {
  updateRobotPose();
  publishRobotTransform();
  sensor_msgs::msg::LaserScan scan_msg = generateLaserScan();
  scanPublisher->publish(scan_msg);
}

void SimulatedLidar::updateRobotPose() {
  rclcpp::Time current_time = this->now();
  double dt = (current_time - lastUpdateTime).seconds();
  lastUpdateTime = current_time;

  // Update orientation
  robotTheta += robotAngularSpeed * dt;

  // Update position (simple circular motion)
  robotX += robotSpeed * cos(robotTheta) * dt;
  robotY += robotSpeed * sin(robotTheta) * dt;
}

void SimulatedLidar::publishRobotTransform() {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";

  t.transform.translation.x = robotX;
  t.transform.translation.y = robotY;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, robotTheta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tfBroadcaster->sendTransform(t);
}

sensor_msgs::msg::LaserScan SimulatedLidar::generateLaserScan() {
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp = this->now();
  scan_msg.header.frame_id = "lidar_link"; // Scan is relative to the lidar frame

  scan_msg.angle_min = -M_PI;
  scan_msg.angle_max = M_PI;
  scan_msg.angle_increment = 2.0 * M_PI / scanNumPoints;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 1.0 / scanPublishRate;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 25.0;

  scan_msg.ranges.resize(scanNumPoints);

  // Ray casting logic
  for (int i = 0; i < scanNumPoints; ++i) {
    double angle = scan_msg.angle_min + i * scan_msg.angle_increment;
    double global_angle = robotTheta + angle;

    // Calculate ray direction vectors
    double cos_ga = cos(global_angle);
    double sin_ga = sin(global_angle);

    double min_dist = scan_msg.range_max;

    // Intersection with top wall (y = roomHeight / 2)
    if (sin_ga > 1e-6) {
      double t = (roomHeight / 2.0 - robotY) / sin_ga;
      if (t > 0 && t < min_dist) min_dist = t;
    }

    // Intersection with bottom wall (y = -roomHeight / 2)
    if (sin_ga < -1e-6) {
      double t = (-roomHeight / 2.0 - robotY) / sin_ga;
      if (t > 0 && t < min_dist) min_dist = t;
    }

    // Intersection with right wall (x = roomWidth / 2)
    if (cos_ga > 1e-6) {
      double t = (roomWidth / 2.0 - robotX) / cos_ga;
      if (t > 0 && t < min_dist) min_dist = t;
    }

    // Intersection with left wall (x = -roomWidth / 2)
    if (cos_ga < -1e-6) {
      double t = (-roomWidth / 2.0 - robotX) / cos_ga;
      if (t > 0 && t < min_dist) min_dist = t;
    }

    scan_msg.ranges[i] = min_dist;
  }

  return scan_msg;
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedLidar>());
  rclcpp::shutdown();
  return 0;
}