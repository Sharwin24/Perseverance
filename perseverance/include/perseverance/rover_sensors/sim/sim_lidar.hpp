#ifndef SIM_LIDAR_HPP_
#define SIM_LIDAR_HPP_

#include <cmath>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

// This node simulates a LiDAR sensor moving within a predefined square room.
// It performs two key functions:
// 1. Publishes a TF2 transform for the robot's pose (map -> base_link).
// 2. Publishes sensor_msgs/msg/LaserScan messages on the /scan topic by
//    calculating ray-intersections with the room's walls.

class SimulatedLidar : public rclcpp::Node {
public:
  SimulatedLidar();
  ~SimulatedLidar() = default;

private:
  // Publisher for the LaserScan messages.
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPublisher;

  // TF2 broadcaster to publish the robot's pose.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;


  // Timer to drive the main simulation loop.
  rclcpp::TimerBase::SharedPtr simulationTimer;
  double roomWidth;          // Width of the simulated room in meters.
  double roomHeight;         // Height of the simulated room in meters.
  double robotSpeed;         // Linear speed of the robot in m/s.
  double robotAngularSpeed;  // Angular speed of the robot in rad/s.
  double scanPublishRate;    // Rate to publish scans in Hz.
  int scanNumPoints;         // Number of points in each laser scan.
  double robotX;             // Robot's current x position.
  double robotY;             // Robot's current y position.
  double robotTheta;         // Robot's current orientation (yaw).
  rclcpp::Time lastUpdateTime;


  /**
   * @brief The main loop for the simulation, called by the timer.
   * It updates the robot's pose, generates a new scan, and publishes both.
   */
  void simulationStep();

  /**
   * @brief Updates the robot's position and orientation based on its speed.
   */
  void updateRobotPose();

  /**
   * @brief Generates a LaserScan message by performing ray casting against the room walls.
   * @return A populated LaserScan message.
   */
  sensor_msgs::msg::LaserScan generateLaserScan();

  /**
   * @brief Publishes the robot's current pose as a TF2 transform (map -> base_link).
   */
  void publishRobotTransform();
};

#endif // SIM_LIDAR_HPP_
