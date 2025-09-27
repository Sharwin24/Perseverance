#ifndef LASER_MANAGER_HPP_
#define LASER_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LaserManager : public rclcpp::Node {
public:
  LaserManager();
  ~LaserManager() = default;

private:
  double mapUpdateFrequency; // Frequency for publishing the map [Hz]
  double mapResolution; // meters per pixel
  int mapWidth;         // pixels
  int mapHeight;       // pixels
  std::string mapFrameId; // The coordinate frame of the map, e.g., "map"
  std::string odomFrameId; // The coordinate frame of the odometry, e.g., "odom"
  std::string baseFrameId; // The robot's base frame, e.g., "base_link"
  nav_msgs::msg::OccupancyGrid currentMap; // The occupancy grid map
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
  rclcpp::TimerBase::SharedPtr mapTimer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;

  /**
   * @brief Initializes the OccupancyGrid message with configured parameters.
   * Sets the resolution, size, origin, and fills the data vector with 'unknown' values.
   */
  void initializeMap();

  /**
   * @brief Callback function for processing laser scan messages.
   *
   * @param msg The laser scan message.
   */
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Converts world coordinates (in meters) to map grid coordinates (pixels).
   * @param worldX X-coordinate in the map frame.
   * @param worldY Y-coordinate in the map frame.
   * @param mapX Output map X-coordinate.
   * @param mapY Output map Y-coordinate.
   * @return true if the coordinate is within the map bounds, false otherwise.
   */
  bool worldToMap(double worldX, double worldY, int& mapX, int& mapY);

  /**
   * @brief Sets the value of a cell in the map's data array.
   * @param mapX The x-coordinate of the cell.
   * @param mapY The y-coordinate of the cell.
   * @param value The value to set (0-100 for free/occupied, -1 for unknown).
   */
  void setMapCell(int mapX, int mapY, int8_t value);

  /**
   * @brief Implements Bresenham's line algorithm to trace a line between two points.
   * This is used to mark cells along a laser beam as "free."
   * @param x0 Starting x-coordinate in map cells.
   * @param y0 Starting y-coordinate in map cells.
   * @param x1 Ending x-coordinate in map cells.
   * @param y1 Ending y-coordinate in map cells.
   */
  void traceLine(int x0, int y0, int x1, int y1);

  /**
   * @brief Updates the map based on a single laser scan.
   * @param msg The laser scan message.
   * @param robotPose The pose of the robot in the map frame at the time of the scan.
   */
  void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr msg, const geometry_msgs::msg::Pose& robotPose);
};

#endif // !LASER_MANAGER_HPP_
