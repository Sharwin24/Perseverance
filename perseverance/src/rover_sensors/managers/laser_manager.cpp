#include "rover_sensors/managers/laser_manager.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


LaserManager::LaserManager() : Node("laser_manager") {
  RCLCPP_INFO(this->get_logger(), "Laser Manager Node has been initialized.");
  // Declare parameters
  this->mapUpdateFrequency = this->declare_parameter("map_update_frequency", 10.0);
  this->mapResolution = this->declare_parameter("map_resolution", 0.05);
  this->mapWidth = this->declare_parameter("map_width", 1024);
  this->mapHeight = this->declare_parameter("map_height", 1024);
  this->mapFrameId = this->declare_parameter("map_frame_id", "map");
  this->odomFrameId = this->declare_parameter("odom_frame_id", "odom");
  this->baseFrameId = this->declare_parameter("base_frame_id", "base_link");
  // Read from yaml file
  this->mapUpdateFrequency = this->get_parameter("map_update_frequency").as_double();
  this->mapResolution = this->get_parameter("map_resolution").as_double();
  this->mapWidth = this->get_parameter("map_width").as_int();
  this->mapHeight = this->get_parameter("map_height").as_int();
  this->mapFrameId = this->get_parameter("map_frame_id").as_string();
  this->odomFrameId = this->get_parameter("odom_frame_id").as_string();
  this->baseFrameId = this->get_parameter("base_frame_id").as_string();

  // Initialize the map
  this->initializeMap();

  // Setup TF Listener and buffer
  this->tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  // Create a QoS profile suitable for receiving LaserScan every 10Hz
  auto laserQoS = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  // Subscribe to the laser scan topic
  this->laserSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", laserQoS, std::bind(&LaserManager::laserScanCallback, this, std::placeholders::_1)
  );

  // Setup a QoS profile suitable for publishing OccupancyGrid messages
  auto mapQoS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  // Setup map publisher
  this->mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", mapQoS);

  // Setup timer to publish the current map
  this->mapTimer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / mapUpdateFrequency),
    [this]() {
    this->currentMap.header.stamp = this->now();
    this->mapPublisher->publish(this->currentMap);
  }
  );
}

void LaserManager::initializeMap() {
  this->currentMap.header.frame_id = this->mapFrameId;
  this->currentMap.info.resolution = this->mapResolution;
  this->currentMap.info.width = this->mapWidth;
  this->currentMap.info.height = this->mapHeight;
  this->currentMap.info.origin.position.x = -(this->mapWidth / 2.0) * this->mapResolution;
  this->currentMap.info.origin.position.y = -(this->mapHeight / 2.0) * this->mapResolution;
  this->currentMap.info.origin.position.z = 0.0;
  this->currentMap.info.origin.orientation.w = 1.0;
  this->currentMap.data.resize(this->mapWidth * this->mapHeight, -1); // Unknown cells everywhere
  RCLCPP_INFO(this->get_logger(), "Map initialized with resolution: %.2f, size: %dx%d",
    this->mapResolution, this->mapWidth, this->mapHeight);
}

void LaserManager::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "Received laser scan data");
  geometry_msgs::msg::TransformStamped mapToLaserBase;
  try {
    mapToLaserBase = tfBuffer->lookupTransform(mapFrameId, msg->header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
      mapFrameId.c_str(), msg->header.frame_id.c_str(), ex.what());
    return;
  }
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = mapToLaserBase.transform.translation.x;
  robot_pose.position.y = mapToLaserBase.transform.translation.y;
  robot_pose.position.z = mapToLaserBase.transform.translation.z;
  robot_pose.orientation = mapToLaserBase.transform.rotation;
  this->updateMap(msg, robot_pose);
}

void LaserManager::updateMap(
  const sensor_msgs::msg::LaserScan::SharedPtr msg,
  const geometry_msgs::msg::Pose& robotPose) {
  int robotMapX, robotMapY;
  if (!worldToMap(robotPose.position.x, robotPose.position.y, robotMapX, robotMapY)) {
    RCLCPP_WARN(this->get_logger(), "Robot pose is out of map bounds");
    return;
  }
  double robotYaw = tf2::getYaw(robotPose.orientation);
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    if (std::isinf(range) || std::isnan(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }
    // From the polar coordinate at this range, get the cartesian coordinates
    const float laserAngle = msg->angle_min + i * msg->angle_increment;
    const float obstacleWorldX = robotPose.position.x + range * std::cos(robotYaw + laserAngle);
    const float obstacleWorldY = robotPose.position.y + range * std::sin(robotYaw + laserAngle);
    // Convert range point (world) to map coordinates
    int obstacleMapX, obstacleMapY;
    if (worldToMap(obstacleWorldX, obstacleWorldY, obstacleMapX, obstacleMapY)) {
      // Set the endpoint as occupied
      this->setMapCell(obstacleMapX, obstacleMapY, 100);
      // Mark cells from robot -> obstacle as free
      this->traceLine(robotMapX, robotMapY, obstacleMapX, obstacleMapY);
    }
  }
}

bool LaserManager::worldToMap(double worldX, double worldY, int& mapX, int& mapY) {
  // Convert world coordinates to map coordinates and account for the map's origin
  mapX = static_cast<int>((worldX - currentMap.info.origin.position.x) / mapResolution);
  mapY = static_cast<int>((worldY - currentMap.info.origin.position.y) / mapResolution);
  return (mapX >= 0 && mapX < mapWidth && mapY >= 0 && mapY < mapHeight);
}

void LaserManager::setMapCell(int mapX, int mapY, int8_t value) {
  if (mapX < 0 || mapX >= mapWidth || mapY < 0 || mapY >= mapHeight) {
    RCLCPP_WARN(this->get_logger(), "Map coordinates out of bounds: (%d, %d)", mapX, mapY);
    return;
  }
  currentMap.data[mapY * mapWidth + mapX] = value;
}

void LaserManager::traceLine(int x0, int y0, int x1, int y1) {
  // Bresenham's line algorithm to trace a line from (x0, y0) to (x1, y1)
  // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  // We don't want to overwrite the final endpoint, which is the obstacle
  while (true) {
    // Stop one cell short of the endpoint (the obstacle)
    if (x0 == x1 && y0 == y1) { break; }

    // Mark the current cell as free (0)
    this->setMapCell(x0, y0, 0);

    // Calculate the error term and update the error
    // for both x and y directions
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
