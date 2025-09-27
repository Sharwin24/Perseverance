/**
 * @file bno055_node.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Header file containing class definition for BNO055Node that publishes sensor data
 * on ROS topics.
 * @date 2025-03-18
 *
 */

#ifndef _BNO055_NODE_HPP_
#define _BNO055_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "rover_sensors/drivers/bno055.hpp"
#include "rover_sensors/filters/alpha_beta_filter.hpp"

 /**
  * @class BNO055Node
  * @brief A ROS2 Node for facilitating sensor readings from the BNO055 IMU sensor
  * and publishing the IMU data, magnetic field data, and temperature data on topics.
  *
  */
class BNO055Node : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the BNO055Node class.
   */
  BNO055Node();

  /**
   * @brief Default Destructor for the BNO055Node class.
   */
  ~BNO055Node() = default;

private:
  BNO055 sensor;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_filtered_pub;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_filtered_pub;

  TempABFilter tempFilter;
  IMUABFilter imuFilter;
  MagABFilter magFilter;
};

#endif // _BNO055_NODE_HPP_