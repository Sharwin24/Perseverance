#include "nodes/bno055_node.hpp"
#include "drivers/bno055.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#define I2C_DEVICE "/dev/i2c-1"

constexpr const char* IMU_TOPIC = "sensors/raw/imu";
constexpr const char* MAG_TOPIC = "sensors/raw/mag";
constexpr const char* TEMP_TOPIC = "sensors/raw/temp";
constexpr const char* IMU_FILTERED_TOPIC = "sensors/filtered/imu";
constexpr const char* MAG_FILTERED_TOPIC = "sensors/filtered/mag";
constexpr const char* TEMP_FILTERED_TOPIC = "sensors/filtered/temp";

BNO055Node::BNO055Node() : Node("BNO055_Sensor") {
  // Declare parameters
  std::string sensor_frame_id = this->declare_parameter("sensor_frame_id", "imu_link");
  double sensorPollFreq = this->declare_parameter("sensor_poll_freq", 100.0); // [Hz]
  double temp_filter_alpha = this->declare_parameter("temp_filter_alpha", 0.8);
  double temp_filter_beta = this->declare_parameter("temp_filter_beta", 0.01);
  double imu_filter_lin_accel_alpha = this->declare_parameter("imu_filter_lin_accel_alpha", 0.8);
  double imu_filter_lin_accel_beta = this->declare_parameter("imu_filter_lin_accel_beta", 0.01);
  double imu_filter_ang_vel_alpha = this->declare_parameter("imu_filter_ang_vel_alpha", 0.8);
  double imu_filter_ang_vel_beta = this->declare_parameter("imu_filter_ang_vel_beta", 0.01);
  double imu_filter_orient_alpha = this->declare_parameter("imu_filter_orient_alpha", 0.8);
  double imu_filter_orient_beta = this->declare_parameter("imu_filter_orient_beta", 0.01);
  double mag_filter_alpha = this->declare_parameter("mag_filter_alpha", 0.8);
  double mag_filter_beta = this->declare_parameter("mag_filter_beta", 0.01);
  // Get parameter from yaml file
  sensor_frame_id = this->get_parameter("sensor_frame_id").as_string();
  sensorPollFreq = this->get_parameter("sensor_poll_freq").as_double();
  temp_filter_alpha = this->get_parameter("temp_filter_alpha").as_double();
  temp_filter_beta = this->get_parameter("temp_filter_beta").as_double();
  imu_filter_lin_accel_alpha = this->get_parameter("imu_filter_lin_accel_alpha").as_double();
  imu_filter_lin_accel_beta = this->get_parameter("imu_filter_lin_accel_beta").as_double();
  imu_filter_ang_vel_alpha = this->get_parameter("imu_filter_ang_vel_alpha").as_double();
  imu_filter_ang_vel_beta = this->get_parameter("imu_filter_ang_vel_beta").as_double();
  imu_filter_orient_alpha = this->get_parameter("imu_filter_orient_alpha").as_double();
  imu_filter_orient_beta = this->get_parameter("imu_filter_orient_beta").as_double();
  mag_filter_alpha = this->get_parameter("mag_filter_alpha").as_double();
  mag_filter_beta = this->get_parameter("mag_filter_beta").as_double();

  // Initialize the AlphaBeta Filters
  this->tempFilter = TempABFilter(temp_filter_alpha, temp_filter_beta);
  this->imuFilter = IMUABFilter(imu_filter_lin_accel_alpha, imu_filter_lin_accel_beta,
    imu_filter_ang_vel_alpha, imu_filter_ang_vel_beta,
    imu_filter_orient_alpha, imu_filter_orient_beta);
  this->magFilter = MagABFilter(mag_filter_alpha, mag_filter_beta);

  // Initialize the BNO055 sensor
  try {
    this->sensor.init(I2C_DEVICE, BNO055_ADDRESS_A);
  }
  catch (const std::exception& e) {
    rclcpp::shutdown();
    throw std::runtime_error("BNO055 Sensor offline, shutting down node: " + std::string(e.what()));
  }

  RCLCPP_INFO(this->get_logger(), "BNO055 Sensor online!");

  // Initialize the publishers
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, qos);
  this->mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(MAG_TOPIC, qos);
  this->temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>(TEMP_TOPIC, qos);
  this->imu_filtered_pub = this->create_publisher<sensor_msgs::msg::Imu>(IMU_FILTERED_TOPIC, qos);
  this->mag_filtered_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(MAG_FILTERED_TOPIC, qos);
  this->temp_filtered_pub = this->create_publisher<sensor_msgs::msg::Temperature>(TEMP_FILTERED_TOPIC, qos);

  RCLCPP_INFO(this->get_logger(),
    "BNO055 Sensor publishing on topics: Raw: (%s), (%s), (%s) Filtered: (%s), (%s), (%s) at %.1f Hz",
    IMU_TOPIC, MAG_TOPIC, TEMP_TOPIC, IMU_FILTERED_TOPIC, MAG_FILTERED_TOPIC, TEMP_FILTERED_TOPIC, sensorPollFreq
  );

  // Create a timer to read the sensor and publish the data
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / sensorPollFreq),
    [this, sensor_frame_id]() -> void {
    IMURecord record = this->sensor.read();

    // IMU data
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = sensor_frame_id;
    imu_msg.linear_acceleration.x = record.raw_linear_acceleration_x;
    imu_msg.linear_acceleration.y = record.raw_linear_acceleration_y;
    imu_msg.linear_acceleration.z = record.raw_linear_acceleration_z;
    imu_msg.angular_velocity.x = record.raw_angular_velocity_x;
    imu_msg.angular_velocity.y = record.raw_angular_velocity_y;
    imu_msg.angular_velocity.z = record.raw_angular_velocity_z;
    imu_msg.orientation.w = record.fused_orientation_w;
    imu_msg.orientation.x = record.fused_orientation_x;
    imu_msg.orientation.y = record.fused_orientation_y;
    imu_msg.orientation.z = record.fused_orientation_z;

    // Magnetic field data
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = this->now();
    mag_msg.header.frame_id = sensor_frame_id;
    mag_msg.magnetic_field.x = record.raw_magnetic_field_x;
    mag_msg.magnetic_field.y = record.raw_magnetic_field_y;
    mag_msg.magnetic_field.z = record.raw_magnetic_field_z;

    // Temperature data
    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header.stamp = this->now();
    temp_msg.header.frame_id = sensor_frame_id;
    temp_msg.temperature = record.temperature;

    // Publish the data
    this->imu_pub->publish(imu_msg);
    this->mag_pub->publish(mag_msg);
    this->temp_pub->publish(temp_msg);

    // Filter the data and publish it as well
    const auto filteredTempMsg = this->tempFilter.update(temp_msg);
    const auto filteredImuMsg = this->imuFilter.update(imu_msg);
    const auto filteredMagMsg = this->magFilter.update(mag_msg);
    this->temp_filtered_pub->publish(filteredTempMsg);
    this->imu_filtered_pub->publish(filteredImuMsg);
    this->mag_filtered_pub->publish(filteredMagMsg);

  });
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<BNO055Node>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), e.what());
  }
  rclcpp::shutdown();
  return 0;
}