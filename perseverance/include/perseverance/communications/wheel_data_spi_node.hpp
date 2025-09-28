#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <atomic>

#include "spi_device.h"
#include "wheel_data.h"

class WheelDataSpiNode : public rclcpp::Node {
public:
  WheelDataSpiNode();
  ~WheelDataSpiNode() = default;

private:
  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

  WheelDataPacket fromTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::string spi_dev_;
  int spi_speed_;
  int spi_mode_;
  std::unique_ptr<SpiDevice> spi_;
  std::atomic<uint32_t> seq_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};
