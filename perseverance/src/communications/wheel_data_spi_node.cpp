#include "communications/wheel_data_spi_node.hpp"

WheelDataSpiNode::WheelDataSpiNode() :
  Node("wheel_data_spi_node"),
  seq_(0) {

  // Declare parameters and get their values from YAML file
  this->spi_dev_ = this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
  this->spi_speed_ = this->declare_parameter<int>("spi_speed_hz", 8'000'000);
  this->spi_mode_ = this->declare_parameter<int>("spi_mode", 0);
  this->timer_freq = this->declare_parameter("timer_freq", 10);
  this->spi_dev_ = this->get_parameter("spi_device").as_string();
  this->spi_speed_ = this->get_parameter("spi_speed_hz").as_int();
  this->spi_mode_ = this->get_parameter("spi_mode").as_int();
  this->timer_freq = this->get_parameter("timer_freq").as_double();

  // Initialize SPI
  try {
    this->spi_ = std::make_unique<SpiDevice>(
      this->spi_dev_, (uint32_t)this->spi_speed_, (uint8_t)this->spi_mode_, 8
    );
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "SPI init failed: %s", e.what());
    throw;
  }

  sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&WheelDataSpiNode::onTwist, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Using SPI dev %s @ %d Hz, mode %d",
    spi_dev_.c_str(), spi_speed_, spi_mode_);

  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq),
    std::bind(&WheelDataSpiNode::timerCallback, this)
  );
}

void WheelDataSpiNode::timerCallback() {
  // Periodically read from the SPI to get the latest Encoder counts
  // and to send the latest wheel speeds and steering angles
}

WheelDataPacket WheelDataSpiNode::fromTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  WheelDataPacket pkt{};
  pkt.preamble = 0xA55A;
  pkt.version = 1;
  pkt.msg_type = 1;
  pkt.seq = this->seq_.fetch_add(1, std::memory_order_relaxed);

  // TODO(spatil): Use Kinematics for converting desired Twists to wheel commands

  // Compute CRC over everything except the crc field itself
  pkt.crc = 0;
  pkt.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&pkt),
    sizeof(WheelDataPacket) - sizeof(pkt.crc));

  return pkt;
}


void WheelDataSpiNode::onTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  WheelDataPacket pkt = fromTwist(msg);

  try {
    spi_->write(&pkt, sizeof(pkt));
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "SPI write failed: %s", e.what());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelDataSpiNode>());
  rclcpp::shutdown();
  return 0;
}
