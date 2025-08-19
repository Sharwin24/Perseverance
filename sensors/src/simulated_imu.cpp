#include "simulated_imu.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedIMU>());
  rclcpp::shutdown();
  return 0;
}