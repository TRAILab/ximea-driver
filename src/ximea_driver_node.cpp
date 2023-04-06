#include "ximea_driver/ximea_driver.hpp"

using namespace trailbot;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XIMEADriver>());
  rclcpp::shutdown();
  return 0;
}
