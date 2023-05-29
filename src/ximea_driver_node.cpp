#include "ximea_driver/ximea_driver.hpp"

using namespace trailbot;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XIMEADriver>();
  node->Run();
  rclcpp::shutdown();
  return 0;
}
