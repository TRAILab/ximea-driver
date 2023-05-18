#pragma once

#include <memory>

#include "m3api/xiApi.h"

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trailbot {

class XIMEADriver : public rclcpp::Node {
public:
  XIMEADriver();

  std::unique_ptr<sensor_msgs::msg::Image> GetImage();

private:
  HANDLE xi_handle_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}
