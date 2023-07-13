#pragma once

#include <memory>

#include "m3api/xiApi.h"

#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trailbot {

class XIMEADriver : public rclcpp::Node {
public:
  XIMEADriver();
  void Run();

  std::unique_ptr<sensor_msgs::msg::Image> GetImage();

private:
  HANDLE xi_handle_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher pub_;
};

}
