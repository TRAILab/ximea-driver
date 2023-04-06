#pragma once

#include <memory>

#include "m3api/xiApi.h"

#include "sensor_msgs/msg/image.hpp"

namespace trailbot {

class XIMEADriver {
public:
  XIMEADriver();

  std::unique_ptr<sensor_msgs::msg::Image> GetImage();

private:
  HANDLE xi_handle_;
};

}
