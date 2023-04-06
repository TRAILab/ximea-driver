#include "ximea_driver/ximea_driver.hpp"

namespace trailbot {

constexpr int kTimeoutMs = 100;

XIMEADriver::XIMEADriver() {
  auto stat = xiOpenDevice(0, &xi_handle_);
  stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, 10000);
  stat = xiStartAcquisition(xi_handle_);
}

std::unique_ptr<sensor_msgs::msg::Image> XIMEADriver::GetImage() {
  XI_IMG xi_image;
  auto stat = xiGetImage(xi_handle_, kTimeoutMs, &xi_image);

  if (stat != XI_OK) {
    return nullptr;
  }
}

}
