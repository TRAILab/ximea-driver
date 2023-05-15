#include "ximea_driver/ximea_driver.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace trailbot {

constexpr int kTimeoutMs = 100;

constexpr int kExposureMs = 50;
constexpr int kExposureUs = kExposureMs * 1000;

XIMEADriver::XIMEADriver() : Node("ximea_driver") {
  pub_ = create_publisher<sensor_msgs::msg::Image>("camera", 10);

  auto stat = xiOpenDevice(0, &xi_handle_);

  stat = xiSetParamInt(xi_handle_, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
  stat = xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);

  stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, kExposureUs);

  stat = xiStartAcquisition(xi_handle_);

  RCLCPP_INFO(get_logger(), "Starting image stream");

  while(rclcpp::ok()) {
    auto img = GetImage();
    if (img != nullptr) {
      pub_->publish(std::move(img));
    }
  }
}

std::unique_ptr<sensor_msgs::msg::Image> XIMEADriver::GetImage() {
  XI_IMG xi_image;
  memset(&xi_image, 0, sizeof(xi_image));
	xi_image.size = sizeof(XI_IMG);

  auto stat = xiGetImage(xi_handle_, kTimeoutMs, &xi_image);

  if (stat != XI_OK) {
    return nullptr;
  }

  auto image = std::make_unique<sensor_msgs::msg::Image>();

  image->height = xi_image.height;
  image->width = xi_image.width;

  image->encoding = sensor_msgs::image_encodings::MONO8;

  const uint8_t bytes_per_pixel = xi_image.bp_size / (xi_image.height * xi_image.width);
  image->step = bytes_per_pixel * xi_image.width;

  image->data.resize(xi_image.bp_size);
  std::memcpy(&(image->data[0]), xi_image.bp, xi_image.bp_size);

  return image;
}

}
