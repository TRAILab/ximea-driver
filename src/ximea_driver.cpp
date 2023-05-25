#include "ximea_driver/ximea_driver.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace trailbot {

constexpr int kTimeoutMs = 100;

XIMEADriver::XIMEADriver() : Node("ximea_driver") {
  this->declare_parameter<bool>("auto_exposure", true);
  this->declare_parameter<int>("exposure_time_ms", 50);
  this->declare_parameter<int>("max_exposure_time_ms", 50);
  this->declare_parameter<float>("gain_db", 0);
  this->declare_parameter<int>("target_brightness", 0);
  this->declare_parameter<float>("gamma_y", 0);
}

void XIMEADriver::Run() {
  it_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
  pub_ = it_->advertise("camera", 10);

  auto stat = xiOpenDevice(0, &xi_handle_);

  if (stat != XI_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to open camera");
  }

  stat = xiSetParamInt(xi_handle_, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
  stat = xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);

  const bool auto_exposure = this->get_parameter("auto_exposure").as_bool();

  if (auto_exposure) {
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG, XI_ON);
    stat = xiSetParamFloat(xi_handle_, XI_PRM_EXP_PRIORITY, 0.5);

    const auto max_exposure_time_us = this->get_parameter("max_exposure_time_ms").as_int() * 1000;
    stat = xiSetParamInt(xi_handle_, XI_PRM_AE_MAX_LIMIT, max_exposure_time_us);

    const auto target_brightness = this->get_parameter("target_brightness").as_int();
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG_LEVEL, target_brightness);
  } else {
    stat = xiSetParamInt(xi_handle_, XI_PRM_AEAG, XI_OFF);
    const auto exposure_time_us = this->get_parameter("exposure_time_ms").as_int() * 1000;
    stat = xiSetParamInt(xi_handle_, XI_PRM_EXPOSURE, exposure_time_us);
    stat = xiSetParamFloat(xi_handle_, XI_PRM_GAIN, this->get_parameter("gain_db").as_double());
  }

  stat = xiSetParamFloat(xi_handle_, XI_PRM_GAMMAY, this->get_parameter("gamma_y").as_double());

  stat = xiSetParamInt(xi_handle_, XI_PRM_HORIZONTAL_FLIP, XI_ON);
  stat = xiSetParamInt(xi_handle_, XI_PRM_VERTICAL_FLIP, XI_ON);

  stat = xiStartAcquisition(xi_handle_);

  RCLCPP_INFO(get_logger(), "Starting image stream");

  while(rclcpp::ok()) {
    auto img = GetImage();
    if (img != nullptr) {
      pub_.publish(std::move(img));
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

  image->header.stamp = this->now();

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
