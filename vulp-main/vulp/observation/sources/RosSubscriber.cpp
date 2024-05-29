#include "vulp/observation/sources/RosSubscriber.h"

namespace vulp::observation::sources {


CpuTemperatureROS2::CpuTemperatureROS2(const std::string& topic_name)
    : Node("cpu_temperature_node"), topic_name_(topic_name), current_temperature_(0.0), has_warned_(false), is_disabled_(false) {
  subscription_ = create_subscription<sensor_msgs::msg::Temperature>(
      topic_name_,
      10, 
      std::bind(&CpuTemperatureROS2::temperature_callback, this, std::placeholders::_1));
}


CpuTemperatureROS2::~CpuTemperatureROS2() {
}


void CpuTemperatureROS2::temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
  current_temperature_ = msg->temperature;
  check_temperature_warning(current_temperature_);
}


void CpuTemperatureROS2::write(Dictionary& observation) {
  if (is_disabled_) {
    return;
  }

  auto& output = observation(prefix());
  output = current_temperature_;
}


void CpuTemperatureROS2::check_temperature_warning(const double temperature) {
  constexpr double kConcerningTemperature = 75.0;
  if (temperature > kConcerningTemperature) {
    if (!has_warned_) {
      spdlog::warn("CPU temperature > {} °C, thermal throttling may occur",
                   kConcerningTemperature);
      has_warned_ = true;
    }
  }

  constexpr double kHysteresisFactor = 0.95;
  if (has_warned_ && temperature < kHysteresisFactor * kConcerningTemperature) {
    has_warned_ = false;
  }
}

}  // namespace vulp::observation::sources
