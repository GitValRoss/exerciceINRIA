#pragma once

#include <string>
#include "vulp/observation/Source.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace vulp::observation::sources {

class CpuTemperatureROS2 : public Source, public rclcpp::Node {
 public:
  CpuTemperatureROS2(const std::string& topic_name = "topic_temperature");
  ~CpuTemperatureROS2() override;;

    //! Prefix of output in the observation dictionary.
  inline std::string prefix() const noexcept final { return "cpu_temperature_ROS2"; }

  void write(Dictionary& observation) final;

  bool is_disabled() const { return is_disabled_; }


 private:
  void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
  void check_temperature_warning(const double temperature);


  std::string topic_name_;
  double current_temperature_;
  bool has_warned_;
  bool is_disabled_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr subscription_;
};

}  // namespace vulp::observation::sources
