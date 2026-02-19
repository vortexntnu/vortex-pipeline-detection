#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vortex_msgs/msg/point2_d_array.hpp>
#include "vortex_pipeline_endpoints/detector.hpp"

namespace vortex_pipeline_endpoints {

class DetectorNode : public rclcpp::Node {
public:
  DetectorNode();

private:
  void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  bool debug_;
  DetectionMethod detection_method_{DetectionMethod::FURTHEST_POINTS};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Publisher<vortex_msgs::msg::Point2DArray>::SharedPtr endpoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

} // namespace vortex_pipeline_endpoints
