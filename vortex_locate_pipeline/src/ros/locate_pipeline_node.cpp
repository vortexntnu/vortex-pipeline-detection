#include "vortex_locate_pipeline/locator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<vortex_locate_pipeline::PipelineLocatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
