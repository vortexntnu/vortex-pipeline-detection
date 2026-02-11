#include "vortex_pipeline_3d/localizer_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vortex_pipeline_3d::LocalizerNode>());
  rclcpp::shutdown();
  return 0;
}
