#include "vortex_pipeline_endpoints/detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vortex_pipeline_endpoints::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
