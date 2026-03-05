#include <rclcpp/rclcpp.hpp>
#include "vortex_pipeline_image_endpoints/detector_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vortex_pipeline_image_endpoints::DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
