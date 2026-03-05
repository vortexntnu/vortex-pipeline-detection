#include <rclcpp/rclcpp.hpp>
#include "vortex_pipeline_position_estimator/position_estimator_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vortex_pipeline_position_estimator::PositionEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
