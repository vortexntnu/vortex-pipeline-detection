#include "vortex_pipeline_image_endpoints/detector_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/msg/point2_d.hpp>
#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("pipeline_image_endpoints", options) {
    auto qos = rclcpp::QoS(1).best_effort();

    // Parameters
    kernel_size_ = this->declare_parameter<int>("morph_kernel_size");
    auto input_topic = this->declare_parameter<std::string>("input_topic");
    auto output_topic = this->declare_parameter<std::string>("output_topic");
    auto debug_topic = this->declare_parameter<std::string>("debug_topic");
    debug_ = this->declare_parameter<bool>("debug");

    // Detection method parameter
    auto method_str = this->declare_parameter<std::string>("detection_method");
    if (method_str == "furthest_points") {
        detection_method_ = DetectionMethod::FURTHEST_POINTS;
    } else if (method_str == "lowest_pixel") {
        detection_method_ = DetectionMethod::LOWEST_PIXEL;
    } else {
        RCLCPP_WARN(get_logger(), "Unknown detection_method '%s', using 'furthest_points'",
                    method_str.c_str());
        detection_method_ = DetectionMethod::FURTHEST_POINTS;
    }

    // Subscriptions
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic, qos, std::bind(&DetectorNode::mask_callback, this, std::placeholders::_1));

    // Publishers
    endpoints_pub_ = this->create_publisher<vortex_msgs::msg::Point2DArray>(output_topic, qos);

    if (debug_) {
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic, qos);
    }

    RCLCPP_INFO(this->get_logger(), "Pipeline image endpoints node started");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detection method: %s", method_str.c_str());
    RCLCPP_INFO(this->get_logger(), "  Debug visualization: %s", debug_ ? "enabled" : "disabled");
    if (debug_) {
        RCLCPP_INFO(this->get_logger(), "  Debug topic: %s", debug_topic.c_str());
    }
}

void DetectorNode::mask_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received mask image: %dx%d", msg->width, msg->height);

    try {
        auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");

        cv::Mat debug_vis;
        auto endpoints = PipelineDetector::find_pipeline_endpoints(
            cv_ptr->image, detection_method_, kernel_size_, debug_ ? &debug_vis : nullptr);

        if (!endpoints) {
            RCLCPP_WARN(this->get_logger(), "No endpoints detected");
            return;
        }

        if (endpoints->endpoint2.has_value()) {
            RCLCPP_DEBUG(this->get_logger(), "Found endpoints: (%d,%d) and (%d,%d)",
                         endpoints->endpoint1.x, endpoints->endpoint1.y, endpoints->endpoint2->x,
                         endpoints->endpoint2->y);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Found endpoint: (%d,%d)", endpoints->endpoint1.x,
                         endpoints->endpoint1.y);
        }

        vortex_msgs::msg::Point2DArray endpoints_msg;
        endpoints_msg.header.stamp = msg->header.stamp;
        endpoints_msg.header.frame_id = msg->header.frame_id;

        vortex_msgs::msg::Point2D pt1;
        pt1.x = endpoints->endpoint1.x;
        pt1.y = endpoints->endpoint1.y;
        endpoints_msg.points.push_back(pt1);

        if (endpoints->endpoint2.has_value()) {
            vortex_msgs::msg::Point2D pt2;
            pt2.x = endpoints->endpoint2->x;
            pt2.y = endpoints->endpoint2->y;
            endpoints_msg.points.push_back(pt2);
        }

        endpoints_pub_->publish(endpoints_msg);

        if (debug_ && !debug_vis.empty()) {
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_vis).toImageMsg();
            debug_pub_->publish(*debug_msg);
        }
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

}  // namespace vortex_pipeline_image_endpoints

RCLCPP_COMPONENTS_REGISTER_NODE(vortex_pipeline_image_endpoints::DetectorNode)
