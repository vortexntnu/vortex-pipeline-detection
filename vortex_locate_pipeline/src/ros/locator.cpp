// ROS wrapper implementation for the pipeline locator node.
// This file converts ROS messages to the core API and publishes results.

#include "vortex_locate_pipeline/locator.hpp"

using std::placeholders::_1;
using namespace vortex_locate_pipeline;

PipelineLocatorNode::PipelineLocatorNode(const rclcpp::NodeOptions &options)
    : Node("pipeline_locator", options) {

  // Load parameters
  mask_topic_ = this->declare_parameter<std::string>("mask_topic");
  depth_topic_ = this->declare_parameter<std::string>("depth_topic");
  camera_info_topic_ =
      this->declare_parameter<std::string>("camera_info_topic");
  publish_topic_ = this->declare_parameter<std::string>("publish_topic");

  // Debug visualization params
  debug_ = this->declare_parameter<bool>("debug", false);
  debug_topic_ = this->declare_parameter<std::string>("debug_topic");

  auto qos = rclcpp::QoS(1).best_effort();

  // Subscriptions
  mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      mask_topic_, qos,
      std::bind(&PipelineLocatorNode::maskCallback, this, _1));
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      std::bind(&PipelineLocatorNode::depthCallback, this, _1));
  caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(10),
      std::bind(&PipelineLocatorNode::cameraInfoCallback, this, _1));

  // Publisher for located start pose
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      publish_topic_, 10);

  // Optional debug publisher (visualize mask with start pixel)
  if (debug_) {
    debug_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>(debug_topic_, qos);
  }

  RCLCPP_INFO(this->get_logger(), "Pipeline Locator node started");
}

void PipelineLocatorNode::maskCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Convert ROS image to single-channel binary mask (mono8 -> 0/255)
  cv::Mat mask;
  try {
    // Force mono8 encoding (caller sends mono8), and ensure binary 0/255
    auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    mask = cv_ptr->image;
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_WARN(this->get_logger(),
                "cv_bridge: failed to convert mask image: %s", e.what());
    return;
  }

  // Use core API to find pixel
  auto start_px = LocatorCore::findStartPixel(mask);
  if (!start_px)
    return; // std::nullopt -> nothing to do
  // Try to backproject to 3D if we have recent depth and camera info

  // Optional debug visualization: mark start pixel and publish
  if (debug_) {
    cv::Mat vis;
    cv::cvtColor(mask, vis, cv::COLOR_GRAY2BGR);
    cv::circle(vis, start_px.value(), 4, cv::Scalar(0, 0, 255), -1);
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
    debug_pub_->publish(*out_msg);
  }
}

void PipelineLocatorNode::depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  last_depth_ = msg;
}

void PipelineLocatorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  last_caminfo_ = msg;
}
