#pragma once

#include "vortex_locate_pipeline/locator_core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vortex_locate_pipeline {

class PipelineLocatorNode : public rclcpp::Node {
public:
  explicit PipelineLocatorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PipelineLocatorNode() = default;

private:
  void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // params/topics
  std::string mask_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string publish_topic_;
  // debug visualization
  bool debug_{false};
  std::string debug_topic_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // shared data
  std::shared_ptr<sensor_msgs::msg::Image> last_depth_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> last_caminfo_;
  std::shared_mutex data_mutex_;
};

} // namespace vortex_locate_pipeline
