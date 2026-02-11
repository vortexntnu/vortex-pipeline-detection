#pragma once

#include "vortex_pipeline_3d/geometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vortex_msgs/msg/point2_d_array.hpp>
#include <shared_mutex>

namespace vortex_pipeline_3d {

class LocalizerNode : public rclcpp::Node {
public:
  explicit LocalizerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Callbacks
  void endpointsCallback(const vortex_msgs::msg::Point2DArray::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void dvlCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // Helper methods
  CameraIntrinsics extractIntrinsics(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);
  cv::Point3d selectClosestEndpointTo3DOrigin(
      const vortex_msgs::msg::Point2DArray& endpoints,
      double dvl_altitude,
      const CameraIntrinsics& intrinsics);

  // ROS interfaces
  rclcpp::Subscription<vortex_msgs::msg::Point2DArray>::SharedPtr endpoints_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dvl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Shared data (protected by mutex)
  std::shared_ptr<sensor_msgs::msg::CameraInfo> last_caminfo_;
  double dvl_altitude_{0.0};
  std::shared_mutex data_mutex_;
};

} // namespace vortex_pipeline_3d
