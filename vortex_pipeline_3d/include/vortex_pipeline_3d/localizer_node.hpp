#pragma once

#include "vortex_pipeline_3d/geometry.hpp"
#include "vortex_pipeline_3d/visualization/image_overlay.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vortex_msgs/msg/detail/landmark_array__struct.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_type_class.hpp>
#include <vortex_msgs/msg/point2_d_array.hpp>
#include <opencv2/core.hpp>
#include <shared_mutex>
#include <memory>
#include <vector>

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
      const std::vector<cv::Point3d>& endpoints_3d);

  // ROS interfaces
  rclcpp::Subscription<vortex_msgs::msg::Point2DArray>::SharedPtr endpoints_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dvl_sub_;
  rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;

  // Visualization (optional)
  std::unique_ptr<ImageOverlayVisualizer> image_viz_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Shared data (protected by mutex)
  std::shared_ptr<sensor_msgs::msg::CameraInfo> last_caminfo_;
  std::shared_ptr<sensor_msgs::msg::Image> last_image_;
  double dvl_altitude_{0.0};
  std::shared_mutex data_mutex_;

  // tf2 for frame transformations
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Parameters
  bool enable_debug_image_{false};
};

} // namespace vortex_pipeline_3d
