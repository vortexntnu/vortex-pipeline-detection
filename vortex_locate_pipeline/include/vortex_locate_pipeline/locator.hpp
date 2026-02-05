#pragma once

#include "vortex_locate_pipeline/locator_core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vortex_locate_pipeline {

// Selected endpoint with both 2D and 3D coordinates
struct SelectedEndpoint {
    cv::Point pixel;      // 2D pixel coordinate
    cv::Point3d point_3d; // 3D backprojected point (already computed)
};

// Observation for multi-frame triangulation
struct Observation {
    cv::Point2d pixel;           // 2D detection in image
    cv::Point3d point_3d;        // 3D backprojected position
    rclcpp::Time timestamp;      // When detected
    std::string camera_frame;    // Camera frame_id
    double dvl_altitude;         // DVL altitude at time of detection
    // Future extensions:
    // Eigen::Isometry3d camera_pose;  // For multi-view triangulation
    // double confidence;               // Detection quality metric
};

class PipelineLocatorNode : public rclcpp::Node {
public:
  explicit PipelineLocatorNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PipelineLocatorNode() = default;

private:
  // Callbacks
  void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void dvlCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // Helper methods
  cv::Mat convertToMono8(const sensor_msgs::msg::Image::SharedPtr& msg);
  CameraIntrinsics extractIntrinsics(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);
  void publishDebugImage(const cv::Mat& img, const std_msgs::msg::Header& header);
  std::optional<SelectedEndpoint> selectClosestEndpointTo3DOrigin(
      const PipelineEndpoints& endpoints,
      double dvl_altitude,
      const CameraIntrinsics& intrinsics);
  void addObservation(const cv::Point& pixel, const cv::Point3d& point_3d,
                     const rclcpp::Time& stamp, const std::string& frame,
                     double altitude);
  void attemptTriangulation();

  // Parameters: Topics
  std::string mask_topic_;
  std::string depth_topic_;  // Legacy - not used with DVL approach
  std::string camera_info_topic_;
  std::string dvl_altitude_topic_;
  std::string publish_topic_;
  std::string debug_topic_;

  // Parameters: Algorithm config
  bool debug_{false};
  bool use_skeleton_method_{true};
  bool enable_triangulation_{false};
  size_t max_observations_{50};
  size_t min_observations_for_triangulation_{5};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;  // Legacy
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dvl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Shared data (protected by mutex)
  std::shared_ptr<sensor_msgs::msg::Image> last_depth_;  // Legacy
  std::shared_ptr<sensor_msgs::msg::CameraInfo> last_caminfo_;
  double dvl_altitude_{0.0};
  std::shared_mutex data_mutex_;

  // Triangulation infrastructure
  std::vector<Observation> observation_buffer_;
  std::mutex observation_mutex_;
};

} // namespace vortex_locate_pipeline
