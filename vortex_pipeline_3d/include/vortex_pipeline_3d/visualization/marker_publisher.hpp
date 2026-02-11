#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace vortex_pipeline_3d {

/**
 * @brief Publisher for 3D visualization markers showing endpoints and reference frames
 *
 * Publishes:
 * - Endpoint spheres (blue) at both 3D endpoints
 * - Selected endpoint sphere (green, larger) at chosen endpoint
 * - Ground plane grid at DVL altitude
 * - Camera coordinate axes (RGB arrows for X/Y/Z)
 * - Camera frustum showing field of view
 * - Rays from camera origin to endpoints
 */
class MarkerPublisher {
public:
  /**
   * @brief Constructor
   * @param node ROS node pointer for logging
   * @param marker_topic Topic to publish marker arrays
   * @param frame_id Frame ID for all markers (usually camera frame)
   */
  MarkerPublisher(
    rclcpp::Node* node,
    const std::string& marker_topic,
    const std::string& frame_id);

  /**
   * @brief Publish all visualization markers for current endpoints
   * @param endpoints_3d Both 3D endpoints
   * @param selected_3d The selected endpoint (closest to origin)
   * @param dvl_altitude Current DVL altitude measurement
   */
  void publishEndpoints(
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d,
    double dvl_altitude);

private:
  /**
   * @brief Add endpoint sphere markers to array
   */
  void addEndpointMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d);

  /**
   * @brief Add ground plane grid marker at DVL altitude
   */
  void addGroundPlaneMarker(
    visualization_msgs::msg::MarkerArray& markers,
    double altitude);

  /**
   * @brief Add camera coordinate axes (RGB arrows)
   */
  void addCameraAxesMarker(
    visualization_msgs::msg::MarkerArray& markers);

  /**
   * @brief Add camera frustum showing field of view
   */
  void addCameraFrustumMarker(
    visualization_msgs::msg::MarkerArray& markers,
    double altitude);

  /**
   * @brief Add ray lines from camera to endpoints
   */
  void addRayMarkers(
    visualization_msgs::msg::MarkerArray& markers,
    const std::vector<cv::Point3d>& endpoints_3d);

  /**
   * @brief Create a basic marker with common properties
   */
  visualization_msgs::msg::Marker createBaseMarker(int id);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Node* node_;
  std::string frame_id_;
  int marker_id_counter_;
};

} // namespace vortex_pipeline_3d
