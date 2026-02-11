#pragma once

#include "vortex_pipeline_3d/geometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace vortex_pipeline_3d {

/**
 * @brief Visualizer for 2D image overlay showing endpoint detection and reprojection
 *
 * Verifies backprojection math by:
 * 1. Drawing original 2D endpoint detections (green circles)
 * 2. Reprojecting 3D points back to 2D (red circles - should overlap with green)
 * 3. Highlighting selected endpoint (yellow circle)
 * 4. Adding text annotations with 3D coordinates and DVL altitude
 */
class ImageOverlayVisualizer {
public:
  /**
   * @brief Constructor
   * @param node ROS node pointer for logging
   * @param input_image_topic Topic to subscribe for camera images
   * @param output_image_topic Topic to publish annotated images
   */
  ImageOverlayVisualizer(
    rclcpp::Node* node,
    const std::string& input_image_topic,
    const std::string& output_image_topic);

  /**
   * @brief Visualize endpoints and 3D backprojection on image
   * @param image Camera image to annotate
   * @param endpoints_2d Original 2D endpoint detections
   * @param endpoints_3d 3D backprojected endpoints
   * @param selected_3d The selected 3D endpoint (closest to origin)
   * @param intrinsics Camera calibration parameters
   * @param dvl_altitude Current DVL altitude measurement
   */
  void visualize(
    const sensor_msgs::msg::Image::SharedPtr& image,
    const std::vector<cv::Point2d>& endpoints_2d,
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d,
    const CameraIntrinsics& intrinsics,
    double dvl_altitude);

private:
  /**
   * @brief Project 3D point back to image coordinates
   * @param pt3d 3D point in camera frame
   * @param intrinsics Camera calibration parameters
   * @return 2D pixel coordinates
   */
  cv::Point2d projectToImage(const cv::Point3d& pt3d, const CameraIntrinsics& intrinsics);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Node* node_;
};

} // namespace vortex_pipeline_3d
