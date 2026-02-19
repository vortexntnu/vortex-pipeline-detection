#include "vortex_pipeline_3d/visualization/image_overlay.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sstream>
#include <iomanip>

namespace vortex_pipeline_3d {

ImageOverlayVisualizer::ImageOverlayVisualizer(
    rclcpp::Node* node,
    const std::string& input_image_topic,
    const std::string& output_image_topic)
    : node_(node) {

  pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      output_image_topic, 10);

  RCLCPP_INFO(node_->get_logger(),
      "Image overlay visualizer initialized: %s -> %s",
      input_image_topic.c_str(), output_image_topic.c_str());
}

void ImageOverlayVisualizer::visualize(
    const sensor_msgs::msg::Image::SharedPtr& image,
    const std::vector<cv::Point2d>& endpoints_2d,
    const std::vector<cv::Point3d>& endpoints_3d,
    const cv::Point3d& selected_3d,
    const CameraIntrinsics& intrinsics,
    double dvl_altitude,
    const geometry_msgs::msg::TransformStamped& camera_to_world) {

  if (endpoints_2d.empty() || endpoints_3d.empty() ||
      endpoints_2d.size() != endpoints_3d.size()) {
    RCLCPP_WARN(node_->get_logger(),
        "ImageOverlayVisualizer: Mismatched or empty endpoints (%zu 2D, %zu 3D)",
        endpoints_2d.size(), endpoints_3d.size());
    return;
  }

  // Convert ROS image to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat& img = cv_ptr->image;

  // Colors
  const cv::Scalar GREEN(0, 255, 0);    // Original 2D detections
  const cv::Scalar RED(0, 0, 255);      // Reprojected 3D points
  const cv::Scalar YELLOW(0, 255, 255); // Selected endpoint
  const cv::Scalar WHITE(255, 255, 255);

  // Draw original 2D detections (green circles) and label them
  for (size_t i = 0; i < endpoints_2d.size(); ++i) {
    cv::circle(img, endpoints_2d[i], 8, GREEN, 2);
    std::string label = (endpoints_2d.size() == 1) ? "START" : "EP" + std::to_string(i + 1);
    cv::putText(img, label, cv::Point(endpoints_2d[i].x + 12, endpoints_2d[i].y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2);
  }

  // Reproject 3D points back to 2D and draw (red circles - should overlap)
  std::vector<cv::Point2d> reprojected_points;
  std::vector<double> reproj_errors;

  for (size_t i = 0; i < endpoints_3d.size(); ++i) {
    cv::Point2d reprojected = projectToImage(endpoints_3d[i], intrinsics, camera_to_world);
    reprojected_points.push_back(reprojected);
    cv::circle(img, reprojected, 6, RED, 2);

    // Draw line connecting original detection to reprojection
    double distance = cv::norm(endpoints_2d[i] - reprojected);
    reproj_errors.push_back(distance);
    cv::Scalar line_color = (distance < 5.0) ? GREEN : cv::Scalar(0, 165, 255); // Orange if misaligned
    cv::line(img, endpoints_2d[i], reprojected, line_color, 1);

    // Warn if reprojection error is large
    if (distance > 5.0) {
      RCLCPP_WARN(node_->get_logger(),
          "Large reprojection error: %.2f pixels for endpoint %zu", distance, i);
    }
  }

  // Highlight selected endpoint (yellow circle)
  cv::Point2d selected_2d = projectToImage(selected_3d, intrinsics, camera_to_world);
  cv::circle(img, selected_2d, 12, YELLOW, 3);

  // ============================================================================
  // Create information panel overlay (semi-transparent background)
  // ============================================================================
  const int line_spacing = 30;
  const int panel_width = 520;
  const int panel_height = line_spacing + static_cast<int>(endpoints_3d.size()) * 3 * line_spacing;
  const int panel_x = 10;
  const int panel_y = 10;

  cv::Mat panel(panel_height, panel_width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::addWeighted(img(cv::Rect(panel_x, panel_y, panel_width, panel_height)),
                  0.3, panel, 0.7, 0,
                  img(cv::Rect(panel_x, panel_y, panel_width, panel_height)));

  // Add information text
  int text_y = panel_y + 25;
  const double font_scale = 0.5;
  const int thickness = 1;

  // DVL Altitude
  std::ostringstream info;
  info << std::fixed << std::setprecision(2) << "DVL Altitude: " << dvl_altitude << " m";
  cv::putText(img, info.str(), cv::Point(panel_x + 10, text_y),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, WHITE, thickness + 1);
  text_y += line_spacing;

  // Per-endpoint data
  for (size_t i = 0; i < endpoints_3d.size(); ++i) {
    std::string ep_label = (endpoints_2d.size() == 1) ? "START" : "EP" + std::to_string(i + 1);

    info.str("");
    info << ep_label << " - 2D: (" << std::fixed << std::setprecision(1)
         << endpoints_2d[i].x << ", " << endpoints_2d[i].y << ") px";
    cv::putText(img, info.str(), cv::Point(panel_x + 10, text_y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, GREEN, thickness);
    text_y += line_spacing;

    info.str("");
    info << "       3D: (" << std::fixed << std::setprecision(3)
         << endpoints_3d[i].x << ", " << endpoints_3d[i].y << ", "
         << endpoints_3d[i].z << ") m";
    cv::putText(img, info.str(), cv::Point(panel_x + 10, text_y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, GREEN, thickness);
    text_y += line_spacing;

    info.str("");
    info << "       Reproj error: " << std::fixed << std::setprecision(2)
         << reproj_errors[i] << " px";
    cv::Scalar error_color = (reproj_errors[i] < 5.0) ? GREEN : cv::Scalar(0, 165, 255);
    cv::putText(img, info.str(), cv::Point(panel_x + 10, text_y),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, error_color, thickness);
    text_y += line_spacing;
  }

  // Add legend at bottom
  int legend_y = img.rows - 80;
  cv::putText(img, "Green circles = 2D detections", cv::Point(10, legend_y),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1);
  cv::putText(img, "Red circles = 3D reprojected", cv::Point(10, legend_y + 25),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, RED, 1);
  cv::putText(img, "Yellow circle = Selected endpoint", cv::Point(10, legend_y + 50),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1);

  // Publish annotated image
  sensor_msgs::msg::Image::SharedPtr output_msg = cv_ptr->toImageMsg();
  output_msg->header = image->header;
  pub_->publish(*output_msg);
}

cv::Point2d ImageOverlayVisualizer::projectToImage(
    const cv::Point3d& pt3d_world,
    const CameraIntrinsics& intrinsics,
    const geometry_msgs::msg::TransformStamped& camera_to_world) {

  // Transform from WORLD frame to CAMERA frame (inverse transform)
  tf2::Quaternion quat;
  tf2::fromMsg(camera_to_world.transform.rotation, quat);
  tf2::Matrix3x3 rotation_matrix(quat);
  tf2::Vector3 translation(
      camera_to_world.transform.translation.x,
      camera_to_world.transform.translation.y,
      camera_to_world.transform.translation.z
  );

  // Point in world frame
  tf2::Vector3 pt_world(pt3d_world.x, pt3d_world.y, pt3d_world.z);

  // Apply inverse transform: p_camera = R^T * (p_world - t)
  tf2::Vector3 pt_relative = pt_world - translation;
  tf2::Vector3 pt_cam = rotation_matrix.transpose() * pt_relative;

  // Check if point is in front of camera
  if (pt_cam.getZ() <= 0) {
    RCLCPP_WARN(node_->get_logger(),
        "Point behind camera (Z <= 0): %.3f", pt_cam.getZ());
    return cv::Point2d(-1, -1);
  }

  // Project to image using pinhole model
  // Camera frame: X=right, Y=down, Z=forward
  double u = intrinsics.fx * (pt_cam.getX() / pt_cam.getZ()) + intrinsics.cx;
  double v = intrinsics.fy * (pt_cam.getY() / pt_cam.getZ()) + intrinsics.cy;

  return cv::Point2d(u, v);
}

} // namespace vortex_pipeline_3d
