#include "vortex_pipeline_3d/localizer_node.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace vortex_pipeline_3d;

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions &options)
    : Node("pipeline_localizer", options) {

  auto qos = rclcpp::QoS(1).best_effort();

  // Read visualization parameters
  enable_debug_image_ = this->declare_parameter<bool>("enable_debug_image", false);
  enable_3d_markers_ = this->declare_parameter<bool>("enable_3d_markers", false);

  // Subscriptions
  endpoints_sub_ = this->create_subscription<vortex_msgs::msg::Point2DArray>(
      "/pipeline/endpoints", qos,
      std::bind(&LocalizerNode::endpointsCallback, this, _1));

  caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/cam/camera_info", rclcpp::QoS(10),
      std::bind(&LocalizerNode::cameraInfoCallback, this, _1));

  dvl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/dvl/altitude", qos,
      std::bind(&LocalizerNode::dvlCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/orca/odom", qos,
      std::bind(&LocalizerNode::odomCallback, this, _1));

  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pipeline/start_point_3d", 10);

  // Create visualizers if enabled
  if (enable_debug_image_) {
    auto input_image_topic = this->declare_parameter<std::string>(
        "debug_input_image_topic", "/cam/image_color");
    auto output_image_topic = this->declare_parameter<std::string>(
        "debug_output_image_topic", "/pipeline/debug/localization_overlay");

    image_viz_ = std::make_unique<ImageOverlayVisualizer>(
        this, input_image_topic, output_image_topic);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_image_topic, rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          std::unique_lock<std::shared_mutex> lock(data_mutex_);
          last_image_ = msg;
        });

    RCLCPP_INFO(this->get_logger(), "Debug image visualization enabled");
  }

  if (enable_3d_markers_) {
    auto marker_topic = this->declare_parameter<std::string>(
        "marker_topic", "/pipeline/debug/markers");

    marker_viz_ = std::make_unique<MarkerPublisher>(
        this, marker_topic, "camera_frame");

    RCLCPP_INFO(this->get_logger(), "3D marker visualization enabled");
  }

  RCLCPP_INFO(this->get_logger(), "Pipeline 3D localizer node started");
}

// ============================================================================
// CALLBACKS
// ============================================================================

void LocalizerNode::endpointsCallback(
    const vortex_msgs::msg::Point2DArray::SharedPtr msg) {

  if (msg->points.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received less than 2 endpoints");
    return;
  }

  std::shared_lock<std::shared_mutex> lock(data_mutex_);

  // Check if we have DVL altitude and camera info
  if (dvl_altitude_ <= 0.0 || !last_caminfo_) {
    if (dvl_altitude_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "No valid DVL altitude data available");
    }
    if (!last_caminfo_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "No camera info available");
    }
    return;
  }

  // Extract camera intrinsics
  CameraIntrinsics intrinsics = extractIntrinsics(last_caminfo_);

  // Check if we have valid pitch data
  if (std::abs(vehicle_pitch_) < 0.01) {  // Less than ~0.5 degrees
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Vehicle pitch very small (%.2f deg) - may not see ground with horizontal camera",
        vehicle_pitch_ * 180.0 / M_PI);
  }

  // Backproject both endpoints to 3D
  cv::Point3d pt1_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(msg->points[0].x),
      static_cast<int>(msg->points[0].y),
      dvl_altitude_, intrinsics, vehicle_pitch_, true);

  cv::Point3d pt2_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(msg->points[1].x),
      static_cast<int>(msg->points[1].y),
      dvl_altitude_, intrinsics, vehicle_pitch_, true);

  std::vector<cv::Point3d> endpoints_3d = {pt1_3d, pt2_3d};

  // Select closest endpoint to 3D origin
  cv::Point3d selected_3d = selectClosestEndpointTo3DOrigin(
      *msg, dvl_altitude_, intrinsics);

  // Publish 3D pose
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = intrinsics.frame_id;
  pose_msg.pose.position.x = selected_3d.x;
  pose_msg.pose.position.y = selected_3d.y;
  pose_msg.pose.position.z = selected_3d.z;
  pose_msg.pose.orientation.w = 1.0;  // Identity (no orientation, just a point)

  pose_pub_->publish(pose_msg);

  RCLCPP_DEBUG(this->get_logger(),
      "Published 3D pose: (%.3f, %.3f, %.3f) in frame %s",
      selected_3d.x, selected_3d.y, selected_3d.z,
      intrinsics.frame_id.c_str());

  // Visualization (if enabled)
  if (enable_debug_image_ && image_viz_ && last_image_) {
    std::vector<cv::Point2d> endpoints_2d = {
      cv::Point2d(msg->points[0].x, msg->points[0].y),
      cv::Point2d(msg->points[1].x, msg->points[1].y)
    };

    image_viz_->visualize(
        last_image_, endpoints_2d, endpoints_3d,
        selected_3d, intrinsics, dvl_altitude_);
  }

  if (enable_3d_markers_ && marker_viz_) {
    marker_viz_->publishEndpoints(endpoints_3d, selected_3d, dvl_altitude_);
  }
}

void LocalizerNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);

  // Store camera info (it's static and doesn't change)
  if (!last_caminfo_) {
    last_caminfo_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received camera info, unsubscribing (static data)");

    // Unsubscribe since camera info doesn't change
    caminfo_sub_.reset();
  }
}

void LocalizerNode::dvlCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  dvl_altitude_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Received DVL altitude: %.3f m", dvl_altitude_);
}

void LocalizerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);

  // Extract pitch from quaternion
  // Using tf2 conversion: pitch = atan2(2*(qw*qy - qz*qx), 1 - 2*(qx^2 + qy^2))
  const auto& q = msg->pose.pose.orientation;
  double pitch = std::atan2(
      2.0 * (q.w * q.y - q.z * q.x),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y)
  );

  vehicle_pitch_ = pitch;

  RCLCPP_DEBUG(this->get_logger(),
      "Vehicle pitch: %.2f deg", pitch * 180.0 / M_PI);
}

// ============================================================================
// HELPER METHODS
// ============================================================================

CameraIntrinsics LocalizerNode::extractIntrinsics(
    const sensor_msgs::msg::CameraInfo::SharedPtr& msg) {
  CameraIntrinsics intrinsics;

  // Extract K matrix: [fx  0 cx]
  //                   [ 0 fy cy]
  //                   [ 0  0  1]
  intrinsics.fx = msg->k[0];
  intrinsics.fy = msg->k[4];
  intrinsics.cx = msg->k[2];
  intrinsics.cy = msg->k[5];
  intrinsics.frame_id = msg->header.frame_id;

  // Extract distortion coefficients
  if (!msg->d.empty()) {
    intrinsics.D = msg->d;
    intrinsics.has_distortion = true;

    // Check if distortion is actually non-zero
    bool all_zero = true;
    for (double coeff : intrinsics.D) {
      if (std::abs(coeff) > 1e-6) {
        all_zero = false;
        break;
      }
    }
    if (all_zero) {
      intrinsics.has_distortion = false;
    }
  } else {
    intrinsics.has_distortion = false;
  }

  RCLCPP_DEBUG(this->get_logger(),
      "Extracted intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f distortion=%s",
      intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
      intrinsics.has_distortion ? "yes" : "no");

  return intrinsics;
}

cv::Point3d LocalizerNode::selectClosestEndpointTo3DOrigin(
    const vortex_msgs::msg::Point2DArray& endpoints,
    double dvl_altitude,
    const CameraIntrinsics& intrinsics) {

  // Backproject both endpoints to 3D WITH PITCH
  cv::Point3d pt1_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(endpoints.points[0].x),
      static_cast<int>(endpoints.points[0].y),
      dvl_altitude, intrinsics, vehicle_pitch_, true);

  cv::Point3d pt2_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(endpoints.points[1].x),
      static_cast<int>(endpoints.points[1].y),
      dvl_altitude, intrinsics, vehicle_pitch_, true);

  // Compute distance to origin (0,0,0)
  double dist1 = std::sqrt(pt1_3d.x * pt1_3d.x + pt1_3d.y * pt1_3d.y + pt1_3d.z * pt1_3d.z);
  double dist2 = std::sqrt(pt2_3d.x * pt2_3d.x + pt2_3d.y * pt2_3d.y + pt2_3d.z * pt2_3d.z);

  RCLCPP_INFO(this->get_logger(),
      "EP1 at pixel (%.0f,%.0f) -> 3D (%.2f,%.2f,%.2f) dist=%.3fm",
      endpoints.points[0].x, endpoints.points[0].y, pt1_3d.x, pt1_3d.y, pt1_3d.z, dist1);

  RCLCPP_INFO(this->get_logger(),
      "EP2 at pixel (%.0f,%.0f) -> 3D (%.2f,%.2f,%.2f) dist=%.3fm",
      endpoints.points[1].x, endpoints.points[1].y, pt2_3d.x, pt2_3d.y, pt2_3d.z, dist2);

  // Select endpoint with minimum distance to origin
  if (dist1 < dist2) {
    RCLCPP_INFO(this->get_logger(), "Selected EP1 (closest to origin)");
    return pt1_3d;
  } else {
    RCLCPP_INFO(this->get_logger(), "Selected EP2 (closest to origin)");
    return pt2_3d;
  }
}
