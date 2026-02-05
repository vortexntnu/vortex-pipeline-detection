// ROS wrapper implementation for the pipeline locator node.
// This file converts ROS messages to the core API and publishes results.

#include "vortex_locate_pipeline/locator.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace vortex_locate_pipeline;

PipelineLocatorNode::PipelineLocatorNode(const rclcpp::NodeOptions &options)
    : Node("pipeline_locator", options) {

  // Load topic parameters
  mask_topic_ = this->declare_parameter<std::string>("mask_topic");
  depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/unused");  // Legacy
  camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic");
  dvl_altitude_topic_ = this->declare_parameter<std::string>("dvl_altitude_topic");
  publish_topic_ = this->declare_parameter<std::string>("publish_topic");
  debug_topic_ = this->declare_parameter<std::string>("debug_topic");

  // Load algorithm config parameters
  debug_ = this->declare_parameter<bool>("debug", false);
  use_skeleton_method_ = this->declare_parameter<bool>("use_skeleton_method", true);
  enable_triangulation_ = this->declare_parameter<bool>("enable_triangulation", false);
  max_observations_ = this->declare_parameter<int>("max_observations", 50);
  min_observations_for_triangulation_ = this->declare_parameter<int>("min_observations_for_triangulation", 5);

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

  dvl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      dvl_altitude_topic_, qos,
      std::bind(&PipelineLocatorNode::dvlCallback, this, _1));

  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      publish_topic_, 10);

  if (debug_) {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic_, qos);
  }

  RCLCPP_INFO(this->get_logger(), "Pipeline Locator node started");
  RCLCPP_INFO(this->get_logger(), "  Skeleton method: %s", use_skeleton_method_ ? "enabled" : "disabled (using fallback)");
  RCLCPP_INFO(this->get_logger(), "  Triangulation: %s", enable_triangulation_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "  Debug visualization: %s", debug_ ? "enabled" : "disabled");
}

// ============================================================================
// CALLBACKS
// ============================================================================

void PipelineLocatorNode::maskCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received mask image %dx%d",
               msg->width, msg->height);

  // Convert ROS image to OpenCV
  cv::Mat mask = convertToMono8(msg);
  if (mask.empty()) {
    return;
  }

  // Find pipeline endpoints using skeleton or fallback method
  cv::Mat debug_vis;
  auto endpoints = LocatorCore::findPipelineEndpoints(
      mask, use_skeleton_method_, debug_ ? &debug_vis : nullptr);

  if (!endpoints) {
    RCLCPP_DEBUG(this->get_logger(), "No pipeline endpoints found in this frame");
    return;
  }

  // Check if we have both endpoints
  if (!endpoints->found_both) {
    RCLCPP_WARN(this->get_logger(), "Only one endpoint found, using it as start point");
  }

  // ========================================================================
  // 3D BACKPROJECTION: Convert endpoints to 3D and select closest to origin
  // ========================================================================

  std::optional<cv::Point> selected_pixel;
  std::optional<cv::Point3d> selected_3d;

  {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);

    // Check if we have DVL altitude and camera info
    if (dvl_altitude_ > 0.0 && last_caminfo_) {
      // Extract camera intrinsics
      CameraIntrinsics intrinsics = extractIntrinsics(last_caminfo_);

      // Select closest endpoint to 3D origin (returns both pixel AND 3D point)
      auto selected_endpoint = selectClosestEndpointTo3DOrigin(*endpoints, dvl_altitude_, intrinsics);

      if (selected_endpoint) {
        // Extract pixel and already-computed 3D point (no redundant backproject!)
        selected_pixel = selected_endpoint->pixel;
        selected_3d = selected_endpoint->point_3d;

        RCLCPP_INFO(this->get_logger(),
            "Selected endpoint at pixel (%d, %d) -> 3D (%.3f, %.3f, %.3f) in frame %s",
            selected_pixel->x, selected_pixel->y,
            selected_3d->x, selected_3d->y, selected_3d->z,
            intrinsics.frame_id.c_str());

        // Publish 3D pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = intrinsics.frame_id;
        pose_msg.pose.position.x = selected_3d->x;
        pose_msg.pose.position.y = selected_3d->y;
        pose_msg.pose.position.z = selected_3d->z;
        pose_msg.pose.orientation.w = 1.0;  // Identity (no orientation, just a point)

        pose_pub_->publish(pose_msg);

        // Add to triangulation buffer if enabled
        if (enable_triangulation_) {
          auto mask_time = rclcpp::Time(msg->header.stamp);
          addObservation(*selected_pixel, *selected_3d, mask_time,
                        intrinsics.frame_id, dvl_altitude_);
        }
      }
    } else {
      if (dvl_altitude_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No valid DVL altitude data available");
      }
      if (!last_caminfo_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No camera info available");
      }

      // Fallback: just use first endpoint for 2D visualization
      selected_pixel = endpoints->endpoint1;
      RCLCPP_DEBUG(this->get_logger(), "Fallback to 2D endpoint at (%d, %d)",
                  selected_pixel->x, selected_pixel->y);
    }
  }

  // ========================================================================
  // DEBUG VISUALIZATION
  // ========================================================================

  if (debug_ && !debug_vis.empty()) {
    // Optionally overlay selected endpoint with special marker
    if (selected_pixel) {
      cv::circle(debug_vis, *selected_pixel, 12, cv::Scalar(0, 0, 255), 3);  // Red circle
      cv::putText(debug_vis, "SELECTED",
                  cv::Point(selected_pixel->x + 15, selected_pixel->y - 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

      // Add 3D coordinates text if available
      if (selected_3d) {
        char text[100];
        snprintf(text, sizeof(text), "3D: (%.2f, %.2f, %.2f)",
                selected_3d->x, selected_3d->y, selected_3d->z);
        cv::putText(debug_vis, text,
                    cv::Point(10, debug_vis.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
      }
    }

    publishDebugImage(debug_vis, msg->header);
  }
}

void PipelineLocatorNode::depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Legacy callback - depth not used with DVL approach
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  last_depth_ = msg;
}

void PipelineLocatorNode::cameraInfoCallback(
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

void PipelineLocatorNode::dvlCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  dvl_altitude_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Received DVL altitude: %.3f m", dvl_altitude_);
}

// ============================================================================
// HELPER METHODS
// ============================================================================

cv::Mat PipelineLocatorNode::convertToMono8(
    const sensor_msgs::msg::Image::SharedPtr& msg) {
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    return cv_ptr->image.clone();
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_WARN(this->get_logger(),
                "cv_bridge failed to convert mask image: %s", e.what());
    return cv::Mat();
  }
}

CameraIntrinsics PipelineLocatorNode::extractIntrinsics(
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

void PipelineLocatorNode::publishDebugImage(
    const cv::Mat& img, const std_msgs::msg::Header& header) {
  if (!debug_pub_) return;

  try {
    auto out_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    debug_pub_->publish(*out_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published debug visualization");
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to publish debug image: %s", e.what());
  }
}

std::optional<SelectedEndpoint> PipelineLocatorNode::selectClosestEndpointTo3DOrigin(
    const PipelineEndpoints& endpoints,
    double dvl_altitude,
    const CameraIntrinsics& intrinsics) {

  // If only one endpoint, return it with its 3D coordinates
  if (!endpoints.found_both) {
    cv::Point3d pt_3d = LocatorCore::backprojectGroundPlane(
        endpoints.endpoint1.x, endpoints.endpoint1.y, dvl_altitude, intrinsics, true);
    return SelectedEndpoint{endpoints.endpoint1, pt_3d};
  }

  // Backproject both endpoints to 3D
  cv::Point3d pt1_3d = LocatorCore::backprojectGroundPlane(
      endpoints.endpoint1.x, endpoints.endpoint1.y, dvl_altitude, intrinsics, true);

  cv::Point3d pt2_3d = LocatorCore::backprojectGroundPlane(
      endpoints.endpoint2.x, endpoints.endpoint2.y, dvl_altitude, intrinsics, true);

  // Compute distance to origin (0,0,0)
  double dist1 = std::sqrt(pt1_3d.x * pt1_3d.x + pt1_3d.y * pt1_3d.y + pt1_3d.z * pt1_3d.z);
  double dist2 = std::sqrt(pt2_3d.x * pt2_3d.x + pt2_3d.y * pt2_3d.y + pt2_3d.z * pt2_3d.z);

  RCLCPP_INFO(this->get_logger(),
      "EP1 at pixel (%d,%d) -> 3D (%.2f,%.2f,%.2f) dist=%.3fm",
      endpoints.endpoint1.x, endpoints.endpoint1.y, pt1_3d.x, pt1_3d.y, pt1_3d.z, dist1);

  RCLCPP_INFO(this->get_logger(),
      "EP2 at pixel (%d,%d) -> 3D (%.2f,%.2f,%.2f) dist=%.3fm",
      endpoints.endpoint2.x, endpoints.endpoint2.y, pt2_3d.x, pt2_3d.y, pt2_3d.z, dist2);

  // Select endpoint with minimum distance to origin and return BOTH pixel and 3D point
  if (dist1 < dist2) {
    RCLCPP_INFO(this->get_logger(), "Selected EP1 (closest to origin)");
    return SelectedEndpoint{endpoints.endpoint1, pt1_3d};
  } else {
    RCLCPP_INFO(this->get_logger(), "Selected EP2 (closest to origin)");
    return SelectedEndpoint{endpoints.endpoint2, pt2_3d};
  }
}

void PipelineLocatorNode::addObservation(
    const cv::Point& pixel, const cv::Point3d& point_3d,
    const rclcpp::Time& stamp, const std::string& frame, double altitude) {

  if (!enable_triangulation_) return;

  std::lock_guard<std::mutex> lock(observation_mutex_);

  Observation obs;
  obs.pixel = cv::Point2d(pixel.x, pixel.y);
  obs.point_3d = point_3d;
  obs.timestamp = stamp;
  obs.camera_frame = frame;
  obs.dvl_altitude = altitude;

  observation_buffer_.push_back(obs);

  // Maintain buffer size limit
  if (observation_buffer_.size() > max_observations_) {
    observation_buffer_.erase(observation_buffer_.begin());
  }

  RCLCPP_DEBUG(this->get_logger(),
      "Added observation: %zu in buffer", observation_buffer_.size());

  // Attempt triangulation if we have enough observations
  if (observation_buffer_.size() >= min_observations_for_triangulation_) {
    attemptTriangulation();
  }
}

void PipelineLocatorNode::attemptTriangulation() {
  // TODO: Implement multi-view triangulation
  //
  // This is a placeholder for future triangulation implementation.
  // Triangulation improves 3D position accuracy by combining multiple
  // observations from different viewpoints.
  //
  // REQUIREMENTS:
  // 1. Camera poses for each observation (from odometry/TF)
  //    - Need to subscribe to /odometry or query tf2
  //    - Transform observations to world frame
  //
  // 2. RANSAC outlier rejection:
  //    - Randomly sample min_samples observations
  //    - Compute candidate 3D position (mean/median)
  //    - Count inliers (observations within threshold, e.g. 0.1m)
  //    - Repeat many times, keep best consensus
  //    - Refine using all inliers
  //
  // 3. OpenCV triangulation (if camera poses available):
  //    - Build projection matrices P = K * [R | t] for each observation
  //    - Call cv::triangulatePoints(P1, P2, points1, points2, points4D)
  //    - Normalize homogeneous coordinates
  //
  // 4. Kalman filter for temporal smoothing (optional):
  //    - Predict: Use motion model (stationary for static pipeline)
  //    - Update: Correct with triangulated position
  //    - Output: Smoothed position + covariance matrix
  //
  // 5. Publish result:
  //    - geometry_msgs::msg::PoseWithCovarianceStamped
  //    - Covariance from RANSAC inliers or Kalman filter
  //    - NOT from YOLO confidence - geometric uncertainty only
  //
  // REFERENCES:
  // - OpenCV triangulation: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
  // - RANSAC: https://en.wikipedia.org/wiki/Random_sample_consensus
  // - Kalman filter: https://en.wikipedia.org/wiki/Kalman_filter

  std::lock_guard<std::mutex> lock(observation_mutex_);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Triangulation not yet implemented. Buffer has %zu observations (min: %zu)",
      observation_buffer_.size(), min_observations_for_triangulation_);

  // Placeholder: simple average of 3D positions (no RANSAC, no proper triangulation)
  if (observation_buffer_.empty()) return;

  cv::Point3d avg(0, 0, 0);
  for (const auto& obs : observation_buffer_) {
    avg.x += obs.point_3d.x;
    avg.y += obs.point_3d.y;
    avg.z += obs.point_3d.z;
  }
  avg.x /= observation_buffer_.size();
  avg.y /= observation_buffer_.size();
  avg.z /= observation_buffer_.size();

  RCLCPP_DEBUG(this->get_logger(),
      "Simple average of observations: (%.3f, %.3f, %.3f)",
      avg.x, avg.y, avg.z);
}
