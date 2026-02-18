#include "vortex_pipeline_3d/localizer_node.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace vortex_pipeline_3d;

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions &options)
    : Node("pipeline_localizer", options) {

  auto qos = rclcpp::QoS(1).best_effort();

  // Initialize tf2 buffer and listener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  RCLCPP_INFO(this->get_logger(), "tf2 buffer and listener initialized");

  // Declare parameters
  auto endpoints_topic = this->declare_parameter<std::string>("endpoints_topic");
  auto camera_info_topic = this->declare_parameter<std::string>("camera_info_topic");
  auto dvl_altitude_topic = this->declare_parameter<std::string>("dvl_altitude_topic");
  auto publish_topic = this->declare_parameter<std::string>("publish_topic");

  // Read visualization parameters
  enable_debug_image_ = this->declare_parameter<bool>("enable_debug_image", false);

  // Subscriptions using parameters
  endpoints_sub_ = this->create_subscription<vortex_msgs::msg::Point2DArray>(
      endpoints_topic, qos,
      std::bind(&LocalizerNode::endpointsCallback, this, _1));

  caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos,
      std::bind(&LocalizerNode::cameraInfoCallback, this, _1));

  dvl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      dvl_altitude_topic, qos,
      std::bind(&LocalizerNode::dvlCallback, this, _1));

  // Publisher using parameter
  landmark_pub_ = this->create_publisher<vortex_msgs::msg::Landmark>(
      publish_topic, 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to:");
  RCLCPP_INFO(this->get_logger(), "  - Endpoints: %s", endpoints_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  - Camera info: %s", camera_info_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  - DVL altitude: %s", dvl_altitude_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s", publish_topic.c_str());

  // Create visualizers if enabled
  if (enable_debug_image_) {
    // Topic names required if debug is enabled (no defaults)
    auto input_image_topic = this->declare_parameter<std::string>("debug_input_image_topic");
    auto output_image_topic = this->declare_parameter<std::string>("debug_output_image_topic");

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

  // Query camera-to-world transform using tf2
  geometry_msgs::msg::TransformStamped camera_to_world;
  try {
    camera_to_world = tf2_buffer_->lookupTransform(
        "odom",                    // target frame (world)
        intrinsics.frame_id,       // source frame (camera_frame from camera_info)
        msg->header.stamp,         // timestamp of observation
        std::chrono::milliseconds(100)  // wait up to 100ms
    );
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform %s to odom: %s", intrinsics.frame_id.c_str(), ex.what());
    return;
  }

  // Backproject both endpoints to 3D using full transform
  cv::Point3d pt1_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(msg->points[0].x),
      static_cast<int>(msg->points[0].y),
      dvl_altitude_, intrinsics, camera_to_world, true);

  cv::Point3d pt2_3d = PipelineGeometry::backprojectGroundPlane(
      static_cast<int>(msg->points[1].x),
      static_cast<int>(msg->points[1].y),
      dvl_altitude_, intrinsics, camera_to_world, true);

  std::vector<cv::Point3d> endpoints_3d = {pt1_3d, pt2_3d};

  // Select closest endpoint to 3D origin
  cv::Point3d selected_3d = selectClosestEndpointTo3DOrigin(endpoints_3d);

  // Publish as Landmark
  vortex_msgs::msg::Landmark landmark_msg;
  landmark_msg.header.stamp = msg->header.stamp;
  landmark_msg.header.frame_id = "odom";
  landmark_msg.type_class.type = vortex_msgs::msg::LandmarkTypeClass::PIPELINE_START;
  landmark_msg.id = 0;
  landmark_msg.pose.pose.position.x = selected_3d.x;
  landmark_msg.pose.pose.position.y = selected_3d.y;
  landmark_msg.pose.pose.position.z = selected_3d.z;
  landmark_msg.pose.pose.orientation.w = 1.0;

  landmark_pub_->publish(landmark_msg);

  RCLCPP_DEBUG(this->get_logger(),
      "Published landmark: (%.3f, %.3f, %.3f) in odom frame",
      selected_3d.x, selected_3d.y, selected_3d.z);

  // Visualization (if enabled)
  if (enable_debug_image_ && image_viz_ && last_image_) {
    std::vector<cv::Point2d> endpoints_2d = {
      cv::Point2d(msg->points[0].x, msg->points[0].y),
      cv::Point2d(msg->points[1].x, msg->points[1].y)
    };

    image_viz_->visualize(
        last_image_, endpoints_2d, endpoints_3d,
        selected_3d, intrinsics, dvl_altitude_, camera_to_world);
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
    const std::vector<cv::Point3d>& endpoints_3d) {

  if (endpoints_3d.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "selectClosestEndpointTo3DOrigin: expected 2 endpoints");
    return cv::Point3d(0, 0, 0);
  }

  const cv::Point3d& pt1_3d = endpoints_3d[0];
  const cv::Point3d& pt2_3d = endpoints_3d[1];

  // Compute distance to origin (0,0,0)
  double dist1 = std::sqrt(pt1_3d.x * pt1_3d.x + pt1_3d.y * pt1_3d.y + pt1_3d.z * pt1_3d.z);
  double dist2 = std::sqrt(pt2_3d.x * pt2_3d.x + pt2_3d.y * pt2_3d.y + pt2_3d.z * pt2_3d.z);

  // Select endpoint with minimum distance to origin
  if (dist1 < dist2) {
    return pt1_3d;
  } else {
    return pt2_3d;
  }
}
