#include "vortex_pipeline_position_estimator/position_estimator_node.hpp"
#include <algorithm>

namespace vortex_pipeline_position_estimator {

PositionEstimatorNode::PositionEstimatorNode() : Node("pipeline_position_estimator") {
    auto qos = rclcpp::QoS(1).best_effort();

    // Initialize tf2 buffer and listener
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Declare parameters
    auto endpoints_topic = this->declare_parameter<std::string>("endpoints_topic");
    auto camera_info_topic = this->declare_parameter<std::string>("camera_info_topic");
    auto dvl_altitude_topic = this->declare_parameter<std::string>("dvl_altitude_topic");
    auto publish_topic = this->declare_parameter<std::string>("publish_topic");
    transform_timeout_ms_ = this->declare_parameter<int>("transform_timeout_ms");
    apply_undistortion_ = this->declare_parameter<bool>("apply_undistortion");

    // Subscriptions
    endpoints_sub_ = this->create_subscription<vortex_msgs::msg::Point2DArray>(
        endpoints_topic, qos,
        std::bind(&PositionEstimatorNode::endpointsCallback, this, std::placeholders::_1));

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos,
        std::bind(&PositionEstimatorNode::cameraInfoCallback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        dvl_altitude_topic, qos,
        std::bind(&PositionEstimatorNode::dvlCallback, this, std::placeholders::_1));

    // Publishers
    landmark_pub_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(publish_topic, qos);

    RCLCPP_INFO(this->get_logger(), "Pipeline position estimator node started");
    RCLCPP_INFO(this->get_logger(), "  Endpoints: %s", endpoints_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera info: %s", camera_info_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  DVL altitude: %s", dvl_altitude_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish to: %s", publish_topic.c_str());
}

void PositionEstimatorNode::endpointsCallback(
    const vortex_msgs::msg::Point2DArray::SharedPtr msg) {
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty endpoints array");
        return;
    }

    auto snapshot = snapshotData();
    if (!snapshot) {
        return;
    }
    auto [dvl_altitude, intrinsics] = *snapshot;

    auto transform = lookupCameraToWorld(intrinsics.frame_id, msg->header.stamp);
    if (!transform) {
        return;
    }

    // Backproject all endpoints to 3D
    std::vector<cv::Point3d> endpoints_3d;
    endpoints_3d.reserve(msg->points.size());
    for (const auto& pt : msg->points) {
        endpoints_3d.push_back(
            backprojectGroundPlane(static_cast<int>(pt.x), static_cast<int>(pt.y), dvl_altitude,
                                   intrinsics, *transform, apply_undistortion_));
    }

    // Select endpoint closest to 3D origin as pipeline start
    auto closest_to_origin = [](const cv::Point3d& a, const cv::Point3d& b) {
        return (a.x * a.x + a.y * a.y + a.z * a.z) < (b.x * b.x + b.y * b.y + b.z * b.z);
    };
    cv::Point3d selected_3d =
        *std::min_element(endpoints_3d.begin(), endpoints_3d.end(), closest_to_origin);

    // Publish as Landmark
    vortex_msgs::msg::LandmarkArray landmark_msg;
    landmark_msg.header.stamp = msg->header.stamp;
    landmark_msg.header.frame_id = "odom";
    landmark_msg.landmarks.resize(1);
    landmark_msg.landmarks.at(0).type_class.type =
        vortex_msgs::msg::LandmarkTypeClass::PIPELINE_START;
    landmark_msg.landmarks.at(0).id = 0;
    landmark_msg.landmarks.at(0).pose.pose.position.x = selected_3d.x;
    landmark_msg.landmarks.at(0).pose.pose.position.y = selected_3d.y;
    landmark_msg.landmarks.at(0).pose.pose.position.z = selected_3d.z;
    landmark_msg.landmarks.at(0).pose.pose.orientation.w = 1.0;
    landmark_pub_->publish(landmark_msg);

    RCLCPP_DEBUG(this->get_logger(), "DVL altitude: %.3f m", dvl_altitude);
    for (size_t i = 0; i < endpoints_3d.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  EP%zu: (%.3f, %.3f, %.3f) m [odom]", i + 1,
                     endpoints_3d[i].x, endpoints_3d[i].y, endpoints_3d[i].z);
    }
    RCLCPP_DEBUG(this->get_logger(), "  Selected: (%.3f, %.3f, %.3f) m [odom]", selected_3d.x,
                 selected_3d.y, selected_3d.z);
}

std::optional<std::pair<double, CameraIntrinsics>> PositionEstimatorNode::snapshotData() {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);

    if (dvl_altitude_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No valid DVL altitude data available");
        return std::nullopt;
    }
    if (!intrinsics_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No camera info available");
        return std::nullopt;
    }

    return std::make_pair(dvl_altitude_, *intrinsics_);
}

std::optional<geometry_msgs::msg::TransformStamped> PositionEstimatorNode::lookupCameraToWorld(
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    try {
        return tf2_buffer_->lookupTransform(
            "odom",    // target frame (world)
            frame_id,  // source frame (camera frame from camera_info)
            stamp,     // timestamp of observation
            std::chrono::milliseconds(transform_timeout_ms_));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Could not transform %s to odom: %s", frame_id.c_str(), ex.what());
        return std::nullopt;
    }
}

void PositionEstimatorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);

    if (!intrinsics_) {
        CameraIntrinsics intrinsics;
        // msg->k is row-major [fx 0 cx / 0 fy cy / 0 0 1]
        intrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        intrinsics.frame_id = msg->header.frame_id;
        if (!msg->d.empty()) {
            intrinsics.D = cv::Mat(msg->d).clone();
        }
        intrinsics_ = intrinsics;

        RCLCPP_INFO(this->get_logger(), "Received camera info, unsubscribing (static data)");
        caminfo_sub_.reset();
    }
}

void PositionEstimatorNode::dvlCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::unique_lock<std::shared_mutex> lock(data_mutex_);
    dvl_altitude_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received DVL altitude: %.3f m", dvl_altitude_);
}

}  // namespace vortex_pipeline_position_estimator
