#include "vortex_pipeline_position_estimator/position_estimator_node.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

namespace vortex_pipeline_position_estimator {

PositionEstimatorNode::PositionEstimatorNode(const rclcpp::NodeOptions& options)
    : Node("pipeline_position_estimator", options) {
    auto qos = rclcpp::QoS(1).best_effort();

    // Initialize tf2 buffer and listener
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_, this);

    // Declare parameters
    auto endpoints_topic = this->declare_parameter<std::string>("endpoints_topic");
    auto camera_info_topic = this->declare_parameter<std::string>("camera_info_topic");
    auto dvl_altitude_topic = this->declare_parameter<std::string>("dvl_altitude_topic");
    auto publish_topic = this->declare_parameter<std::string>("publish_topic");
    transform_timeout_ms_ = this->declare_parameter<int>("transform_timeout_ms");
    apply_undistortion_ = this->declare_parameter<bool>("apply_undistortion");
    reference_frame_ = this->declare_parameter<std::string>("reference_frame");

    // Subscriptions
    endpoints_sub_ = this->create_subscription<vortex_msgs::msg::Point2DArray>(
        endpoints_topic, qos,
        std::bind(&PositionEstimatorNode::endpoints_callback, this, std::placeholders::_1));

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, qos,
        std::bind(&PositionEstimatorNode::camera_info_callback, this, std::placeholders::_1));

    dvl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        dvl_altitude_topic, qos,
        std::bind(&PositionEstimatorNode::dvl_callback, this, std::placeholders::_1));

    // Publishers
    landmark_pub_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(publish_topic, qos);

    RCLCPP_INFO(this->get_logger(), "Pipeline position estimator node started");
    RCLCPP_INFO(this->get_logger(), "  Endpoints: %s", endpoints_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera info: %s", camera_info_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  DVL altitude: %s", dvl_altitude_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish to: %s", publish_topic.c_str());
}

void PositionEstimatorNode::endpoints_callback(
    const vortex_msgs::msg::Point2DArray::SharedPtr msg) {
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty endpoints array");
        return;
    }

    if (dvl_altitude_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No valid DVL altitude data available");
        return;
    }
    if (!intrinsics_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No camera info available");
        return;
    }

    auto transform = lookup_camera_to_world(camera_frame_id_, msg->header.stamp);
    if (!transform) {
        return;
    }

    // Convert ROS transform to plain math types for core library
    tf2::Quaternion quat;
    tf2::fromMsg(transform->transform.rotation, quat);
    tf2::Matrix3x3 rot(quat);
    // clang-format off
    cv::Matx33d rotation(rot[0][0], rot[0][1], rot[0][2],
                         rot[1][0], rot[1][1], rot[1][2],
                         rot[2][0], rot[2][1], rot[2][2]);
    cv::Vec3d translation(transform->transform.translation.x,
                          transform->transform.translation.y,
                          transform->transform.translation.z);
    // clang-format on

    // Backproject all endpoints to 3D
    std::vector<cv::Point3d> endpoints_3d;
    endpoints_3d.reserve(msg->points.size());
    for (const auto& pt : msg->points) {
        endpoints_3d.push_back(
            backprojectGroundPlane(static_cast<int>(pt.x), static_cast<int>(pt.y), dvl_altitude_,
                                   *intrinsics_, rotation, translation, apply_undistortion_));
    }

    // Select endpoint closest to 3D origin as pipeline start
    auto closest_to_origin = [](const cv::Point3d& a, const cv::Point3d& b) {
        return (a.x * a.x + a.y * a.y + a.z * a.z) < (b.x * b.x + b.y * b.y + b.z * b.z);
    };
    cv::Point3d selected_3d =
        *std::min_element(endpoints_3d.begin(), endpoints_3d.end(), closest_to_origin);

    landmark_pub_->publish(build_landmark_msg(msg->header.stamp, selected_3d, endpoints_3d));

    RCLCPP_DEBUG(this->get_logger(), "DVL altitude: %.3f m", dvl_altitude_);
    for (size_t i = 0; i < endpoints_3d.size(); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  EP%zu: (%.3f, %.3f, %.3f) m [%s]", i + 1,
                     endpoints_3d[i].x, endpoints_3d[i].y, endpoints_3d[i].z,
                     reference_frame_.c_str());
    }
    RCLCPP_DEBUG(this->get_logger(), "  Selected: (%.3f, %.3f, %.3f) m [%s]", selected_3d.x,
                 selected_3d.y, selected_3d.z, reference_frame_.c_str());
}

std::optional<geometry_msgs::msg::TransformStamped> PositionEstimatorNode::lookup_camera_to_world(
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    try {
        return tf2_buffer_->lookupTransform(
            reference_frame_,  // target frame (world)
            frame_id,          // source frame (camera frame from camera_info)
            stamp,             // timestamp of observation
            std::chrono::milliseconds(transform_timeout_ms_));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Could not transform %s to %s: %s", frame_id.c_str(),
                             reference_frame_.c_str(), ex.what());
        return std::nullopt;
    }
}

vortex_msgs::msg::LandmarkArray PositionEstimatorNode::build_landmark_msg(
    const rclcpp::Time& stamp,
    const cv::Point3d& selected_3d,
    const std::vector<cv::Point3d>& endpoints_3d) {
    vortex_msgs::msg::LandmarkArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = reference_frame_;
    msg.landmarks.resize(1);
    msg.landmarks.at(0).type.value = vortex_msgs::msg::LandmarkType::PIPELINE_START;
    msg.landmarks.at(0).subtype.value = vortex_msgs::msg::LandmarkSubtype::PIPELINE_START_CAMERA;
    msg.landmarks.at(0).id = 0;
    msg.landmarks.at(0).pose.pose.position.x = selected_3d.x;
    msg.landmarks.at(0).pose.pose.position.y = selected_3d.y;
    msg.landmarks.at(0).pose.pose.position.z = selected_3d.z;
    msg.landmarks.at(0).pose.pose.orientation.x = 0.0;
    msg.landmarks.at(0).pose.pose.orientation.y = 0.0;
    if (endpoints_3d.size() == 2) {
        auto closest_to_origin = [](const cv::Point3d& a, const cv::Point3d& b) {
            return (a.x * a.x + a.y * a.y + a.z * a.z) < (b.x * b.x + b.y * b.y + b.z * b.z);
        };
        // The far endpoint is the one NOT selected as start — compute the pipeline direction
        // from start toward far end, expressed as a pure yaw quaternion (x=0, y=0)
        auto furthest =
            std::max_element(endpoints_3d.begin(), endpoints_3d.end(), closest_to_origin);
        double yaw = std::atan2(furthest->y - selected_3d.y, furthest->x - selected_3d.x);
        msg.landmarks.at(0).pose.pose.orientation.z = std::sin(yaw / 2.0);
        msg.landmarks.at(0).pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
        // Only one endpoint detected — no direction information, use identity orientation
        msg.landmarks.at(0).pose.pose.orientation.z = 0.0;
        msg.landmarks.at(0).pose.pose.orientation.w = 1.0;
    }
    return msg;
}

void PositionEstimatorNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!intrinsics_) {
        CameraIntrinsics intrinsics;
        // msg->k is row-major [fx 0 cx / 0 fy cy / 0 0 1]
        intrinsics.K = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        camera_frame_id_ = msg->header.frame_id;
        if (!msg->d.empty()) {
            intrinsics.D = cv::Mat(msg->d).clone();
        }
        intrinsics_ = intrinsics;

        RCLCPP_INFO(this->get_logger(), "Received camera info, unsubscribing (static data)");
        caminfo_sub_.reset();
    }
}

void PositionEstimatorNode::dvl_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    dvl_altitude_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Received DVL altitude: %.3f m", dvl_altitude_);
}

}  // namespace vortex_pipeline_position_estimator

RCLCPP_COMPONENTS_REGISTER_NODE(vortex_pipeline_position_estimator::PositionEstimatorNode)
