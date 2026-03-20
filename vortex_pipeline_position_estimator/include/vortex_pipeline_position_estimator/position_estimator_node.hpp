#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <vortex_msgs/msg/point2_d_array.hpp>
#include "vortex_pipeline_position_estimator/backproject_ground_plane.hpp"

namespace vortex_pipeline_position_estimator {

class PositionEstimatorNode : public rclcpp::Node {
   public:
    explicit PositionEstimatorNode(const rclcpp::NodeOptions& options);

   private:
    // Callbacks
    void endpoints_callback(const vortex_msgs::msg::Point2DArray::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void dvl_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // Helper methods
    std::optional<geometry_msgs::msg::TransformStamped> lookup_camera_to_world(
        const std::string& frame_id,
        const rclcpp::Time& stamp);
    vortex_msgs::msg::LandmarkArray build_landmark_msg(
        const rclcpp::Time& stamp,
        const cv::Point3d& selected_3d,
        const std::vector<cv::Point3d>& endpoints_3d);

    // ROS interfaces
    rclcpp::Subscription<vortex_msgs::msg::Point2DArray>::SharedPtr endpoints_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dvl_sub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;

    std::optional<CameraIntrinsics> intrinsics_;
    double dvl_altitude_{0.0};

    // tf2 for frame transformations
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // Parameters
    int transform_timeout_ms_;
    bool apply_undistortion_;
    std::string reference_frame_;
    std::string camera_frame_id_;
};

}  // namespace vortex_pipeline_position_estimator
