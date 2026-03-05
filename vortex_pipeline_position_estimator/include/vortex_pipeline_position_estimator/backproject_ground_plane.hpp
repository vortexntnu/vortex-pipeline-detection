#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace vortex_pipeline_position_estimator {

// Camera calibration parameters
struct CameraIntrinsics {
    cv::Mat K;  // 3x3 camera matrix
    cv::Mat D;  // Distortion coefficients (empty = no distortion)
    std::string frame_id;
};

// Backproject pixel to 3D using DVL altitude and flat ground plane assumption.
// altitude: DVL height above ground (meters, positive)
// camera_to_world: Full 6-DOF transform from camera frame to world frame (from tf2)
//                  Includes rotation (roll, pitch, yaw) and translation
// Returns: 3D point in world frame (NED: X=North, Y=East, Z=Down)
cv::Point3d backprojectGroundPlane(int u,
                                   int v,
                                   double altitude,
                                   const CameraIntrinsics& intrinsics,
                                   const geometry_msgs::msg::TransformStamped& camera_to_world,
                                   bool apply_undistortion = true);

}  // namespace vortex_pipeline_position_estimator
