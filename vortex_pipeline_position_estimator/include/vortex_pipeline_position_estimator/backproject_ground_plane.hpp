#pragma once

#include <opencv2/opencv.hpp>

namespace vortex_pipeline_position_estimator {

// Camera calibration parameters
struct CameraIntrinsics {
    cv::Mat K;  // 3x3 camera matrix
    cv::Mat D;  // Distortion coefficients (empty = no distortion)
};

// Backproject pixel to 3D using DVL altitude and flat ground plane assumption.
// altitude: DVL height above ground (meters, positive)
// rotation: 3x3 rotation matrix from camera frame to world frame
// translation: camera position in world frame
// Returns: 3D point in world frame (NED: X=North, Y=East, Z=Down)
cv::Point3d backprojectGroundPlane(int u,
                                   int v,
                                   double altitude,
                                   const CameraIntrinsics& intrinsics,
                                   const cv::Matx33d& rotation,
                                   const cv::Vec3d& translation,
                                   bool apply_undistortion = true);

}  // namespace vortex_pipeline_position_estimator
