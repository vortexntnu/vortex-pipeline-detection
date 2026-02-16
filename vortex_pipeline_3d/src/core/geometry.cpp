// Implementation of 3D geometry and backprojection utilities.
// This file is independent of ROS and can be reused by other packages.

#include "vortex_pipeline_3d/geometry.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <cmath>

namespace vortex_pipeline_3d {

// ============================================================================
// LENS DISTORTION CORRECTION
// ============================================================================

cv::Point2f PipelineGeometry::undistortPoint(const cv::Point &pixel,
                                             const CameraIntrinsics &intrinsics) {
    // If no distortion, return original point
    if (!intrinsics.has_distortion || intrinsics.D.empty()) {
        return cv::Point2f(pixel.x, pixel.y);
    }

    // Create intrinsics matrix K
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0, intrinsics.cx,
        0, intrinsics.fy, intrinsics.cy,
        0, 0, 1);

    // Create distortion vector
    cv::Mat D = cv::Mat(intrinsics.D);

    // Undistort using OpenCV
    std::vector<cv::Point2f> distorted_pts = {cv::Point2f(pixel.x, pixel.y)};
    std::vector<cv::Point2f> undistorted_pts;
    cv::undistortPoints(distorted_pts, undistorted_pts, K, D, cv::noArray(), K);

    return undistorted_pts[0];
}

// ============================================================================
// 3D BACKPROJECTION: DVL + Flat Ground Plane
// ============================================================================

cv::Point3d PipelineGeometry::backprojectGroundPlane(
    int u, int v, double altitude, const CameraIntrinsics &intrinsics,
    const geometry_msgs::msg::TransformStamped& camera_to_world,
    bool apply_undistortion) {

    // Handle invalid altitude
    if (altitude <= 0.0 || std::isnan(altitude) || std::isinf(altitude)) {
        return cv::Point3d(0, 0, 0);
    }

    // Undistort pixel if needed
    cv::Point2f pixel(u, v);
    if (apply_undistortion && intrinsics.has_distortion) {
        pixel = undistortPoint(cv::Point(u, v), intrinsics);
    }

    // Compute ray direction in CAMERA frame
    // Camera frame: X=right, Y=down, Z=forward
    double ray_x_cam = (pixel.x - intrinsics.cx) / intrinsics.fx;
    double ray_y_cam = (pixel.y - intrinsics.cy) / intrinsics.fy;
    double ray_z_cam = 1.0;

    // Normalize ray
    double norm = std::sqrt(ray_x_cam*ray_x_cam +
                           ray_y_cam*ray_y_cam +
                           ray_z_cam*ray_z_cam);
    ray_x_cam /= norm;
    ray_y_cam /= norm;
    ray_z_cam /= norm;

    // Rotate ray from CAMERA frame to WORLD frame using full tf2 transform
    tf2::Quaternion quat;
    tf2::fromMsg(camera_to_world.transform.rotation, quat);
    tf2::Matrix3x3 rotation_matrix(quat);

    // Create camera ray as tf2::Vector3
    tf2::Vector3 ray_cam(ray_x_cam, ray_y_cam, ray_z_cam);

    // Apply full rotation matrix
    tf2::Vector3 ray_world = rotation_matrix * ray_cam;

    double ray_x_world = ray_world.getX();
    double ray_y_world = ray_world.getY();
    double ray_z_world = ray_world.getZ();

    // Intersect ray with ground plane in WORLD frame (NED convention)
    // Ground plane: Z_world = altitude (below camera at Z=0)
    // NED frame: X=North (forward), Y=East (right), Z=Down
    // Ray: P = t * (ray_x_world, ray_y_world, ray_z_world)
    // Solve: t * ray_z_world = altitude

    if (std::abs(ray_z_world) < 1e-6) {
        // Ray nearly parallel to ground plane
        return cv::Point3d(0, 0, 0);
    }

    if (ray_z_world < 0) {
        // Ray points upward - won't hit ground below
        return cv::Point3d(0, 0, 0);
    }

    double t = altitude / ray_z_world;

    // Get camera position in world frame
    double cam_x = camera_to_world.transform.translation.x;
    double cam_y = camera_to_world.transform.translation.y;
    double cam_z = camera_to_world.transform.translation.z;

    // 3D point on ground in WORLD frame (NED)
    // Ray equation: P = camera_pos + t * ray_direction
    double X = cam_x + ray_x_world * t;  // North
    double Y = cam_y + ray_y_world * t;  // East
    double Z = cam_z + altitude;         // Down (ground plane below camera)

    return cv::Point3d(X, Y, Z);
}

}  // namespace vortex_pipeline_3d
