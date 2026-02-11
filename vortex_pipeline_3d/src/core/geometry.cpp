// Implementation of 3D geometry and backprojection utilities.
// This file is independent of ROS and can be reused by other packages.

#include "vortex_pipeline_3d/geometry.hpp"
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

    std::cout << "[DEBUG UNDISTORT] (" << pixel.x << "," << pixel.y << ") -> ("
              << undistorted_pts[0].x << "," << undistorted_pts[0].y << ")" << std::endl;

    return undistorted_pts[0];
}

// ============================================================================
// 3D BACKPROJECTION: DVL + Flat Ground Plane
// ============================================================================

cv::Point3d PipelineGeometry::backprojectGroundPlane(
    int u, int v, double altitude, const CameraIntrinsics &intrinsics,
    bool apply_undistortion) {

    // Handle invalid altitude
    if (altitude <= 0.0 || std::isnan(altitude) || std::isinf(altitude)) {
        std::cout << "[WARN] Invalid altitude: " << altitude << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    // Undistort pixel if needed
    cv::Point2f pixel(u, v);
    if (apply_undistortion && intrinsics.has_distortion) {
        pixel = undistortPoint(cv::Point(u, v), intrinsics);
    }

    // Compute ray direction from camera through pixel
    // Camera frame convention: X=right, Y=down, Z=forward
    double ray_x = (pixel.x - intrinsics.cx) / intrinsics.fx;
    double ray_y = (pixel.y - intrinsics.cy) / intrinsics.fy;
    double ray_z = 1.0;

    // Intersect ray with ground plane at Y = altitude (downward from camera)
    // Ray equation: P = t * (ray_x, ray_y, ray_z)
    // Ground plane: Y = altitude
    // Solve for t: t * ray_y = altitude => t = altitude / ray_y
    if (std::abs(ray_y) < 1e-6) {
        std::cout << "[WARN] Ray nearly parallel to ground plane (ray_y ≈ 0)" << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    double t = altitude / ray_y;

    // 3D point on ground in camera frame
    double X = ray_x * t;
    double Y = altitude;  // On the ground plane
    double Z = ray_z * t;  // Forward distance from camera

    std::cout << "[DEBUG BACKPROJECT] Pixel (" << u << "," << v
              << ") + altitude " << altitude << "m => Camera 3D ("
              << X << "," << Y << "," << Z << ")" << std::endl;

    return cv::Point3d(X, Y, Z);
}

}  // namespace vortex_pipeline_3d
