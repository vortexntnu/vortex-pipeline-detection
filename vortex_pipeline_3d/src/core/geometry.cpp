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
    double pitch_angle, bool apply_undistortion) {

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

    // Rotate ray from CAMERA frame to WORLD frame using pitch
    // When vehicle pitches down θ, we apply INVERSE rotation to go camera→world
    // Rotation matrix R_x(-θ) for camera-to-world transform:
    // [ 1      0         0    ] [ray_x]
    // [ 0   cos(θ)   sin(θ) ] [ray_y]
    // [ 0  -sin(θ)   cos(θ) ] [ray_z]

    double cos_pitch = std::cos(pitch_angle);
    double sin_pitch = std::sin(pitch_angle);

    double ray_x_world = ray_x_cam;
    double ray_y_world = cos_pitch * ray_y_cam + sin_pitch * ray_z_cam;
    double ray_z_world = -sin_pitch * ray_y_cam + cos_pitch * ray_z_cam;

    // Intersect ray with ground plane in WORLD frame
    // Ground plane: Y_world = altitude (below camera at Y=0)
    // Ray: P = t * (ray_x_world, ray_y_world, ray_z_world)
    // Solve: t * ray_y_world = altitude

    if (std::abs(ray_y_world) < 1e-6) {
        std::cout << "[WARN] Ray nearly parallel to ground plane" << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    if (ray_y_world < 0) {
        // Ray points upward - won't hit ground below
        std::cout << "[WARN] Ray points upward (pitch too small or pixel above horizon)"
                  << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    double t = altitude / ray_y_world;

    // 3D point on ground in WORLD frame
    double X = ray_x_world * t;
    double Y = altitude;  // On the ground plane
    double Z = ray_z_world * t;  // Should now be POSITIVE

    std::cout << "[DEBUG BACKPROJECT] Pixel (" << u << "," << v
              << ") pitch=" << (pitch_angle * 180.0 / M_PI) << "deg"
              << " => World 3D (" << X << "," << Y << "," << Z << ")"
              << std::endl;

    return cv::Point3d(X, Y, Z);
}

}  // namespace vortex_pipeline_3d
