#include "vortex_pipeline_position_estimator/backproject_ground_plane.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace vortex_pipeline_position_estimator {

cv::Point3d backprojectGroundPlane(
    int u, int v, double altitude, const CameraIntrinsics& intrinsics,
    const geometry_msgs::msg::TransformStamped& camera_to_world,
    bool apply_undistortion) {

    if (altitude <= 0.0 || std::isnan(altitude) || std::isinf(altitude)) {
        return cv::Point3d(0, 0, 0);
    }
    
    // Undistort pixel if needed
    cv::Point2f pixel(u, v);
    if (apply_undistortion && !intrinsics.D.empty()) {
        std::vector<cv::Point2f> pts = {pixel};
        std::vector<cv::Point2f> undistorted;
        cv::undistortPoints(pts, undistorted, intrinsics.K, intrinsics.D, cv::noArray(), intrinsics.K);
        pixel = undistorted[0];
    }
    
    // Unpack camera matrix
    double fx = intrinsics.K.at<double>(0, 0);
    double fy = intrinsics.K.at<double>(1, 1);
    double cx = intrinsics.K.at<double>(0, 2);
    double cy = intrinsics.K.at<double>(1, 2);

    // Compute ray direction in CAMERA frame (X=right, Y=down, Z=forward)
    double ray_x_cam = (pixel.x - cx) / fx;
    double ray_y_cam = (pixel.y - cy) / fy;
    double ray_z_cam = 1.0;

    // Normalize ray
    double norm = std::sqrt(ray_x_cam*ray_x_cam + ray_y_cam*ray_y_cam + ray_z_cam*ray_z_cam);
    ray_x_cam /= norm;
    ray_y_cam /= norm;
    ray_z_cam /= norm;

    // Rotate ray from CAMERA frame to WORLD frame
    tf2::Quaternion quat;
    tf2::fromMsg(camera_to_world.transform.rotation, quat);
    tf2::Matrix3x3 rotation_matrix(quat);
    tf2::Vector3 ray_world = rotation_matrix * tf2::Vector3(ray_x_cam, ray_y_cam, ray_z_cam);

    double ray_x_world = ray_world.getX();
    double ray_y_world = ray_world.getY();
    double ray_z_world = ray_world.getZ();

    // Intersect ray with ground plane in WORLD frame (NED: X=North, Y=East, Z=Down)
    // Ground plane at Z = cam_z + altitude. Solve: t * ray_z_world = altitude
    if (ray_z_world < 1e-6) return cv::Point3d(0, 0, 0);  // ray parallel to or pointing away from ground

    double t = altitude / ray_z_world;

    double cam_x = camera_to_world.transform.translation.x;
    double cam_y = camera_to_world.transform.translation.y;
    double cam_z = camera_to_world.transform.translation.z;

    // Ray equation: P = camera_pos + t * ray_direction
    return cv::Point3d(
        cam_x + ray_x_world * t,  // North
        cam_y + ray_y_world * t,  // East
        cam_z + altitude          // Down (ground plane below camera)
    );
}

}  // namespace vortex_pipeline_position_estimator
