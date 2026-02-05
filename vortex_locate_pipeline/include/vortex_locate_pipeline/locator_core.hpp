// Core, non-ROS image/geometry utilities for pipeline locating
#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace vortex_locate_pipeline {

// Camera calibration parameters including distortion
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::vector<double> D;      // Distortion coefficients [k1, k2, p1, p2, k3]
  std::string frame_id;
  bool has_distortion = false;
};

// Result of endpoint detection containing both detected endpoints
struct PipelineEndpoints {
  cv::Point endpoint1;        // First detected endpoint (pixel coords)
  cv::Point endpoint2;        // Second detected endpoint (pixel coords)
  bool found_both = false;    // True if two distinct endpoints found
};

class LocatorCore {
public:
  // SKELETON-BASED METHOD: Find both pipeline endpoints in binary mask
  // Returns struct with both endpoints if found
  // use_skeleton_method: if true, uses skeleton extraction; if false, falls back to bottom-most
  // debug_out: optional debug visualization (nullptr = no debug)
  static std::optional<PipelineEndpoints> findPipelineEndpoints(
      const cv::Mat &mask,
      bool use_skeleton_method = true,
      cv::Mat *debug_out = nullptr);

  // LEGACY FALLBACK: Find bottom-most pixel in largest component
  // Kept as fallback option for when skeleton method fails
  // debug_out: optional debug visualization (nullptr = no debug)
  static std::optional<cv::Point> findStartPixelBottomMost(
      const cv::Mat &mask,
      cv::Mat *debug_out = nullptr);

  // BACKWARD COMPATIBLE: Wrapper that returns single pixel
  // Uses skeleton method and returns first endpoint
  static std::optional<cv::Point> findStartPixel(
      const cv::Mat &mask,
      cv::Mat *debug_out = nullptr);

  // Backproject pixel (u,v) to 3D using DVL altitude and flat ground plane assumption
  // altitude: DVL height above ground (meters, positive value)
  // intrinsics: camera calibration parameters
  // apply_undistortion: if true and intrinsics.has_distortion, undistort pixel first
  // Returns 3D point in camera frame (Z is negative = below camera)
  static cv::Point3d backprojectGroundPlane(
      int u, int v,
      double altitude,
      const CameraIntrinsics &intrinsics,
      bool apply_undistortion = true);

  // Legacy depth-based backprojection (kept for compatibility)
  static cv::Point3d backproject(int u, int v, double z,
                                 const CameraIntrinsics &intrinsics);

private:
  // Find two points furthest apart in binary mask using ConvexHull
  // Returns pair of endpoints if successful, nullopt otherwise
  static std::optional<std::pair<cv::Point, cv::Point>>
      findFurthestPoints(const cv::Mat &binary);

  // Extract 1-pixel-wide skeleton from binary mask (DEPRECATED - replaced by ConvexHull)
  static cv::Mat extractSkeleton(const cv::Mat &binary);

  // Find all endpoints in skeleton (pixels with exactly 1 neighbor) (DEPRECATED)
  static std::vector<cv::Point> findSkeletonEndpoints(const cv::Mat &skeleton);

  // Undistort a single pixel using camera distortion model
  // Returns undistorted pixel coordinates
  static cv::Point2f undistortPoint(const cv::Point &pt,
                                    const CameraIntrinsics &intrinsics);
};

} // namespace vortex_locate_pipeline
