// Core, non-ROS image/geometry utilities for pipeline locating
#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace vortex_locate_pipeline {

struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::string frame_id;
};

class LocatorCore {
public:
  // Find pipeline starting pixel in a binary mask (mono8). Returns (u,v).
  static std::optional<cv::Point> findStartPixel(const cv::Mat &mask);

  // Backproject pixel (u,v) with depth z (meters) and intrinsics to 3D point
  // in camera frame.
  static cv::Point3d backproject(int u, int v, double z,
                                 const CameraIntrinsics &intrinsics);
};

} // namespace vortex_locate_pipeline
