#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace vortex_pipeline_endpoints {

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

class PipelineDetector {
public:
  // Find both pipeline endpoints using ConvexHull furthest-points method
  // Returns struct with both endpoints if found
  // debug_out: optional debug visualization (nullptr = no debug)
  static std::optional<PipelineEndpoints> findPipelineEndpoints(
      const cv::Mat &mask,
      cv::Mat *debug_out = nullptr);

private:
  // Find two points furthest apart in binary mask using ConvexHull
  // Returns pair of endpoints if successful, nullopt otherwise
  static std::optional<std::pair<cv::Point, cv::Point>>
      findFurthestPoints(const cv::Mat &binary);

  // Undistort a single pixel using camera distortion model
  // Returns undistorted pixel coordinates
  static cv::Point2f undistortPoint(const cv::Point &pixel,
                                    const CameraIntrinsics &intrinsics);
};

} // namespace vortex_pipeline_endpoints
