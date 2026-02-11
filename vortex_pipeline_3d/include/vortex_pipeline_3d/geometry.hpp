#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>
#include <string>

namespace vortex_pipeline_3d {

// Camera calibration parameters including distortion
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::vector<double> D;  // Distortion coefficients
  std::string frame_id;
  bool has_distortion = false;
};

class PipelineGeometry {
public:
  // Backproject pixel to 3D using DVL altitude and flat ground plane
  // altitude: DVL height above ground (meters, positive)
  // Returns: 3D point in camera frame (X=right, Y=down/altitude, Z=forward)
  static cv::Point3d backprojectGroundPlane(
      int u, int v,
      double altitude,
      const CameraIntrinsics &intrinsics,
      bool apply_undistortion = true);

private:
  // Undistort a single pixel using camera distortion model
  static cv::Point2f undistortPoint(const cv::Point &pixel,
                                    const CameraIntrinsics &intrinsics);
};

} // namespace vortex_pipeline_3d
