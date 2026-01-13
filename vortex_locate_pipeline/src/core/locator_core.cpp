// Implementation of core, non-ROS logic for pipeline locating.
// This file contains image-processing utilities that are independent of ROS
// and can be reused by other packages. Keep ROS types out of this file.

#include "vortex_locate_pipeline/locator_core.hpp"

namespace vortex_locate_pipeline {

std::optional<cv::Point> LocatorCore::findStartPixel(const cv::Mat &mask) {
    if (mask.empty()) return std::nullopt;
    
    // 1. MORPHOLOGY: Clean underwater segmentation artifacts
    cv::Mat clean_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);  // Fill holes from reflections
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);  // Remove debris
    
    // 2. CONNECTED COMPONENTS: Fast isolation of main pipe
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);
    
    if (numComponents <= 1) return std::nullopt;  // Only background
    
    // Find largest component (skip index 0 = background)
    int largestIdx = 1;
    int largestArea = stats.at<int>(1, cv::CC_STAT_AREA);
    for (int i = 2; i < numComponents; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > largestArea) {
            largestArea = area;
            largestIdx = i;
        }
    }
    
    // Create binary mask of main pipe only
    cv::Mat pipeMask = (labels == largestIdx);
    
    // 3. DISTANCE TRANSFORM: Find geometric centerline
    cv::Mat dist;
    cv::distanceTransform(pipeMask, dist, cv::DIST_L2, 3);
    
    // 4. Search bottom 20% of image
    int bottomStart = static_cast<int>(dist.rows * 0.80);
    cv::Mat bottomRegion = dist(cv::Rect(0, bottomStart, dist.cols, dist.rows - bottomStart));
    
    // 5. Find centerline point (local maximum)
    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(bottomRegion, nullptr, &maxVal, nullptr, &maxLoc);
    
    // Validate: For fixed-size underwater pipe, center should be at least 2 pixels from edge
    if (maxVal < 2.0) return std::nullopt;
    
    // Adjust coordinates back to full image
    maxLoc.y += bottomStart;
    return maxLoc;
}

cv::Point3d LocatorCore::backproject(int u, int v, double z,
                                     const CameraIntrinsics &intrinsics) {
  // Backproject pixel (u,v) with depth z (meters) to camera frame coordinates
  return cv::Point3d(0, 0, 0);
}

}  // namespace vortex_locate_pipeline
