// Implementation of core 2D endpoint detection logic.
// This file contains image-processing utilities that are independent of ROS
// and can be reused by other packages. Keep ROS types out of this file.

#include "vortex_pipeline_endpoints/detector.hpp"

namespace vortex_pipeline_endpoints {

// ============================================================================
// FURTHEST POINTS: Find two points furthest apart using ConvexHull
// ============================================================================

std::optional<std::pair<cv::Point, cv::Point>>
PipelineDetector::findFurthestPoints(const cv::Mat &binary) {
    // Find contours of the binary mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return std::nullopt;
    }

    // Get largest contour (should be the main pipeline)
    auto largest_it = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    std::vector<cv::Point> largest_contour = *largest_it;
    // Compute convex hull to find extreme points
    std::vector<cv::Point> hull;
    cv::convexHull(largest_contour, hull);

    if (hull.size() < 2) {
        return std::nullopt;
    }

    // Find two hull points with maximum distance
    double max_dist = 0;
    cv::Point pt1, pt2;

    for (size_t i = 0; i < hull.size(); i++) {
        for (size_t j = i + 1; j < hull.size(); j++) {
            double dist = cv::norm(hull[i] - hull[j]);
            if (dist > max_dist) {
                max_dist = dist;
                pt1 = hull[i];
                pt2 = hull[j];
            }
        }
    }

    // Validate we found meaningful endpoints (at least 10 pixels apart)
    if (max_dist < 10) {
        return std::nullopt;
    }

    return std::make_pair(pt1, pt2);
}

// ============================================================================
// CONVEXHULL-BASED ENDPOINT DETECTION
// ============================================================================

std::optional<PipelineEndpoints> PipelineDetector::findPipelineEndpoints(
    const cv::Mat &mask, cv::Mat *debug_out) {

    if (mask.empty()) return std::nullopt;

    // 1. MORPHOLOGY: Clean mask
    cv::Mat clean_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);

    // 2. CONNECTED COMPONENTS: Isolate largest component
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);

    if (numComponents <= 1) {
        return std::nullopt;
    }

    // Find largest component
    int largestIdx = 1;
    int largestArea = stats.at<int>(1, cv::CC_STAT_AREA);
    for (int i = 2; i < numComponents; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > largestArea) {
            largestArea = area;
            largestIdx = i;
        }
    }

    cv::Mat pipeMask = (labels == largestIdx);

    // 3. FURTHEST POINTS DETECTION using ConvexHull
    auto furthest = findFurthestPoints(pipeMask);

    if (!furthest) {
        return std::nullopt;
    }

    // Success - have both endpoints from ConvexHull
    PipelineEndpoints result;
    result.endpoint1 = furthest->first;
    result.endpoint2 = furthest->second;
    result.found_both = true;

    // 4. DEBUG VISUALIZATION (if requested)
    if (debug_out != nullptr) {
        // Convert to color for visualization
        cv::cvtColor(pipeMask, *debug_out, cv::COLOR_GRAY2BGR);

        // Draw line between endpoints to show detected pipe axis
        if (result.found_both) {
            cv::line(*debug_out, result.endpoint1, result.endpoint2,
                    cv::Scalar(255, 255, 0), 2);  // Cyan axis line
        }

        // Mark first endpoint (endpoint1) in green
        cv::circle(*debug_out, result.endpoint1, 8, cv::Scalar(0, 255, 0), -1);
        cv::putText(*debug_out, "EP1",
                    cv::Point(result.endpoint1.x + 12, result.endpoint1.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        // Mark second endpoint (endpoint2) in magenta if different
        if (result.found_both) {
            cv::circle(*debug_out, result.endpoint2, 8, cv::Scalar(255, 0, 255), -1);
            cv::putText(*debug_out, "EP2",
                        cv::Point(result.endpoint2.x + 12, result.endpoint2.y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
        }
    }

    return result;
}

// ============================================================================
// LENS DISTORTION CORRECTION
// ============================================================================

cv::Point2f PipelineDetector::undistortPoint(const cv::Point &pixel,
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

}  // namespace vortex_pipeline_endpoints
