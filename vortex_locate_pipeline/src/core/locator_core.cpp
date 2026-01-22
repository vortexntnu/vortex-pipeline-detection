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
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);

    // 2. CONNECTED COMPONENTS: Fast isolation of main pipe
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);

    std::cout << "[DEBUG FAST] Found " << numComponents - 1 << " components in mask" << std::endl;

    if (numComponents <= 1) return std::nullopt;

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

    std::cout << "[DEBUG FAST] Largest component area: " << largestArea << " pixels" << std::endl;

    // Create binary mask of main pipe only
    cv::Mat pipeMask = (labels == largestIdx);

    // 3. FIND BOTTOMMOST PIXELS: Get all pixels at maximum Y coordinate
    int maxY = -1;
    std::vector<cv::Point> bottomPixels;

    for (int y = 0; y < pipeMask.rows; y++) {
        for (int x = 0; x < pipeMask.cols; x++) {
            if (pipeMask.at<uchar>(y, x) > 0) {
                if (y > maxY) {
                    maxY = y;
                    bottomPixels.clear();
                    bottomPixels.push_back(cv::Point(x, y));
                } else if (y == maxY) {
                    bottomPixels.push_back(cv::Point(x, y));
                }
            }
        }
    }

    if (bottomPixels.empty()) {
        std::cout << "[DEBUG FAST] No pixels found in pipeline mask" << std::endl;
        return std::nullopt;
    }

    // 4. AVERAGE POSITION: If multiple pixels at bottom, take their centroid
    int sumX = 0;
    for (const auto& pt : bottomPixels) {
        sumX += pt.x;
    }
    int cx = sumX / bottomPixels.size();

    cv::Point startPixel(cx, maxY);

    std::cout << "[DEBUG FAST] Found " << bottomPixels.size() << " pixels at bottom row (y=" << maxY << ")" << std::endl;
    std::cout << "[DEBUG FAST] Start pixel (averaged) at (" << startPixel.x << ", " << startPixel.y << ")" << std::endl;

    return startPixel;
}

std::optional<cv::Point> LocatorCore::findStartPixelWithDebug(const cv::Mat &mask, cv::Mat &debug_out) {
    if (mask.empty()) return std::nullopt;

    // 1. MORPHOLOGY
    cv::Mat clean_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);

    // 2. CONNECTED COMPONENTS
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);

    std::cout << "[DEBUG FAST] Found " << numComponents - 1 << " components in mask" << std::endl;

    if (numComponents <= 1) return std::nullopt;

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

    std::cout << "[DEBUG FAST] Largest component area: " << largestArea << " pixels" << std::endl;

    cv::Mat pipeMask = (labels == largestIdx);

    // 3. FIND BOTTOMMOST PIXELS: Get all pixels at maximum Y coordinate
    int maxY = -1;
    std::vector<cv::Point> bottomPixels;

    for (int y = 0; y < pipeMask.rows; y++) {
        for (int x = 0; x < pipeMask.cols; x++) {
            if (pipeMask.at<uchar>(y, x) > 0) {
                if (y > maxY) {
                    maxY = y;
                    bottomPixels.clear();
                    bottomPixels.push_back(cv::Point(x, y));
                } else if (y == maxY) {
                    bottomPixels.push_back(cv::Point(x, y));
                }
            }
        }
    }

    if (bottomPixels.empty()) {
        std::cout << "[DEBUG FAST] No pixels found in pipeline mask" << std::endl;
        return std::nullopt;
    }

    // 4. AVERAGE POSITION: If multiple pixels at bottom, take their centroid
    int sumX = 0;
    for (const auto& pt : bottomPixels) {
        sumX += pt.x;
    }
    int cx = sumX / bottomPixels.size();

    cv::Point startPixel(cx, maxY);

    std::cout << "[DEBUG FAST] Found " << bottomPixels.size() << " pixels at bottom row (y=" << maxY << ")" << std::endl;
    std::cout << "[DEBUG FAST] Start pixel (averaged) at (" << startPixel.x << ", " << startPixel.y << ")" << std::endl;

    // CREATE DEBUG VISUALIZATION
    cv::cvtColor(pipeMask, debug_out, cv::COLOR_GRAY2BGR);

    // Highlight all bottommost pixels in green
    for (const auto& pt : bottomPixels) {
        cv::circle(debug_out, pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // Mark computed start pixel (average of bottommost) with large red circle
    cv::circle(debug_out, startPixel, 8, cv::Scalar(0, 0, 255), -1);
    cv::circle(debug_out, startPixel, 10, cv::Scalar(255, 255, 255), 2);  // White outline

    return startPixel;
}

cv::Point3d LocatorCore::backproject(int u, int v, double z,
                                     const CameraIntrinsics &intrinsics) {
  // Backproject pixel (u,v) with depth z (meters) to camera frame coordinates
  return cv::Point3d(0, 0, 0);
}

}  // namespace vortex_locate_pipeline
