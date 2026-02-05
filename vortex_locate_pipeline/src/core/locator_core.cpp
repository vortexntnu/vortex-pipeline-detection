// Implementation of core, non-ROS logic for pipeline locating.
// This file contains image-processing utilities that are independent of ROS
// and can be reused by other packages. Keep ROS types out of this file.

#include "vortex_locate_pipeline/locator_core.hpp"
#include <iostream>
#include <cmath>

namespace vortex_locate_pipeline {

// ============================================================================
// LEGACY FALLBACK METHOD: Bottom-most pixel detection
// ============================================================================

std::optional<cv::Point> LocatorCore::findStartPixelBottomMost(
    const cv::Mat &mask, cv::Mat *debug_out) {
    if (mask.empty()) return std::nullopt;

    // 1. MORPHOLOGY: Clean underwater segmentation artifacts
    cv::Mat clean_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);

    // 2. CONNECTED COMPONENTS: Fast isolation of main pipe
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);

    std::cout << "[DEBUG BOTTOM-MOST] Found " << numComponents - 1 << " components in mask" << std::endl;

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

    std::cout << "[DEBUG BOTTOM-MOST] Largest component area: " << largestArea << " pixels" << std::endl;

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
        std::cout << "[DEBUG BOTTOM-MOST] No pixels found in pipeline mask" << std::endl;
        return std::nullopt;
    }

    // 4. AVERAGE POSITION: If multiple pixels at bottom, take their centroid
    int sumX = 0;
    for (const auto& pt : bottomPixels) {
        sumX += pt.x;
    }
    int cx = sumX / bottomPixels.size();

    cv::Point startPixel(cx, maxY);

    std::cout << "[DEBUG BOTTOM-MOST] Found " << bottomPixels.size()
              << " pixels at bottom row (y=" << maxY << ")" << std::endl;
    std::cout << "[DEBUG BOTTOM-MOST] Start pixel (averaged) at ("
              << startPixel.x << ", " << startPixel.y << ")" << std::endl;

    // CREATE DEBUG VISUALIZATION (if requested)
    if (debug_out != nullptr) {
        cv::cvtColor(pipeMask, *debug_out, cv::COLOR_GRAY2BGR);

        // Highlight all bottommost pixels in green
        for (const auto& pt : bottomPixels) {
            cv::circle(*debug_out, pt, 2, cv::Scalar(0, 255, 0), -1);
        }

        // Mark computed start pixel (average of bottommost) with large red circle
        cv::circle(*debug_out, startPixel, 8, cv::Scalar(0, 0, 255), -1);
        cv::circle(*debug_out, startPixel, 10, cv::Scalar(255, 255, 255), 2);  // White outline
    }

    return startPixel;
}

// ============================================================================
// FURTHEST POINTS: Find two points furthest apart using ConvexHull
// ============================================================================

std::optional<std::pair<cv::Point, cv::Point>>
LocatorCore::findFurthestPoints(const cv::Mat &binary) {
    // Find contours of the binary mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cout << "[DEBUG FURTHEST] No contours found" << std::endl;
        return std::nullopt;
    }

    // Get largest contour (should be the main pipeline)
    auto largest_it = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    std::vector<cv::Point> largest_contour = *largest_it;
    std::cout << "[DEBUG FURTHEST] Largest contour has " << largest_contour.size()
              << " points, area = " << cv::contourArea(largest_contour) << std::endl;

    // Compute convex hull to find extreme points
    std::vector<cv::Point> hull;
    cv::convexHull(largest_contour, hull);

    std::cout << "[DEBUG FURTHEST] Convex hull has " << hull.size() << " points" << std::endl;

    if (hull.size() < 2) {
        std::cout << "[WARN] Convex hull too small" << std::endl;
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
        std::cout << "[WARN] Furthest points too close: " << max_dist << " pixels" << std::endl;
        return std::nullopt;
    }

    std::cout << "[DEBUG FURTHEST] Found endpoints at (" << pt1.x << "," << pt1.y
              << ") and (" << pt2.x << "," << pt2.y << ") with distance "
              << max_dist << " pixels" << std::endl;

    return std::make_pair(pt1, pt2);
}

// ============================================================================
// SKELETON EXTRACTION: Morphological thinning to 1-pixel centerline (DEPRECATED)
// ============================================================================

cv::Mat LocatorCore::extractSkeleton(const cv::Mat &binary) {
    // Morphological skeleton using iterative erosion
    // See: https://homepages.inf.ed.ac.uk/rbf/HIPR2/skeleton.htm

    cv::Mat skeleton = cv::Mat::zeros(binary.size(), CV_8U);
    cv::Mat temp;
    binary.copyTo(temp);
    cv::Mat eroded, opening;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    int iteration = 0;
    while (true) {
        cv::erode(temp, eroded, element);
        cv::morphologyEx(eroded, opening, cv::MORPH_OPEN, element);
        cv::subtract(temp, opening, opening);
        cv::bitwise_or(skeleton, opening, skeleton);
        eroded.copyTo(temp);

        if (cv::countNonZero(temp) == 0) {
            break;
        }

        iteration++;
        if (iteration > 100) {  // Safety limit
            std::cout << "[WARN] Skeleton extraction hit iteration limit" << std::endl;
            break;
        }
    }

    std::cout << "[DEBUG SKELETON] Extraction completed in " << iteration << " iterations" << std::endl;

    return skeleton;
}

// ============================================================================
// ENDPOINT DETECTION: Find skeleton endpoints (pixels with exactly 1 neighbor)
// ============================================================================

std::vector<cv::Point> LocatorCore::findSkeletonEndpoints(const cv::Mat &skeleton) {
    std::vector<cv::Point> endpoints;

    // Scan through skeleton, count 8-connected neighbors
    for (int y = 1; y < skeleton.rows - 1; y++) {
        for (int x = 1; x < skeleton.cols - 1; x++) {
            if (skeleton.at<uchar>(y, x) == 0) continue;

            // Count 8-connected neighbors
            int neighbors = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (skeleton.at<uchar>(y + dy, x + dx) > 0) {
                        neighbors++;
                    }
                }
            }

            // Endpoint has exactly 1 neighbor
            if (neighbors == 1) {
                endpoints.push_back(cv::Point(x, y));
            }
        }
    }

    std::cout << "[DEBUG SKELETON] Found " << endpoints.size() << " endpoints" << std::endl;

    return endpoints;
}

// ============================================================================
// SKELETON-BASED ENDPOINT DETECTION
// ============================================================================

std::optional<PipelineEndpoints> LocatorCore::findPipelineEndpoints(
    const cv::Mat &mask, bool use_skeleton_method, cv::Mat *debug_out) {

    if (mask.empty()) return std::nullopt;

    // If skeleton method disabled, fall back to bottom-most
    if (!use_skeleton_method) {
        std::cout << "[INFO] Skeleton method disabled, using bottom-most fallback" << std::endl;
        auto bottom_pixel = findStartPixelBottomMost(mask, debug_out);
        if (!bottom_pixel) return std::nullopt;

        PipelineEndpoints result;
        result.endpoint1 = *bottom_pixel;
        result.endpoint2 = *bottom_pixel;
        result.found_both = false;
        return result;
    }

    // 1. MORPHOLOGY: Clean mask
    cv::Mat clean_mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, clean_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(clean_mask, clean_mask, cv::MORPH_OPEN, kernel);

    // 2. CONNECTED COMPONENTS: Isolate largest component
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(clean_mask, labels, stats, centroids);

    if (numComponents <= 1) {
        std::cout << "[DEBUG SKELETON] No components found" << std::endl;
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
    std::cout << "[DEBUG CONVEXHULL] Largest component area: " << largestArea << " pixels" << std::endl;

    // 3. FURTHEST POINTS DETECTION using ConvexHull
    auto furthest = findFurthestPoints(pipeMask);

    if (!furthest) {
        std::cout << "[WARN] ConvexHull failed to find endpoints, falling back to bottom-most" << std::endl;
        auto bottom_pixel = findStartPixelBottomMost(mask, debug_out);
        if (!bottom_pixel) return std::nullopt;

        PipelineEndpoints result;
        result.endpoint1 = *bottom_pixel;
        result.endpoint2 = *bottom_pixel;
        result.found_both = false;
        return result;
    }

    // Success - have both endpoints from ConvexHull
    PipelineEndpoints result;
    result.endpoint1 = furthest->first;
    result.endpoint2 = furthest->second;
    result.found_both = true;

    std::cout << "[DEBUG CONVEXHULL] Found both endpoints: ("
              << result.endpoint1.x << "," << result.endpoint1.y << ") and ("
              << result.endpoint2.x << "," << result.endpoint2.y << ")" << std::endl;

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
// BACKWARD COMPATIBLE WRAPPER
// ============================================================================

std::optional<cv::Point> LocatorCore::findStartPixel(
    const cv::Mat &mask, cv::Mat *debug_out) {

    // Use skeleton method by default
    auto endpoints = findPipelineEndpoints(mask, true, debug_out);
    if (!endpoints) return std::nullopt;

    // Return first endpoint for backward compatibility
    return endpoints->endpoint1;
}

// ============================================================================
// LENS DISTORTION CORRECTION
// ============================================================================

cv::Point2f LocatorCore::undistortPoint(const cv::Point &pt,
                                        const CameraIntrinsics &intrinsics) {
    // If no distortion, return original point
    if (!intrinsics.has_distortion || intrinsics.D.empty()) {
        return cv::Point2f(pt.x, pt.y);
    }

    // Create intrinsics matrix K
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0, intrinsics.cx,
        0, intrinsics.fy, intrinsics.cy,
        0, 0, 1);

    // Create distortion vector
    cv::Mat D = cv::Mat(intrinsics.D);

    // Undistort using OpenCV
    std::vector<cv::Point2f> distorted_pts = {cv::Point2f(pt.x, pt.y)};
    std::vector<cv::Point2f> undistorted_pts;
    cv::undistortPoints(distorted_pts, undistorted_pts, K, D, cv::noArray(), K);

    std::cout << "[DEBUG UNDISTORT] (" << pt.x << "," << pt.y << ") -> ("
              << undistorted_pts[0].x << "," << undistorted_pts[0].y << ")" << std::endl;

    return undistorted_pts[0];
}

// ============================================================================
// 3D BACKPROJECTION: DVL + Flat Ground Plane
// ============================================================================

cv::Point3d LocatorCore::backprojectGroundPlane(
    int u, int v, double altitude, const CameraIntrinsics &intrinsics,
    bool apply_undistortion) {

    // Handle invalid altitude
    if (altitude <= 0.0 || std::isnan(altitude) || std::isinf(altitude)) {
        std::cout << "[WARN] Invalid altitude: " << altitude << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    // Undistort pixel if needed
    cv::Point2f pixel(u, v);
    if (apply_undistortion) {
        pixel = undistortPoint(cv::Point(u, v), intrinsics);
    }

    // Compute ray direction from camera through pixel
    // Normalized ray: (ray_x, ray_y, 1)
    double ray_x = (pixel.x - intrinsics.cx) / intrinsics.fx;
    double ray_y = (pixel.y - intrinsics.cy) / intrinsics.fy;
    double ray_z = 1.0;

    // Intersect ray with ground plane at Z = -altitude
    // Ray equation: P = t * (ray_x, ray_y, ray_z)
    // Ground plane: Z = -altitude
    // Solve for t: t * ray_z = -altitude => t = -altitude / ray_z
    double t = -altitude / ray_z;

    // 3D point on ground in camera frame
    double X = ray_x * t;
    double Y = ray_y * t;
    double Z = -altitude;

    std::cout << "[DEBUG BACKPROJECT] Pixel (" << u << "," << v
              << ") + altitude " << altitude << "m => 3D ("
              << X << "," << Y << "," << Z << ")" << std::endl;

    return cv::Point3d(X, Y, Z);
}

// ============================================================================
// LEGACY DEPTH-BASED BACKPROJECTION (kept for compatibility)
// ============================================================================

cv::Point3d LocatorCore::backproject(int u, int v, double z,
                                     const CameraIntrinsics &intrinsics) {
    // Handle invalid depth
    if (z <= 0.0 || std::isnan(z) || std::isinf(z)) {
        return cv::Point3d(0, 0, 0);
    }

    // Standard pinhole camera model
    double X = (u - intrinsics.cx) * z / intrinsics.fx;
    double Y = (v - intrinsics.cy) * z / intrinsics.fy;
    double Z = z;

    return cv::Point3d(X, Y, Z);
}

}  // namespace vortex_locate_pipeline
