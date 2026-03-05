#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

cv::Mat PipelineDetector::cleanMask(const cv::Mat &mask) {
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, result, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    return result;
}

std::optional<cv::Mat> PipelineDetector::largestComponent(const cv::Mat &mask) {
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    if (numComponents == 1) return std::nullopt;  // only background, no foreground

    int largestIdx = 1;
    int largestArea = stats.at<int>(1, cv::CC_STAT_AREA);
    for (int i = 2; i < numComponents; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > largestArea) {
            largestArea = area;
            largestIdx = i;
        }
    }
    return (labels == largestIdx);
}

cv::Point2f PipelineDetector::undistortPoint(const cv::Point &pixel,
                                             const CameraIntrinsics &intrinsics) {
    if (!intrinsics.has_distortion || intrinsics.D.empty()) {
        return cv::Point2f(pixel.x, pixel.y);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0, intrinsics.cx,
        0, intrinsics.fy, intrinsics.cy,
        0, 0, 1);

    cv::Mat D = cv::Mat(intrinsics.D);

    std::vector<cv::Point2f> distorted_pts = {cv::Point2f(pixel.x, pixel.y)};
    std::vector<cv::Point2f> undistorted_pts;
    cv::undistortPoints(distorted_pts, undistorted_pts, K, D, cv::noArray(), K);

    return undistorted_pts[0];
}

}  // namespace vortex_pipeline_image_endpoints
