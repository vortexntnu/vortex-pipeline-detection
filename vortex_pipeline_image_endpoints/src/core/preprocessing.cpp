#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

cv::Mat PipelineDetector::clean_mask(const cv::Mat& mask, int kernel_size) {
    cv::Mat result;
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(mask, result, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    return result;
}

std::optional<cv::Mat> PipelineDetector::largest_component(const cv::Mat& mask) {
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

}  // namespace vortex_pipeline_image_endpoints
