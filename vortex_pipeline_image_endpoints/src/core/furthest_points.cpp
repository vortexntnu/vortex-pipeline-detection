#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

std::optional<std::pair<cv::Point, cv::Point>> PipelineDetector::find_furthest_points(
    const cv::Mat& binary) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return std::nullopt;

    auto largest_it =
        std::max_element(contours.begin(), contours.end(),
                         [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                             return cv::contourArea(a) < cv::contourArea(b);
                         });

    std::vector<cv::Point> hull;
    cv::convexHull(*largest_it, hull);

    if (hull.size() < 2) return std::nullopt;

    // Find two hull points with maximum pairwise distance
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

    return std::make_pair(pt1, pt2);
}

}  // namespace vortex_pipeline_image_endpoints
