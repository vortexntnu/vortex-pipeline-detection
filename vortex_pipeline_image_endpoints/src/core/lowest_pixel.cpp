// Implementation of the lowest-pixel endpoint detection method.
// Finds the centroid of foreground pixels in the lowest (highest y-index) row.

#include <numeric>
#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

std::optional<cv::Point> PipelineDetector::find_lowest_pixel(const cv::Mat& binary) {
    for (int row = binary.rows - 1; row >= 0; --row) {
        std::vector<int> cols;
        const uchar* ptr = binary.ptr<uchar>(row);
        for (int col = 0; col < binary.cols; ++col) {
            if (ptr[col] > 0) cols.push_back(col);
        }
        if (!cols.empty()) {
            int avg_x = static_cast<int>(std::accumulate(cols.begin(), cols.end(), 0) /
                                         static_cast<int>(cols.size()));
            return cv::Point(avg_x, row);
        }
    }
    return std::nullopt;
}

std::optional<cv::Point> PipelineDetector::find_highest_pixel(const cv::Mat& binary) {
    for (int row = 0; row < binary.rows; ++row) {
        std::vector<int> cols;
        const uchar* ptr = binary.ptr<uchar>(row);
        for (int col = 0; col < binary.cols; ++col) {
            if (ptr[col] > 0) cols.push_back(col);
        }
        if (!cols.empty()) {
            int avg_x = static_cast<int>(std::accumulate(cols.begin(), cols.end(), 0) /
                                         static_cast<int>(cols.size()));
            return cv::Point(avg_x, row);
        }
    }
    return std::nullopt;
}

}  // namespace vortex_pipeline_image_endpoints
