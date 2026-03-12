#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace vortex_pipeline_image_endpoints {

enum class DetectionMethod { FURTHEST_POINTS, LOWEST_PIXEL };

// Result of endpoint detection
struct PipelineEndpoints {
    cv::Point endpoint1;                 // First (or only) detected endpoint (pixel coords)
    std::optional<cv::Point> endpoint2;  // Second endpoint; nullopt for single-endpoint methods
};

class PipelineDetector {
   public:
    // Find pipeline endpoints using the specified detection method
    // Returns struct with endpoint(s) if found
    // debug_out: optional debug visualization (nullptr = no debug)
    static std::optional<PipelineEndpoints> findPipelineEndpoints(const cv::Mat& mask,
                                                                  DetectionMethod method,
                                                                  cv::Mat* debug_out = nullptr);

   private:
    // --- Preprocessing ---
    // Apply morphological close+open to remove noise and fill small holes
    static cv::Mat cleanMask(const cv::Mat& mask);
    // Return a binary mask of the largest connected foreground component
    static std::optional<cv::Mat> largestComponent(const cv::Mat& mask);
    // --- Detection methods ---
    // Find two points furthest apart using convex hull
    static std::optional<std::pair<cv::Point, cv::Point>> findFurthestPoints(
        const cv::Mat& binary);
    // Find the centroid of the lowest (highest y-index) foreground row
    static std::optional<cv::Point> findLowestPixel(const cv::Mat& binary);

    // --- Debug ---
    // Draw detected endpoints onto debug_out, initialised from pipe_mask
    static void drawEndpoints(cv::Mat& debug_out,
                              const cv::Mat& pipe_mask,
                              const PipelineEndpoints& endpoints);
};

}  // namespace vortex_pipeline_image_endpoints
