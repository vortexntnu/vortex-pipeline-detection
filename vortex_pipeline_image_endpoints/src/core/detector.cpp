#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

std::optional<PipelineEndpoints> PipelineDetector::find_pipeline_endpoints(const cv::Mat& mask,
                                                                           DetectionMethod method,
                                                                           int kernel_size,
                                                                           cv::Mat* debug_out) {
    if (mask.empty()) return std::nullopt;

    auto pipe_mask = largest_component(clean_mask(mask, kernel_size));
    if (!pipe_mask) return std::nullopt;

    PipelineEndpoints result;

    switch (method) {
        case DetectionMethod::FURTHEST_POINTS: {
            auto furthest = find_furthest_points(*pipe_mask);
            if (!furthest) return std::nullopt;
            result.endpoint1 = furthest->first;
            result.endpoint2 = furthest->second;
            break;
        }
        case DetectionMethod::LOWEST_PIXEL: {
            auto lowest = find_lowest_pixel(*pipe_mask);
            if (!lowest) return std::nullopt;
            result.endpoint1 = *lowest;
            result.endpoint2 = find_highest_pixel(*pipe_mask);
            break;
        }
    }

    if (debug_out != nullptr) {
        draw_endpoints(*debug_out, *pipe_mask, result);
    }

    return result;
}

}  // namespace vortex_pipeline_image_endpoints
