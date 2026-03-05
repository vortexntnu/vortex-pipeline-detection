#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

std::optional<PipelineEndpoints> PipelineDetector::findPipelineEndpoints(
    const cv::Mat &mask, DetectionMethod method, cv::Mat *debug_out) {

    if (mask.empty()) return std::nullopt;

    auto pipe_mask = largestComponent(cleanMask(mask));
    if (!pipe_mask) return std::nullopt;

    PipelineEndpoints result;

    switch (method) {
        case DetectionMethod::FURTHEST_POINTS: {
            auto furthest = findFurthestPoints(*pipe_mask);
            if (!furthest) return std::nullopt;
            result.endpoint1 = furthest->first;
            result.endpoint2 = furthest->second;
            break;
        }
        case DetectionMethod::LOWEST_PIXEL: {
            auto lowest = findLowestPixel(*pipe_mask);
            if (!lowest) return std::nullopt;
            result.endpoint1 = *lowest;
            break;
        }
    }

    if (debug_out != nullptr) {
        drawEndpoints(*debug_out, *pipe_mask, result);
    }

    return result;
}

}  // namespace vortex_pipeline_image_endpoints
