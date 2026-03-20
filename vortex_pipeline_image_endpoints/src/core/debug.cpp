#include "vortex_pipeline_image_endpoints/detector.hpp"

namespace vortex_pipeline_image_endpoints {

void PipelineDetector::draw_endpoints(cv::Mat& debug_out,
                                      const cv::Mat& pipe_mask,
                                      const PipelineEndpoints& endpoints) {
    cv::cvtColor(pipe_mask, debug_out, cv::COLOR_GRAY2BGR);

    if (endpoints.endpoint2.has_value()) {
        cv::line(debug_out, endpoints.endpoint1, *endpoints.endpoint2, cv::Scalar(255, 255, 0),
                 2);  // cyan axis line
    }

    // First endpoint in green
    const std::string label1 = endpoints.endpoint2.has_value() ? "EP1" : "START";
    cv::circle(debug_out, endpoints.endpoint1, 8, cv::Scalar(0, 255, 0), -1);
    cv::putText(debug_out, label1, cv::Point(endpoints.endpoint1.x + 12, endpoints.endpoint1.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

    // Second endpoint in magenta if present
    if (endpoints.endpoint2.has_value()) {
        cv::circle(debug_out, *endpoints.endpoint2, 8, cv::Scalar(255, 0, 255), -1);
        cv::putText(debug_out, "EP2",
                    cv::Point(endpoints.endpoint2->x + 12, endpoints.endpoint2->y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
    }
}

}  // namespace vortex_pipeline_image_endpoints
