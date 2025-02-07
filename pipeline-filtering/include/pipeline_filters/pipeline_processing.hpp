#ifndef PIPELINE_PROCESSING_HPP
#define PIPELINE_PROCESSING_HPP

#include <iostream>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xphoto.hpp>

namespace vortex::pipeline_processing {

struct OtsuParams {
  bool gamma_auto_correction;
  double gamma_auto_correction_weight;
  bool otsu_segmentation;
  double gsc_weight_r;
  double gsc_weight_g;
  double gsc_weight_b;
};

struct FilterParams {
  OtsuParams otsu;
};

typedef void (*FilterFunction)(const FilterParams &, const cv::Mat &,
                               cv::Mat &);

/**
 * Reads the filter_type from the FilterParams struct
 * and calls the appropriate filter function from the filter_functions map.
 */
void apply_filter(const std::string &filter, const FilterParams &params,
                  const cv::Mat &original, cv::Mat &output);

/**
 * No filter, just copy the image
 */
void no_filter(const FilterParams &params, const cv::Mat &original,
               cv::Mat &output);

/**
 * A filter based on Otsu's method
 */
void otsu_segmentation_filter(const FilterParams &params,
                              const cv::Mat &original, cv::Mat &output);

const static std::map<std::string, FilterFunction> filter_functions = {
    {"no_filter", no_filter}, {"otsu", otsu_segmentation_filter}};

} // namespace vortex::pipeline_processing
#endif // PIPELINE_PROCESSING_HPP
