#include <stdio.h>
#include <cmath>
#include <iostream>
#include <numeric>
#include <pipeline_filters/pipeline_processing.hpp>
#include <vector>

namespace vortex::pipeline_processing {

void no_filter([[maybe_unused]] const FilterParams& params,
               const cv::Mat& original,
               cv::Mat& filtered) {
    original.copyTo(filtered);
}

// Define the applyGammaCorrection function
void applyGammaCorrection(cv::Mat& image, double gamma) {  // Pass by reference
    // Create a lookup table for gamma correction
    cv::Mat lookup(1, 256, CV_8U);
    uchar* p = lookup.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    // Apply the gamma correction using the lookup table
    cv::LUT(image, lookup, image);
}

double calculateAutoGamma(const cv::Mat& image) {
    // Convert the image to grayscale if it's not already
    cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image;
    }

    // Calculate the mean intensity
    cv::Scalar meanIntensity = mean(grayImage);

    // The ideal mean intensity is 128 (midpoint for 8-bit grayscale images)
    double idealMean = 128.0;
    double currentMean = meanIntensity[0];

    // Automatically set gamma value based on the mean intensity
    double gamma;
    if (currentMean > 0) {
        gamma = log(idealMean / 255.0) / log(currentMean / 255.0);
    } else {
        gamma = 1.0;  // Default gamma if the image has no intensity
    }

    // Ensure gamma is within a reasonable range (e.g., between 0.1 and 3.0)
    gamma = std::max(0.1, std::min(gamma, 3.0));

    return gamma;
}

void otsu_segmentation_filter(const FilterParams& params,
                              const cv::Mat& original,
                              cv::Mat& filtered) {
    bool gamma_auto_correction = params.otsu.gamma_auto_correction;
    double gamma_auto_correction_weight =
        params.otsu.gamma_auto_correction_weight;

    bool otsu_segmentation = params.otsu.otsu_segmentation;

    cv::cvtColor(original, filtered, cv::COLOR_BGR2GRAY);

    if (gamma_auto_correction) {
        double autoGamma =
            calculateAutoGamma(filtered) * gamma_auto_correction_weight;

        // Print out the auto-calculated gamma for verification
        std::cout << "Automatically calculated gamma: " << autoGamma
                  << std::endl;

        applyGammaCorrection(filtered, autoGamma);
    }

    if (otsu_segmentation) {
        // Calculate the histogram
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::Mat hist;
        calcHist(&filtered, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange,
                 true, false);

        // Normalize histogram to get probabilities
        hist /= filtered.total();

        // Initialize variables for Otsu's method
        std::vector<float> sigma2_list(256, 0.0);
        std::vector<float> p(hist.begin<float>(),
                             hist.end<float>());  // Probabilities

        for (int th = 1; th < 256; ++th) {
            // Calculate omega (weights) for foreground and background
            float omega_fg = std::accumulate(p.begin(), p.begin() + th, 0.0f);
            float omega_bg = std::accumulate(p.begin() + th, p.end(), 0.0f);

            // Calculate mu (means) for foreground and background
            float mu_fg = 0, mu_bg = 0;
            for (int i = 0; i < th; ++i) {
                mu_fg += i * p[i];
            }
            for (int i = th; i < 256; ++i) {
                mu_bg += i * p[i];
            }

            if (omega_fg > 0)
                mu_fg /= omega_fg;
            if (omega_bg > 0)
                mu_bg /= omega_bg;

            // Calculate sigma squared and store in list
            sigma2_list[th] = omega_fg * omega_bg * pow(mu_fg - mu_bg, 2);
        }

        // Find the threshold corresponding to the maximum sigma squared
        int optimalThreshold =
            std::max_element(sigma2_list.begin(), sigma2_list.end()) -
            sigma2_list.begin();
        std::cout << "Automatically calculated threshold: " << optimalThreshold
                  << std::endl;
        // Apply the threshold to the image
        cv::Mat binaryImage;
        cv::threshold(filtered, binaryImage, optimalThreshold, 255,
                      cv::THRESH_BINARY);

        // Apply erosion followed by dilation (opening)
        cv::Mat openedImage;
        int erosionSize = 10;
        cv::Mat erosionKernel = getStructuringElement(
            cv::MORPH_CROSS, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1),
            cv::Point(erosionSize, erosionSize));
        cv::erode(binaryImage, openedImage, erosionKernel);

        int dilutionSize = 45;
        cv::Mat dilutionKernel = getStructuringElement(
            cv::MORPH_CROSS,
            cv::Size(2 * dilutionSize + 1, 2 * dilutionSize + 1),
            cv::Point(dilutionSize, dilutionSize));
        cv::dilate(openedImage, openedImage, dilutionKernel);

        int erosionSize2 = 35;
        cv::Mat erosionKernel2 = getStructuringElement(
            cv::MORPH_CROSS,
            cv::Size(2 * erosionSize2 + 1, 2 * erosionSize2 + 1),
            cv::Point(erosionSize2, erosionSize2));
        cv::erode(openedImage, openedImage, erosionKernel2);

        dilutionSize = 10;
        dilutionKernel = getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2 * dilutionSize + 1, 2 * dilutionSize + 1),
            cv::Point(dilutionSize, dilutionSize));
        cv::dilate(openedImage, openedImage, dilutionKernel);

        erosionSize2 = 10;
        erosionKernel2 = getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(2 * erosionSize2 + 1, 2 * erosionSize2 + 1),
            cv::Point(erosionSize2, erosionSize2));
        cv::erode(openedImage, openedImage, erosionKernel2);

        filtered = openedImage;
    }
}

void apply_filter(const std::string& filter,
                  const FilterParams& params,
                  const cv::Mat& original,
                  cv::Mat& filtered) {
    if (filter_functions.contains(filter)) {
        ((filter_functions.at(filter)))(params, original, filtered);
    }
}

}  // namespace vortex::pipeline_processing
