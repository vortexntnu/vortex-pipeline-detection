#include <pipeline_filters/pipeline_processing.hpp>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <cmath>

namespace vortex::pipeline_processing
{

void no_filter([[maybe_unused]] const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
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

void customGrayscale(const cv::Mat& original, cv::Mat& filtered, double weightR, double weightG, double weightB) {
    // Make the sum of weights equal 1
    double weightSum = weightR + weightG + weightB;
    weightR /= weightSum;
    weightG /= weightSum;
    weightB /= weightSum;

    // Create an empty single-channel image
    filtered = cv::Mat::zeros(original.size(), CV_8UC1);

    // Iterate through each pixel
    for (int row = 0; row < original.rows; ++row) {
        for (int col = 0; col < original.cols; ++col) {
            // Get the BGR values
            cv::Vec3b bgr = original.at<cv::Vec3b>(row, col);
            uchar blue = bgr[0];
            uchar green = bgr[1];
            uchar red = bgr[2];

            // Calculate the grayscale value with custom weights
            uchar gray = static_cast<uchar>(weightB*blue + weightG*green + weightR*red);

            // Assign to the output image
            filtered.at<uchar>(row, col) = gray;
        }
    }
}

void customTransformation(const cv::Mat& input, cv::Mat& output, double alpha, double n) {
    input.convertTo(output, CV_32F, 1.0 / 255.0); // Normalize to [0, 1]
    cv::pow(output, n, output);                  // Apply power transformation
    output *= alpha;                             // Scale the brightness
    output.convertTo(output, CV_8U, 255.0);      // Convert back to [0, 255]
}

void otsu_segmentation_filter(const FilterParams& params, const cv::Mat &original, cv::Mat &filtered) 
{
	bool gamma_auto_correction = params.otsu.gamma_auto_correction;
	double gamma_auto_correction_weight = params.otsu.gamma_auto_correction_weight;
	
	bool otsu_segmentation = params.otsu.otsu_segmentation;
	
	// Custom weights for red, green, and blue channels
    double weightR = params.otsu.gsc_weight_r;
	double weightG = params.otsu.gsc_weight_g;
	double weightB = params.otsu.gsc_weight_b;

    // Convert to grayscale with custom weights
    customGrayscale(original, filtered, weightR, weightG, weightB);

	cv::Mat output;
	double alpha = 2.0; // Adjust for brightness normalization
    double n = 5.0;     // Power for darkening
    customTransformation(filtered, output, alpha, n);

	filtered = output;

	if(gamma_auto_correction) {
		double autoGamma = calculateAutoGamma(filtered) * gamma_auto_correction_weight;

		// Print out the auto-calculated gamma for verification
		//std::cout << "Automatically calculated gamma: " << autoGamma << std::endl;

		applyGammaCorrection(filtered, autoGamma);
	}

	if(otsu_segmentation) {
		// Calculate the histogram
		int histSize = 256;
		float range[] = {0, 256};
		const float* histRange = {range};
		cv::Mat hist;
		calcHist(&filtered, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);

		// Normalize histogram to get probabilities
		hist /= filtered.total();

		// Initialize variables for Otsu's method
		std::vector<float> sigma2_list(256, 0.0);
		std::vector<float> p(hist.begin<float>(), hist.end<float>()); // Probabilities

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

			if (omega_fg > 0) mu_fg /= omega_fg;
			if (omega_bg > 0) mu_bg /= omega_bg;

			// Calculate sigma squared and store in list
			sigma2_list[th] = omega_fg * omega_bg * pow(mu_fg - mu_bg, 2);
		}

		// Find the threshold corresponding to the maximum sigma squared
		int optimalThreshold = std::max_element(sigma2_list.begin(), sigma2_list.end()) - sigma2_list.begin();
		//std::cout << "Automatically calculated threshold: " << optimalThreshold << std::endl;
		// Apply the threshold to the image
		cv::Mat binaryImage;
		cv::threshold(filtered, binaryImage, optimalThreshold, 255, cv::THRESH_BINARY);

		// Apply erosion followed by dilation (opening)
		cv::Mat openedImage;
		int erosionSize = 10;
		cv::Mat erosionKernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize, erosionSize));
		cv::erode(binaryImage, openedImage, erosionKernel);

		int dilutionSize = 45;
		cv::Mat dilutionKernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * dilutionSize + 1, 2 * dilutionSize + 1), cv::Point(dilutionSize, dilutionSize));
		cv::dilate(openedImage, openedImage, dilutionKernel);

		int erosionSize2 = 35;
		cv::Mat erosionKernel2 = getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * erosionSize2 + 1, 2 * erosionSize2 + 1), cv::Point(erosionSize2, erosionSize2));
		cv::erode(openedImage, openedImage, erosionKernel2);

		dilutionSize = 10;
		dilutionKernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * dilutionSize + 1, 2 * dilutionSize + 1), cv::Point(dilutionSize, dilutionSize));
		cv::dilate(openedImage, openedImage, dilutionKernel);

		erosionSize2 = 10;
		erosionKernel2 = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize2 + 1, 2 * erosionSize2 + 1), cv::Point(erosionSize2, erosionSize2));
		cv::erode(openedImage, openedImage, erosionKernel2);

		filtered = openedImage;
	}
}


void apply_filter(const std::string& filter, const FilterParams& params, const cv::Mat &original, cv::Mat &filtered)
{
	if(filter_functions.contains(filter)){
		((filter_functions.at(filter)))(params, original, filtered);
    }
}

} // namespace vortex::pipeline_processing