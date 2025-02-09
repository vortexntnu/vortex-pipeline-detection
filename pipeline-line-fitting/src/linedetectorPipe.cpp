#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <pipeline_line_fitting/linedetectorPipe.hpp>

void removeBorderArtifacts(cv::Mat& img) {
    img.row(0).setTo(cv::Scalar(0));
    img.row(img.rows - 1).setTo(cv::Scalar(0));
    img.col(0).setTo(cv::Scalar(0));
    img.col(img.cols - 1).setTo(cv::Scalar(0));
}

void LinedetectorPipe::preprocess(cv::Mat& img, bool dist) {
    // Calculate scaling factors
    scaleX_ = static_cast<double>(size_) / img.cols;
    scaleY_ = static_cast<double>(size_) / img.rows;

    cv::resize(img, img, cv::Size(size_, size_));

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

    // Apply distance transform to get the center lines
    if (dist) {
        cv::Mat dist_img;
        cv::distanceTransform(img, dist_img, cv::DIST_L2, 5);
        cv::normalize(dist_img, dist_img, 0, 1.0, cv::NORM_MINMAX);

        // Threshold the distance transform image to get the skeleton
        cv::threshold(dist_img, img, 0.2, 1.0, cv::THRESH_BINARY);
    }

    // Convert the image to 8-bit
    img.convertTo(img, CV_8U, 255);

    // Apply morphological operations to clean up the result
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,
    // 3));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

    // Skeletonize the image using Zhang-Suen thinning algorithm
    cv::ximgproc::thinning(img, img, cv::ximgproc::THINNING_ZHANGSUEN);

    removeBorderArtifacts(img);
}

LinedetectorPipe::LinedetectorPipe() {};
LinedetectorPipe::~LinedetectorPipe() {};

void LinedetectorPipe::postprocess() {}

int LinedetectorPipe::detectSingleLine(const arma::mat& points,
                                       const arma::mat& values,
                                       const std::vector<Line>& lines,
                                       const int i) {
    // Extract columns and reshape
    if (points.n_rows < 5) {
        return 1;
    }

    arma::mat X = points.col(1);
    X.reshape(points.n_rows, 1);
    arma::mat y = points.col(0);
    y.reshape(points.n_rows, 1);

    // Set the d parameter for RANSAC
    int d = points.n_elem * fracOfPoints_;
    randsac_.d = d;

    // Fit the RANSAC model
    randsac_.fit(X, y, values, lines);

    // Check the best_fit and bestValue conditions
    if (randsac_.bestFit.params.size() == 0 ||
        randsac_.bestScore < finalScorethresh_) {
        return 1;
    }
    std::cout << "Found line " << i + 1 << " with score: " << randsac_.bestScore
              << std::endl;

    return 0;
}

void LinedetectorPipe::getEndPoints(Line& line, bool swap) {
    int minX = -1;
    int maxX = -1;
    int minX_yval = -1;
    int maxX_yval = -1;
    for (double x = 0; x < size_; x += 0.05) {
        int y = line.slope * x + line.intercept;

        if (y < 0 || y >= size_) {
            continue;
        }
        int pixel;
        if (swap) {
            pixel = orgImg_.at<uchar>(x, y);
        } else {
            pixel = orgImg_.at<uchar>(y, x);
        }
        if (pixel > 0) {
            if (minX == -1) {
                minX = x;
                minX_yval = y;
            }
            if (x > maxX) {
                maxX = x;
                maxX_yval = y;
            }
        }
    }

    if (swap) {
        line.start = cv::Point(static_cast<int>(minX_yval / scaleX_),
                               static_cast<int>(minX / scaleY_));
        line.end = cv::Point(static_cast<int>(maxX_yval / scaleX_),
                             static_cast<int>(maxX / scaleY_));
    } else {
        line.start = cv::Point(static_cast<int>(minX / scaleX_),
                               static_cast<int>(minX_yval / scaleY_));
        line.end = cv::Point(static_cast<int>(maxX / scaleX_),
                             static_cast<int>(maxX_yval / scaleY_));
    }
}

std::vector<Line> LinedetectorPipe::operator()(const cv::Mat& img,
                                               const int maxLines = 3) {
    orgImg_ = img.clone();
    cv::resize(orgImg_, orgImg_, cv::Size(size_, size_));
    processedImg_ = img.clone();
    preprocess(processedImg_);

    // Find points where img > 0
    std::vector<cv::Point> pointList;
    cv::findNonZero(processedImg_, pointList);

    // Convert points to arma::mat
    arma::mat points(pointList.size(), 2);
    for (size_t i = 0; i < pointList.size(); ++i) {
        points(i, 0) = pointList[i].y;
        points(i, 1) = pointList[i].x;
    }

    // Extract values from the image at the points
    arma::mat values(pointList.size(), 1);
    for (size_t i = 0; i < pointList.size(); ++i) {
        values(i, 0) = processedImg_.at<uchar>(pointList[i].y, pointList[i].x);
    }

    std::vector<Line> lines;

    for (int i = 0; i < maxLines; ++i) {
        int returnCode = detectSingleLine(points, values, lines, i);
        Line line;

        if (returnCode) {
            std::cout << "RANSAC failed to find line number " << i + 1
                      << std::endl;
            // rotate points and retry
            arma::mat newPoints(points.n_cols, points.n_rows);
            for (size_t j = 0; j < points.n_rows; ++j) {
                newPoints(0, j) = points(j, 1);
                newPoints(1, j) = points(j, 0);
            }

            newPoints = newPoints.t();

            returnCode = detectSingleLine(newPoints, values, lines, i);

            if (returnCode) {
                std::cout << "RANSAC failed to find line number " << i + 1
                          << " again" << std::endl;
                continue;
            }

            line = Line{randsac_.bestFit.params[1], randsac_.bestFit.params[0],
                        randsac_.bestScore};
            // use a rotated image to get end points also
            getEndPoints(line, true);

        } else {
            line = Line{randsac_.bestFit.params[1], randsac_.bestFit.params[0],
                        randsac_.bestScore};
            getEndPoints(line);
        }

        // Remove points for next iteration
        arma::mat newPoints;
        arma::mat newValues;
        for (size_t j = 0; j < points.n_rows; ++j) {
            if (std::find(randsac_.rempointids.begin(),
                          randsac_.rempointids.end(),
                          j) == randsac_.rempointids.end()) {
                newPoints.insert_rows(newPoints.n_rows, points.row(j));
                newValues.insert_rows(newValues.n_rows, values.row(j));
            }
        }
        points = newPoints;
        values = newValues;

        lines.push_back(line);
    }

    return lines;
}
