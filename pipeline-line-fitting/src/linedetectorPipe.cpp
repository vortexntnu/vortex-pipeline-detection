#include <iostream>
#include <pipeline_line_fitting/linedetectorPipe.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std;

void LinedetectorPipe::_preprocess(cv::Mat &img, bool dist) {
    original_width = img.cols;
    original_height = img.rows;

    // Calculate scaling factors
    scale_x = static_cast<double>(size) / original_width;
    scale_y = static_cast<double>(size) / original_height;

    // Resize image to fixed size
    cv::resize(img, img, cv::Size(size, size));

    if (dist) {
        cv::distanceTransform(img, img, cv::DIST_L2, 5);
    }

    // Apply morphological opening
    cv::Mat kernel = cv::Mat::ones(1, 1, CV_8U);
    cv::morphologyEx(img, img, cv::MORPH_OPEN, kernel);
    img.convertTo(img, CV_8U);
}

LinedetectorPipe::~LinedetectorPipe() {
    // Destructor
}

void LinedetectorPipe::_postprocess() {
    // Postprocess
}

int LinedetectorPipe::detectSingleLine(const mat &points, const mat &values, const vector<Line> &lines, const int i) {
    // Extract columns and reshape
    mat X = points.col(1);
    X.reshape(points.n_rows, 1);
    mat y = points.col(0);
    y.reshape(points.n_rows, 1);

    // Set the d parameter for RANSAC
    int d = points.n_elem * fracOfPoints;
    randsac.d = d;

    // Fit the RANSAC model
    randsac.fit(X, y, values, lines);

    // Print the best value
    cout << "Found line " << i + 1 << " with score: " << randsac.bestScore << endl;

    // Check the best_fit and bestValue conditions
    if (randsac.bestFit.params.size() == 0 || randsac.bestScore < finalScorethresh) {
        return 1;
    }
    return 0;
}

void LinedetectorPipe::_getEndPoints(Line &line, const mat &points) {
    int min_x = -1;
    int max_x = -1;
    int min_x_yval;
    int max_x_yval;

    for (int x = 0; x < size; ++x) {
        int y = line.slope * x + line.intercept;

        if (y < 0 || y >= size) {
            continue;
        }
        int pixel = processedImg.at<uchar>(y, x);
        if (pixel > 0) {
            if (min_x == -1) {
                min_x = x;
                min_x_yval = y;
            }
            if (x > max_x) {
                max_x = x;
                max_x_yval = y;
            }
        }
    }

    // Apply scaling back to original coordinates
    line.start = cv::Point(static_cast<int>(min_x / scale_x), static_cast<int>(min_x_yval / scale_y));
    line.end = cv::Point(static_cast<int>(max_x / scale_x), static_cast<int>(max_x_yval / scale_y));
}

vector<Line> LinedetectorPipe::operator()(const cv::Mat &img, const int maxLines = 2) {
    processedImg = img.clone();
    _preprocess(processedImg);

    // Find points where img > 0
    vector<cv::Point> pointList;
    cv::findNonZero(processedImg, pointList);

    // Convert points to arma::mat
    mat points(pointList.size(), 2);
    for (size_t i = 0; i < pointList.size(); ++i) {
        points(i, 0) = pointList[i].y;
        points(i, 1) = pointList[i].x;
    }

    // Extract values from the image at the points
    mat values(pointList.size(), 1);
    for (size_t i = 0; i < pointList.size(); ++i) {
        values(i, 0) = processedImg.at<uchar>(pointList[i].y, pointList[i].x);
    }

    vector<Line> lines;

    for (int i = 0; i < maxLines; ++i) {
        int returnCode = detectSingleLine(points, values, lines, i);

        if (returnCode) {
            cout << "RANSAC failed to find line number " << i + 1 << endl;
            continue;
        }

        Line line{randsac.bestFit.params[1], randsac.bestFit.params[0], randsac.bestScore, {0, 0}, {0, 0}};
        _getEndPoints(line, points);

        // Remove points for next iteration
        mat newPoints;
        mat newValues;
        for (size_t j = 0; j < points.n_rows; ++j) {
            if (find(randsac.rempointids.begin(), randsac.rempointids.end(), j) == randsac.rempointids.end()) {
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

void LinedetectorPipe::drawResults(const cv::Mat &img, const vector<Line> &lines, string saveDest) {
    // Draw the lines on the original size image
    cv::Mat img2 = img.clone();
    cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < lines.size(); i += 1) {
        // Scale line endpoints back to original image coordinates if necessary
        cv::line(img2, lines[i].start, lines[i].end, cv::Scalar(255, 0, 255), 2);
    }

    cv::imwrite(saveDest, img2);
}
