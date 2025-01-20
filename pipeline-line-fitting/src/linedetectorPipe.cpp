#include <iostream>
#include <pipeline_line_fitting/linedetectorPipe.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

using namespace std;


void thinningIteration(cv::Mat& img, int iter) {
    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    for (int i = 1; i < img.rows - 1; i++) {
        for (int j = 1; j < img.cols - 1; j++) {
            uchar p2 = img.at<uchar>(i - 1, j);
            uchar p3 = img.at<uchar>(i - 1, j + 1);
            uchar p4 = img.at<uchar>(i, j + 1);
            uchar p5 = img.at<uchar>(i + 1, j + 1);
            uchar p6 = img.at<uchar>(i + 1, j);
            uchar p7 = img.at<uchar>(i + 1, j - 1);
            uchar p8 = img.at<uchar>(i, j - 1);
            uchar p9 = img.at<uchar>(i - 1, j - 1);

            int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
                    (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
                    (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                    (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0) {
                marker.at<uchar>(i, j) = 1;
            }
        }
    }

    img &= ~marker;
}

void thinning(cv::Mat& img) {
    img /= 255;

    cv::Mat prev = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(img, 0);
        thinningIteration(img, 1);
        cv::absdiff(img, prev, diff);
        img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);

    img *= 255;
}

void removeBorderArtifacts(cv::Mat& img) {
    img.row(0).setTo(cv::Scalar(0));
    img.row(img.rows - 1).setTo(cv::Scalar(0));
    img.col(0).setTo(cv::Scalar(0));
    img.col(img.cols - 1).setTo(cv::Scalar(0));
}

void LinedetectorPipe::_preprocess(cv::Mat& img, bool dist) {
    // Calculate scaling factors
    double scale_x = static_cast<double>(size) / img.cols;
    double scale_y = static_cast<double>(size) / img.rows;

    cv::resize(img, img, cv::Size(size, size));

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
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

    // Skeletonize the image using Zhang-Suen thinning algorithm
    thinning(img);

    removeBorderArtifacts(img);
}

LinedetectorPipe::~LinedetectorPipe(){
    // Destructor
}

void LinedetectorPipe::_postprocess() {
    // Postprocess
}

int LinedetectorPipe::detectSingleLine(const mat &points, const mat &values, const vector<Line> &lines, const int i) {
    // Extract columns and reshape
    if (points.n_rows < 5) {
        return 1;
    }

    mat X = points.col(1);
    X.reshape(points.n_rows, 1);
    mat y = points.col(0);
    y.reshape(points.n_rows, 1);

    // Set the d parameter for RANSAC
    int d = points.n_elem * fracOfPoints;
    randsac.d = d;

    // Fit the RANSAC model
    randsac.fit(X, y, values, lines);

    // Check the best_fit and bestValue conditions
    if (randsac.bestFit.params.size() == 0 || randsac.bestScore < finalScorethresh) {
        return 1;
    }
    cout << "Found line " << i+1 << " with score: " << randsac.bestScore << endl;

    return 0;
}

void LinedetectorPipe::_getEndPoints(Line &line, bool swap) {
    int min_x = -1;
    int max_x = -1;
    int min_x_yval;
    int max_x_yval;
    cout << line.slope << " " << line.intercept << endl;
    for (int x = 0; x < size; ++x) {
        int y = line.slope * x + line.intercept;

        if (y < 0 || y >= size) {
            continue;
        }
        int pixel;
        if (swap) {
            pixel = orgImg.at<uchar>(x, y);
        }
        else{
            pixel = orgImg.at<uchar>(y, x);
        }
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
        else if (min_x != -1 && false){
            //line has started, but ended. //thus, we find the end point more accuratly and break

            //find the end point more accurately
            int y1 = line.slope * (x-1) + line.intercept;
            int y2 = line.slope * (x) + line.intercept;
            for (int y = y1; y < y2; y++){
                if (swap) {
                    pixel = orgImg.at<uchar>(x, y);
                }
                else{
                    pixel = orgImg.at<uchar>(y, x);
                }
                if (pixel > 0) {
                    max_x = x;
                    max_x_yval = y;
                    break;
                }
            }
            break;
        }
    }


    // Apply scaling back to original coordinates - nooooooooooooooooooooooooooooooooooo you dont
    //not yet, anyway

    /*line.start = cv::Point(static_cast<int>(min_x / scale_x), static_cast<int>(min_x_yval / scale_y));
    line.end = cv::Point(static_cast<int>(max_x / scale_x), static_cast<int>(max_x_yval / scale_y));*/
    line.start = cv::Point(min_x, min_x_yval);
    line.end = cv::Point(max_x, max_x_yval);
}

vector<Line> LinedetectorPipe::operator()(const cv::Mat &img, const int maxLines=3){
    orgImg = img.clone();
    cv::resize(orgImg, orgImg, cv::Size(size, size));
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

    // function to rotate points 45 degrees
    mat rotate45 = {{0.7071, -0.7071}, {0.7071, 0.7071}};

    vector<Line> lines;

    for (int i = 0; i < maxLines; ++i) {
        int returnCode = detectSingleLine(points, values, lines, i);
        Line line;

        if (returnCode) {
            cout << "RANSAC failed to find line number " << i + 1 << endl;
            //rotate points and retry
            mat newPoints(points.n_cols, points.n_rows);
            for (size_t j = 0; j < points.n_rows; ++j) {
                newPoints(0, j) = points(j, 1);
                newPoints(1, j) = points(j, 0);
            }
            
            newPoints = newPoints.t();


            returnCode = detectSingleLine(newPoints, values, lines, i);

            if (returnCode) {
                cout << "RANSAC failed to find line number " << i + 1 << " again" << endl;
                continue;
            }

            line = Line{randsac.bestFit.params[1], randsac.bestFit.params[0], randsac.bestScore, randsac.bestFit.vertical, {0, 0}, {0, 0}};
            //use a rotated image to get end points also
            _getEndPoints(line, true);
            //roate back end points
            line.end = cv::Point(line.end.y, line.end.x);
            line.start = cv::Point(line.start.y, line.start.x);
            cout << "Line " << i+1 << " rotated ---------------------------" << endl;

        }
        else{
            line = Line{randsac.bestFit.params[1], randsac.bestFit.params[0], randsac.bestScore, randsac.bestFit.vertical, {0, 0}, {0, 0}};
            _getEndPoints(line);

        }


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

cv::Mat LinedetectorPipe::drawResults(const cv::Mat &img, const vector<Line> &lines){
    // Draw the lines
    cv::Mat img2 = img.clone();

    _preprocess(img2);
    cv::resize(img2, img2, cv::Size(size, size));
    img2.convertTo(img2, CV_8U);
    
    cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < lines.size(); i += 1) {
        // Scale line endpoints back to original image coordinates if necessary
        cv::line(img2, lines[i].start, lines[i].end, cv::Scalar(255, 0, 255), 2);
    }
    return img2;
}
