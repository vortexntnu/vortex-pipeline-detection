#include <iostream>
#include <ros_linefitting/linedetectorPipe.h>

#include <chrono>

using namespace std;

void LinedetectorPipe::_preprocess(cv::Mat &img, bool dist){
    
    cv::resize(img, img, cv::Size(size, size));

    if (dist){cv::distanceTransform(img, img, cv::DIST_L2, 5);}

    // Apply morphological opening
    cv::Mat kernel = cv::Mat::ones(1, 1, CV_8U);
    cv::morphologyEx(img, img, cv::MORPH_OPEN, kernel);
    img.convertTo(img, CV_8U);
}

LinedetectorPipe::~LinedetectorPipe(){
    // Destructor
}

void LinedetectorPipe::_postprocess(){
    // Postprocess
}

int LinedetectorPipe::detectSingleLine(const mat &points, const mat &values, const vector<Line> &lines, const int i){
    // Extract columns and reshape
    mat X = points.col(1);
    X.reshape(points.n_rows, 1);
    mat y = points.col(0);
    y.reshape(points.n_rows, 1);

    // Set the d parameter for RANDSAC
    int d = points.n_elem * fracOfPoints;
    randsac.d = d;

    // Fit the RANDSAC model
    randsac.fit(X, y, values, lines);

    // Print the best value
    cout << "Found line " << i+1 << " with score: " << randsac.bestScore << endl;


    // Check the best_fit and bestValue conditions
    // if params is empty, not solution was found
    if (randsac.bestFit.params.size() == 0 || randsac.bestScore < finalScorethresh) {
        return 1;
    }
    return 0;

}

void LinedetectorPipe::_getEndPoints(Line &line, const mat &points){
    //inliers er randsac.
    // Find the start and end points of the line, using our randsac friends
    // there probably is a better way to do this, but this part does not need to be fast
    int min_x = -1;
    int max_x = -1;
    int min_x_yval;
    int max_x_yval;

    for (int x = 0; x < size; ++x){
        int y = line.slope * x + line.intercept;

        if (y < 0 || y >= size){
            continue;
        }
        int pixel = processedImg.at<uchar>(y, x);
        if (pixel > 0){

            if (min_x == -1){
                min_x = x;
                min_x_yval = y;
            }
            if (x > max_x){
                max_x = x;
                max_x_yval = y;
            }
        }


    }

    line.start = cv::Point(min_x, min_x_yval);
    line.end = cv::Point(max_x, max_x_yval);

}

vector<Line> LinedetectorPipe::operator()(const cv::Mat &img, const int maxLines=2){
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

void LinedetectorPipe::drawResults(const cv::Mat &img, const vector<Line> &lines, string saveDest){
    // Draw the lines
    cv::Mat img2 = img.clone();
    cv::cvtColor(img, img2, cv::COLOR_GRAY2BGR);
    cv::resize(img2, img2, cv::Size(size, size));


    for (int i = 0; i < lines.size(); i+=1){
        cv::line(img2, lines[i].start, lines[i].end, cv::Scalar(255, 0, 255), 2);
    }
    cv::imwrite(saveDest, img2);
}
