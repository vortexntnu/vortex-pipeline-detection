#ifndef LINDETECTORPIPE_HPP
#define LINDETECTORPIPE_HPP

#include <armadillo>
#include <iostream>
#include <pipeline_line_fitting/randsac.hpp>

struct RandsacParams {
    int n;
    int k;
    float t;
    float fracOfPoints;
    float removeT;
    float finalScorethresh;
    float minTurnAngle;
    int size;
};

class LinedetectorPipe {
    int n_;
    int k_;
    float t_;
    float fracOfPoints_;
    float removeT_;
    float finalScorethresh_;
    float minTurnAngle_;
    int size_;
    cv::Mat orgImg_;  // Original image
    double scaleX_;   // Scaling factor for x-axis
    double scaleY_;   // Scaling factor for y-axis

    RANDSAC randsac_;
    cv::Mat processedImg_;

    // quick summary of the pipeline
   public:
    void preprocess(cv::Mat& img, bool dist = true);
    int detectSingleLine(const arma::mat& points,
                         const arma::mat& values,
                         const std::vector<Line>& lines,
                         const int i);
    void postprocess();
    void getEndPoints(Line& line, bool swap = false);

   public:
    ~LinedetectorPipe();
    // call operator is the entry point for the pipeline
    std::vector<Line> operator()(const cv::Mat& img, const int maxLines);

    LinedetectorPipe();
    LinedetectorPipe(RandsacParams params) {
        // A line detector using RANSAC to find to lines in a bitmask image
        n_ = params.n;
        k_ = params.k;
        t_ = params.t;
        fracOfPoints_ = params.fracOfPoints;
        removeT_ = params.removeT;
        finalScorethresh_ = params.finalScorethresh;
        minTurnAngle_ = params.minTurnAngle;
        size_ = params.size;

        randsac_ =
            RANDSAC(n_, k_, t_, 2, removeT_, finalScorethresh_, minTurnAngle_);
    }
};

#endif  // LINDETECTORPIPE_HPP
