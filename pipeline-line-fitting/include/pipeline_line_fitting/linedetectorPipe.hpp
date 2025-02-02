#include <armadillo>
#include <iostream>
#include <pipeline_line_fitting/randsac.hpp>

// using namespace cv;
using namespace std;
using namespace arma;

class LinedetectorPipe {
  int n;
  int k;
  float t;
  float fracOfPoints;
  float removeT;
  float finalScorethresh;
  float minTurnAngle;
  int size;
  cv::Mat orgImg;         // Original image
  double original_width;  // Member variable for original image width
  double original_height; // Member variable for original image height
  double scaleX;          // Scaling factor for x-axis
  double scaleY;          // Scaling factor for y-axis

  RANDSAC randsac;
  cv::Mat processedImg;

  // quick summary of the pipeline
public:
  void _preprocess(cv::Mat &img, bool dist = true);
  int detectSingleLine(const mat &points, const mat &values,
                       const vector<Line> &lines, const int i);
  void _postprocess();
  void _getEndPoints(Line &line, bool swap = false);

public:
  ~LinedetectorPipe();
  // call operator is the entry point for the pipeline
  vector<Line> operator()(const cv::Mat &img, const int maxLines);
  cv::Mat drawResults(const cv::Mat &img, const vector<Line> &lines);

  LinedetectorPipe(int n = 5, int k = 500, float t = 50.0,
                   float fracOfPoints = 0.001, float removeT = 1000.0,
                   float finalScorethresh = 65.0, float minTurnAngle = 1.5,
                   int size = 200)
      : n(n), k(k), t(t), fracOfPoints(fracOfPoints), removeT(removeT),
        finalScorethresh(finalScorethresh), minTurnAngle(minTurnAngle),
        size(size) {
    /*
    A line detector using RANSAC to find to lines in a bitmask image
    n: How many random points to choose to start each iteration
    k: How many iterations to run
    t: Threshold for points to be considered inliers, for the purpose of scoring
    fracOfPoints: minimum fraction of points that need to bee inliers for a line
    to be considered removeT: Threshold for points to be removed between
    iterations finalScorethresh: required score to be counted as a detected line
    minTurnAngle: minimum angle difference between lines, radians (ofc)
    size: what size of square the image will be resized to during preprocessing
    */
    randsac = RANDSAC(n, k, t, 2, removeT, finalScorethresh, minTurnAngle);
  }
};
