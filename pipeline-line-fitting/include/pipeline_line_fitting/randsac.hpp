#include <armadillo>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace arma;

struct Line{
    double slope;
    double intercept;
    double score;
    cv::Point start;
    cv::Point end;
};

mat squareErrorLoss(const mat &y_true, const mat &y_pred);
double value(const mat &inliersVal);

using LossFunction = mat(*)(const mat&, const mat&);
using MetricFunction = double(*)(const mat&);

struct LinearRegressor{
    vector<double> params;
    LinearRegressor();
    ~LinearRegressor();
    void fit(const mat &X, const mat &y, double lambda = 11);
    mat predict(const mat &X);
};

class RANDSAC{
    int n;
    int k;
    float t;
    float remT;
    float turnAngle;
    LinearRegressor model;
    LossFunction loss;
    MetricFunction metric;
    mat points;
    bool failed;



    void _reset();
    bool legalAlpha(double alpha, const vector<Line> &lines);
    public:

        double bestScore;
        LinearRegressor bestFit;

        vector<int> orgPoints;
        vector<int> rempointids;

        int d;

        RANDSAC(){};
        RANDSAC(
            int n, int k, float t, int d, float remT, float finalScorethresh, float turnAngle, 
            LinearRegressor model = LinearRegressor(), LossFunction loss = squareErrorLoss, MetricFunction metric = value) 
            : n(n), k(k), t(t), d(d), remT(remT), turnAngle(turnAngle), loss(loss), metric(metric), model(model){
                /*
                n: Minimum number of data points to estimate parameters
                k: Maximum iterations allowed
                t: Threshold value to determine if points are fit well
                d: Number of close data points required to assert model fits well
                remT: Threshold value to determine if what points should be removed
                model: class implementing `fit` and `predict`
                loss: function of `y_true` and `y_pred` that returns a vector, this is the basis for the threshold
                metric: function of inliers that returns a scalar, larger is better
                turnAngle: min angle-difference between lines. If you want more than two lines, this should be 0
                */
               _reset();
            }
    
        ~RANDSAC();
        mat predict(const mat &X);
        void fit (const mat &X, const mat &y, const mat &values, const vector<Line> &lines);
};