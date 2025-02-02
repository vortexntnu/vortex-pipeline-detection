#ifndef RANDSAC_HPP
#define RANDSAC_HPP

#include <armadillo>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

struct Line {
    double slope;
    double intercept;
    double score;
    cv::Point start = {0, 0};
    cv::Point end = {0, 0};
};

arma::mat squareErrorLoss(const arma::mat &y_true, const arma::mat &y_pred);
double value(const arma::mat &inliersVal);

using LossFunction = arma::mat(*)(const arma::mat&, const arma::mat&);
using MetricFunction = double(*)(const arma::mat&);

struct LinearRegressor {
    vector<double> params;
    bool vertical;
    LinearRegressor();
    ~LinearRegressor();
    void fit(const arma::mat &X, const arma::mat &y);
    arma::mat predict(const arma::mat &X);
};

class RANDSAC{
    int n_;
    int k_;
    float t_;
    float remT_;
    float turnAngle_;
    LinearRegressor model_;
    LossFunction loss_;
    MetricFunction metric_;
    arma::mat points_;
    bool failed_;



    void reset();
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
            : n_(n), k_(k), t_(t), d(d), remT_(remT), turnAngle_(turnAngle), loss_(loss), metric_(metric), model_(model){
                /*
                n: Minimum number of data points to estimate parameters
                k: Maximum iterations allowed
                t: Threshold value to determine if points are fit well
                d: Number of close data points required to assert model fits well
                remT: Threshold value to determine if what points should be removed
                model: class implementing `fit` and `predict`
                loss: function of `y_true` and `y_pred` that returns a vector, this is the basis for the threshold
                metric: function of inliers that returns a scalar, larger is better
                */
               reset();
            }
    
        ~RANDSAC();
        arma::mat predict(const arma::mat &X);
        void fit (const arma::mat &X, const arma::mat &y, const arma::mat &values, const vector<Line> &lines);
};

#endif //RANDSAC_HPP