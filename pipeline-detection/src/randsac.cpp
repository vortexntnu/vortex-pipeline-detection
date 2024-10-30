
#include <ros_linefitting/randsac.h>

using namespace std;

using namespace arma;

//Utils
mat squareErrorLoss(const mat &y_true, const mat &y_pred){
    return square(y_true - y_pred);
}

double value(const mat &inliersVal){
    return mean(mean(inliersVal));
}


//LinearRegressor
LinearRegressor::LinearRegressor(){
    params = {};
}

LinearRegressor::~LinearRegressor(){
    params.clear();
}

void LinearRegressor::fit(const mat &X, const mat &y){
    //make sure to account for matrix not invertible error
    mat X_ = join_horiz(ones(X.n_rows, 1), X);
    params = conv_to<vector<double>>::from(solve(X_, y));
}

mat LinearRegressor::predict(const mat &X){
    mat X_ = join_horiz(ones(X.n_rows, 1), X);
    return X_ * conv_to<mat>::from(params);
}

//RANDSAC
void RANDSAC::_reset(){
    bestScore = -999999;
    failed = false;
}

RANDSAC::~RANDSAC(){
    _reset();
}

bool RANDSAC::legalAlpha(double alpha, const vector<Line> &lines){
    for (int i = 0; i < lines.size(); i++){
        if (abs(atan(lines[i].slope) - atan(alpha)) < turnAngle){
            return false;
        }
    }
    return true;
}
mat RANDSAC::predict(const mat &X){
    return bestFit.predict(X);
}

void RANDSAC::fit (const mat &X, const mat &y, const mat &values, const vector<Line> &lines){
    _reset();
    random_device rd;
    mt19937 rng(rd());
    vector<int> ids(X.n_rows);
    iota(ids.begin(), ids.end(), 0);

    for (int i = 0; i < k; i++) {
        shuffle(ids.begin(), ids.end(), rng);

        vector<int> maybe_inliers(ids.begin(), ids.begin() + n);
        uvec maybeInliersUvec = conv_to<uvec>::from(maybe_inliers);

        LinearRegressor maybe_model = model;
        maybe_model.fit(X.rows(maybeInliersUvec), y.rows(maybeInliersUvec));

        mat y_pred = maybe_model.predict(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
        umat thresholded = loss(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), y_pred) < t;

        vector<int> inlier_ids;
        for (size_t j = n; j < ids.size(); j++) {
            if (thresholded(j - n)) {
                inlier_ids.push_back(ids[j]);
            }
        }

        if (inlier_ids.size() > d && legalAlpha(maybe_model.params[1], lines)) {
            failed = false;
            vector<int> inlier_points = maybe_inliers;
            uvec inlierIdsUvec = conv_to<uvec>::from(inlier_ids);
            inlier_points.insert(inlier_points.end(), inlier_ids.begin(), inlier_ids.end());

            LinearRegressor better_model = model;
            better_model.fit(X.rows(inlierIdsUvec), y.rows(inlierIdsUvec));

            double thisValue = metric(values.rows(inlierIdsUvec));

            if (thisValue > bestScore) {
                orgPoints = maybe_inliers;

                mat rem_y_pred = better_model.predict(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
                umat rem_thresholded = loss(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), rem_y_pred) < remT;

                rempointids.clear();
                for (size_t j = n; j < ids.size(); j++) {
                    if (rem_thresholded(j - n)) {
                        rempointids.push_back(ids[j]);
                    }
                }

                bestScore = thisValue;
                bestFit = better_model;
            }
        }
    }
}
