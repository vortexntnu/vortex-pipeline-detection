
#include <pipeline_line_fitting/randsac.hpp>

// why is this still here? because armadillo freaks out if it's not here, so
// here it will stay
using namespace std;

// Utils
arma::mat squareErrorLoss(const arma::mat& y_true, const arma::mat& y_pred) {
    return square(y_true - y_pred);
}

double value(const arma::mat& inliersVal) {
    return inliersVal.size();
}

// LinearRegressor
LinearRegressor::LinearRegressor() {
    params = {};
}

LinearRegressor::~LinearRegressor() {
    params.clear();
}

void LinearRegressor::fit(const arma::mat& X, const arma::mat& y) {
    // make sure to account for matrix not invertible error
    arma::mat X_ = join_horiz(arma::ones(X.n_rows, 1), X);

    arma::mat XtX = X_.t() * X_;

    if (X.min() == X.max() && X.size() >= 5) {
        vertical = true;
        params = {X.at(0), 0};

    } else {
        // Solve the normal equation
        params =
            arma::conv_to<std::vector<double>>::from(solve(XtX, X_.t() * y));
        vertical = false;
    }
}

arma::mat LinearRegressor::predict(const arma::mat& X) {
    if (vertical) {
        return arma::ones(X.n_rows) *
               params.at(0);  // the vertical line will return x-predections,
                              // instead of y-predictions
    }

    arma::mat X_ = join_horiz(arma::ones(X.n_rows, 1), X);
    return X_ * arma::conv_to<arma::mat>::from(params);
}

// RANDSAC
void RANDSAC::reset() {
    bestScore = -999999;
    failed_ = false;
}

RANDSAC::~RANDSAC() {
    reset();
}

bool RANDSAC::legalAlpha(double alpha, const vector<Line>& lines) {
    for (int i = 0; i < lines.size(); i++) {
        if (abs(atan(lines[i].slope) - atan(alpha)) < turnAngle_) {
            return false;
        }
    }
    return true;
}
arma::mat RANDSAC::predict(const arma::mat& X) {
    return bestFit.predict(X);
}

void RANDSAC::fit(const arma::mat& X,
                  const arma::mat& y,
                  const arma::mat& values,
                  const vector<Line>& lines) {
    reset();

    random_device rd;
    mt19937 rng(rd());
    vector<int> ids(X.n_rows);
    iota(ids.begin(), ids.end(), 0);
    arma::mat y_pred;
    arma::umat thresholded;

    for (int i = 0; i < k_; i++) {
        shuffle(ids.begin(), ids.end(), rng);

        vector<int> maybe_inliers(ids.begin(), ids.begin() + n_);
        arma::uvec maybeInliersUvec =
            arma::conv_to<arma::uvec>::from(maybe_inliers);

        LinearRegressor maybe_model = model_;
        maybe_model.fit(X.rows(maybeInliersUvec), y.rows(maybeInliersUvec));

        y_pred.clear();
        thresholded.clear();

        y_pred = maybe_model.predict(X.rows(arma::conv_to<arma::uvec>::from(
            vector<int>(ids.begin() + n_, ids.end()))));
        thresholded = loss_(y.rows(arma::conv_to<arma::uvec>::from(
                                vector<int>(ids.begin() + n_, ids.end()))),
                            y_pred) < t_;

        vector<int> inlier_ids;
        for (size_t j = n_; j < ids.size(); j++) {
            if (thresholded(j - n_)) {
                inlier_ids.push_back(ids[j]);
            }
        }

        if (inlier_ids.size() > 10 &&
            legalAlpha(maybe_model.params[1], lines)) {
            failed_ = false;

            vector<int> inlier_points = maybe_inliers;
            arma::uvec inlierIdsUvec =
                arma::conv_to<arma::uvec>::from(inlier_ids);
            inlier_points.insert(inlier_points.end(), inlier_ids.begin(),
                                 inlier_ids.end());

            LinearRegressor better_model = model_;
            better_model.fit(X.rows(inlierIdsUvec), y.rows(inlierIdsUvec));

            double thisValue = metric_(values.rows(inlierIdsUvec));

            if (thisValue > bestScore) {
                orgPoints = maybe_inliers;

                // find points that are inliers to remove
                arma::umat rem_thresholded;
                arma::mat rem_y_pred =
                    better_model.predict(X.rows(arma::conv_to<arma::uvec>::from(
                        vector<int>(ids.begin() + n_, ids.end()))));
                arma::mat rem_x_pred;
                rem_thresholded =
                    loss_(y.rows(arma::conv_to<arma::uvec>::from(
                              vector<int>(ids.begin() + n_, ids.end()))),
                          rem_y_pred) < remT_;

                if (better_model.vertical) {
                    rem_x_pred = better_model.predict(
                        y.rows(arma::conv_to<arma::uvec>::from(
                            vector<int>(ids.begin() + n_, ids.end()))));
                    rem_thresholded =
                        loss_(X.rows(arma::conv_to<arma::uvec>::from(
                                  vector<int>(ids.begin() + n_, ids.end()))),
                              rem_x_pred) < remT_;
                }

                rempointids.clear();
                for (size_t j = n_; j < ids.size(); j++) {
                    if (rem_thresholded(j - n_)) {
                        rempointids.push_back(ids[j]);
                    }
                }

                bestScore = thisValue;
                bestFit = better_model;
            }
        }
    }
}
