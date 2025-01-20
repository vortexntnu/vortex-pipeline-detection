
#include <pipeline_line_fitting/randsac.hpp>

using namespace std;

using namespace arma;

//Utils
mat squareErrorLoss(const mat &y_true, const mat &y_pred){
    return square(y_true - y_pred);
}

double value(const mat &inliersVal){
    return inliersVal.size();
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

    mat XtX = X_.t() * X_;
    //double cond_number = cond(XtX);

    if (X.min() == X.max() && X.size() >= 5) { // Threshold for condition number
        // Handle the case where the matrix is not invertible
        // For example, you can set params to a default value or use a regularization method
        vertical = true;
        params = {X.at(0), 0};

    } else {
        // Solve the normal equation
        params = conv_to<vector<double>>::from(solve(XtX, X_.t() * y));
        vertical = false;
    }
    //params = conv_to<vector<double>>::from(solve(X_, y));
}

mat LinearRegressor::predict(const mat &X){
    if (vertical){
        /*cout << ones(X.n_rows) * params.at(0) << endl;
        cout << params.at(0) << endl;*/
        return ones(X.n_rows) * params.at(0); // the vertical line will return x-predections, instead of y-predictions
    }

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

    int verticals = 0;
    int vertScore = 0;
    int others = 0;
    int otherScore = 0;

    random_device rd;
    mt19937 rng(rd());
    vector<int> ids(X.n_rows);
    iota(ids.begin(), ids.end(), 0);
    mat y_pred;
    mat x_pred;
    umat thresholded;

    for (int i = 0; i < k; i++) {
        shuffle(ids.begin(), ids.end(), rng);

        vector<int> maybe_inliers(ids.begin(), ids.begin() + n);
        uvec maybeInliersUvec = conv_to<uvec>::from(maybe_inliers);

        LinearRegressor maybe_model = model;
        maybe_model.fit(X.rows(maybeInliersUvec), y.rows(maybeInliersUvec));

        y_pred.clear();
        x_pred.clear();
        thresholded.clear();


        if (maybe_model.vertical && false){
            verticals++;
            //in case of a vertical line, the dimensions must be swapped
            thresholded.clear();

            x_pred = maybe_model.predict(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
            thresholded = loss(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), x_pred) < t;
        }
        else{
            others++;
            y_pred = maybe_model.predict(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
            thresholded = loss(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), y_pred) < t;
        }

        
        vector<int> inlier_ids;
        for (size_t j = n; j < ids.size(); j++) {
            if (thresholded(j - n)) {
                inlier_ids.push_back(ids[j]);
            }
        }
        //if (maybe_model.vertical){cout << "inlier_ids: " << inlier_ids.size() << " " << maybe_model.params.at(0) << endl;}

        if (inlier_ids.size() > 10){//} && legalAlpha(maybe_model.params[1], lines)) {
            failed = false;

            vector<int> inlier_points = maybe_inliers;
            uvec inlierIdsUvec = conv_to<uvec>::from(inlier_ids);
            inlier_points.insert(inlier_points.end(), inlier_ids.begin(), inlier_ids.end());

            LinearRegressor better_model = model;
            better_model.fit(X.rows(inlierIdsUvec), y.rows(inlierIdsUvec));


            double thisValue = metric(values.rows(inlierIdsUvec));

            if (thisValue > bestScore) {
                
                orgPoints = maybe_inliers;

                //find points that are inliers to remove
                umat rem_thresholded;
                mat rem_y_pred = better_model.predict(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
                mat rem_x_pred;
                rem_thresholded = loss(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), rem_y_pred) < remT;

                if (better_model.vertical){
                    vertScore++;
                    rem_x_pred = better_model.predict(y.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))));
                    rem_thresholded = loss(X.rows(conv_to<uvec>::from(vector<int>(ids.begin() + n, ids.end()))), rem_x_pred) < remT;
                }
                else{
                    otherScore++;
                }

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
    /*
    cout << "Verticals: " << verticals << " Others: " << others << endl;
    cout << "Verticals with score: " << vertScore << " Others with score: " << otherScore << endl;*/
}
