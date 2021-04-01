#include "factors.h"

Vector Factor1::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3, const VectorXd& X4,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3,
                            boost::optional<gtsam::Matrix&> H4) const {


    MatrixXd error;

    MatrixXd delta = MatrixXd::Zero(1,1);
    delta << 2;

    error = X1 + X2 - X3 - X4;

    if(H1) (*H1) = gtsam::Matrix::Identity(1,1);
    if(H2) (*H2) = gtsam::Matrix::Identity(1,1);
    if(H3) (*H3) = -gtsam::Matrix::Identity(1,1);
    if(H4) (*H4) = -gtsam::Matrix::Identity(1,1);


    return error;    

}

Vector Factor2::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3) const {

    MatrixXd error;

    MatrixXd delta = MatrixXd::Zero(1,1);
    delta << 2;

    error = X1 - X2 - X3;

    if(H1) (*H1) = gtsam::Matrix::Identity(1,1);
    if(H2) (*H2) = -gtsam::Matrix::Identity(1,1);
    if(H3) (*H3) = -gtsam::Matrix::Identity(1,1);

    return error;

}

Vector Factor3::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3) const {

    MatrixXd error;

    MatrixXd delta = MatrixXd::Zero(1,1);
    delta << 2;

    error = X1 + X2 - X3;

    if(H1) (*H1) = gtsam::Matrix::Identity(1,1);
    if(H2) (*H2) = gtsam::Matrix::Identity(1,1);
    if(H3) (*H3) = -gtsam::Matrix::Identity(1,1);

    return error;

}

Vector Factor4::evaluateError(const VectorXd& X1, const VectorXd& X2,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2) const {

    MatrixXd error;

    MatrixXd delta = MatrixXd::Zero(1,1);
    delta << 2;

    error = X1 - X2;


    if(H1) (*H1) = gtsam::Matrix::Identity(1,1);
    if(H2) (*H2) = -gtsam::Matrix::Identity(1,1);


    return error;

}