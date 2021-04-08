#include "factor.h"

Vector active_end::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3) const {


    MatrixXd error;

    MatrixXd A = MatrixXd::Zero(2,2);
    A << 1, -1, 0, 1;
    
    MatrixXd B = MatrixXd::Zero(2,1);
    B << 0, 1;

    error = A*X1 + B*X2 - X3;

    if(H1) (*H1) = A;
    if(H2) (*H2) = B;
    if(H3) (*H3) = -gtsam::Matrix::Identity(2,2);

    return error;    

}

Vector passive_end::evaluateError(const VectorXd& X1, const VectorXd& X2,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2) const {

    MatrixXd error;

    MatrixXd A = MatrixXd::Zero(2,2);
    A << 1, -1, 0, 1;

    error = A*X1 - X2;

    if(H1) (*H1) = A;
    if(H2) (*H2) = -gtsam::Matrix::Identity(2,2);

    return error;

}

Vector active::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3, const VectorXd& X4,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3,
                            boost::optional<gtsam::Matrix&> H4) const {

    MatrixXd error;

    MatrixXd A_last = MatrixXd::Zero(2,2);
    A_last << 0, 1, 0, 0;

    MatrixXd A = MatrixXd::Zero(2,2);
    A << 1, -1, 0, 1;

    MatrixXd B = MatrixXd::Zero(2,1);
    B << 0, 1;

    error = A_last*X1 + A*X2 + B*X3 - X4;

    if(H1) (*H1) = A_last;
    if(H2) (*H2) = A;
    if(H3) (*H3) = B;
    if(H4) (*H4) = -gtsam::Matrix::Identity(2,2);

    return error;

}

Vector passive::evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1,
                            boost::optional<gtsam::Matrix&> H2,
                            boost::optional<gtsam::Matrix&> H3) const {

    MatrixXd error;

    MatrixXd A_last = MatrixXd::Zero(2,2);
    A_last << 0, 1, 0, 0;

    MatrixXd A = MatrixXd::Zero(2,2);
    A << 1, -1, 0, 1;

    error = A_last*X1 + A*X2 - X3;


    if(H1) (*H1) = A_last;
    if(H2) (*H2) = A;
    if(H3) (*H3) = -gtsam::Matrix::Identity(2,2);

    return error;

}