#pragma once
#ifndef FACTORS_H
#define FACTORS_H

#define _USE_MATH_DEFINES
#define EIGEN_STACK_ALLOCATION_LIMIT 0
#define PI 3.1416

#include <iostream>
#include <Eigen/Dense>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
#include <vector>

using namespace Eigen;
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <bits/stdc++.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>
#include <fstream>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/linear/LinearInequality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <cstdlib>
#include <chrono> 
#include <ctime> 
#include <ct/optcon/optcon.h>  
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Rot2.h>
#include <boost/optional.hpp>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;
using namespace std;

class Factor1: public NoiseModelFactor4<VectorXd, VectorXd, VectorXd, VectorXd> {
    public:
        typedef boost::shared_ptr<Factor1> shared_ptr;
        gtsam::Vector value_;

        Factor1(Key x1, Key x2, Key x3, Key x4, const SharedNoiseModel& model):
            NoiseModelFactor4<VectorXd, VectorXd, VectorXd, VectorXd>(model, x1, x2, x3, x4){}

        virtual ~Factor1(){}

        Vector evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3, const VectorXd& X4,
                            boost::optional<gtsam::Matrix&> H1 = boost::none,
                            boost::optional<gtsam::Matrix&> H2 = boost::none,
                            boost::optional<gtsam::Matrix&> H3 = boost::none,
                            boost::optional<gtsam::Matrix&> H4 = boost::none) const;

};

class Factor2: public NoiseModelFactor3<VectorXd, VectorXd, VectorXd> {
    public:
        typedef boost::shared_ptr<Factor2> shared_ptr;
        gtsam::Vector value_;

        Factor2(Key x1, Key x2, Key x3, const SharedNoiseModel& model):
            NoiseModelFactor3<VectorXd, VectorXd, VectorXd>(model, x1, x2, x3){}

        virtual ~Factor2(){}

        Vector evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1 = boost::none,
                            boost::optional<gtsam::Matrix&> H2 = boost::none,
                            boost::optional<gtsam::Matrix&> H3 = boost::none) const;
};

class Factor3: public NoiseModelFactor3<VectorXd, VectorXd, VectorXd> {
    public:
        typedef boost::shared_ptr<Factor3> shared_ptr;
        gtsam::Vector value_;

        Factor3(Key x1, Key x2, Key x3, const SharedNoiseModel& model):
            NoiseModelFactor3<VectorXd, VectorXd, VectorXd>(model, x1, x2, x3){}

        virtual ~Factor3(){}

        Vector evaluateError(const VectorXd& X1, const VectorXd& X2, const VectorXd& X3,
                            boost::optional<gtsam::Matrix&> H1 = boost::none,
                            boost::optional<gtsam::Matrix&> H2 = boost::none,
                            boost::optional<gtsam::Matrix&> H3 = boost::none) const;
};

class Factor4: public NoiseModelFactor2<VectorXd, VectorXd> {
    public:
        typedef boost::shared_ptr<Factor4> shared_ptr;
        gtsam::Vector value_;

        Factor4(Key x1, Key x2, const SharedNoiseModel& model):
            NoiseModelFactor2<VectorXd, VectorXd>(model, x1, x2){}

        virtual ~Factor4(){}

        Vector evaluateError(const VectorXd& X1, const VectorXd& X2,
                            boost::optional<gtsam::Matrix&> H1 = boost::none,
                            boost::optional<gtsam::Matrix&> H2 = boost::none) const;
};

#endif