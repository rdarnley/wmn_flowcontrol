#ifndef LQR_H
#define LQR_H

#pragma once

#include <yaml-cpp/yaml.h>
#include <iostream>


#include <utility>
#include <vector>
#include <map>

#include "factor.h"

#include <Eigen/Dense>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
#include <algorithm>

using namespace Eigen;
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <bits/stdc++.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>

#include "network_utils.h"

class SolverOutput{
public:
	Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic> states;
	MatrixXd controls;
	double cost;
	double runtime;
	int orderingType; // 0 - COLAMD, 1 - Serial, 2 - Interwoven
	int num_timesteps;
	std::string solverType; 
    int num_nodes;
    int num_control;

	SolverOutput(){}

	SolverOutput(int n_nodes, int n_times, int n_controls){		
		states = Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic>(n_nodes, n_times);
		controls = MatrixXd(n_controls, n_times);
        num_nodes = n_nodes;
        num_control = n_controls;		
	}
};

class LqrSolver{

    public:

        LqrSolver(){}
        LqrSolver(YAML::Node lqr_info, WirelessNetwork network_);

        void CreateLqrFg();
        void SolveLqrFg();
        void LqrMatrix();
        void LqrCt();

    private:

        WirelessNetwork network;

        int num_timesteps;
        double timestep;

        double state_cost;
        double statef_cost;
        double control_cost;

        int queue_init;
        double rate_init;
        int queue_final;
        double rate_final;

        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr dyn_noise;

        gtsam::GaussianFactorGraph graph;

        gtsam::Vector prior_state;
        gtsam::Vector final_state;

        std::map<int, vector<gtsam::Key>> X;
        std::map<int, vector<gtsam::Key>> U;

        gtsam::Vector Q_node;
        gtsam::Vector Qf_node;
        gtsam::Vector R_node;

        MatrixXd A_full, B_full, Q_full, Qf_full, R_full, X_init;

};

#endif