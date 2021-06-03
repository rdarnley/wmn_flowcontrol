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
        // LqrSolver(YAML::Node lqr_info, WirelessNetwork network_);
        LqrSolver(YAML::Node lqr_info);

        void CreateLqrFg();
        SolverOutput SolveLqrFg();
        SolverOutput LqrMatrix();

        void CreateLqrFgDecentralized(int node_id, int prev_id, WirelessNetwork& network);
        gtsam::GaussianFactor::shared_ptr ConvertLqrFgDecentralized(int node_id, WirelessNetwork& network);
        gtsam::VectorValues SolveBayesNetDecentralized(int node_id, int prev_id);
        void ForwardPass(int node_id, int prev_id, WirelessNetwork& network);
        void BackwardPass(int node_id, int prev_id);
        void DisplayResults();

        template<int stateSpaceSize, int numControlCT>
		SolverOutput LqrCt(){

            const int num_nodes = network.num_nodes;
            // const int numControlCT = network.num_control;
            // const int stateSpaceSize = 2*num_nodes;

			/**** Control toolbox for benchmarking ****/
            MatrixXd X_stacked = MatrixXd::Zero(stateSpaceSize, num_timesteps);
            MatrixXd U_stacked = MatrixXd::Zero(numControlCT, num_timesteps-1);

            shared_ptr<ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>> intermCost(
                new ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>(Q_full, R_full));

            shared_ptr<ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>> finCost(
                new ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>(Qf_full, R_full));

            shared_ptr<ct::optcon::CostFunctionQuadratic<stateSpaceSize, numControlCT>> quadraticCost(
                new ct::optcon::CostFunctionAnalytical<stateSpaceSize, numControlCT>());

            quadraticCost->addIntermediateTerm(intermCost);
            quadraticCost->addFinalTerm(finCost);

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now(); 

            ct::optcon::FHDTLQR<stateSpaceSize, numControlCT> lqrSolver(quadraticCost);
            ct::core::FeedbackArray<stateSpaceSize, numControlCT> K_ct;

            ct::core::StateVectorArray<stateSpaceSize> x_ref_init(num_timesteps, 
                    ct::core::StateVector<stateSpaceSize>::Zero() );

            ct::core::ControlVectorArray<numControlCT> u0_ff(num_timesteps-1, ct::core::ControlVector<numControlCT>::Zero());

            ct::core::StateMatrix<stateSpaceSize> A_ct(A_full);
            ct::core::StateMatrixArray<stateSpaceSize> A_ct_vec(num_timesteps-1, A_ct);
            ct::core::StateControlMatrix<stateSpaceSize, numControlCT> B_ct(B_full);
            ct::core::StateControlMatrixArray<stateSpaceSize, numControlCT> B_ct_vec(num_timesteps-1, B_ct);

            lqrSolver.designController( x_ref_init, u0_ff, A_ct_vec, B_ct_vec, 0.05, K_ct);
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start; 

            SolverOutput ctSoln(num_nodes, num_timesteps, numControlCT);
            ctSoln.runtime = elapsed_seconds.count();
            ctSoln.solverType = "Control Toolbox";

            X_stacked.col(0) = X_init;
            MatrixXd cost_ct = MatrixXd::Zero(1,1);
            for(int i1=0; i1<num_timesteps-1; i1++){

                U_stacked.col(i1) = K_ct[i1] * X_stacked.col(i1);
                    X_stacked.col(i1+1) = A_full * X_stacked.col(i1) + B_full * U_stacked.col(i1);

                    cost_ct += X_stacked.col(i1).transpose() * Q_full * X_stacked.col(i1);
                    cost_ct += U_stacked.col(i1).transpose() * R_full * U_stacked.col(i1);		

            }
            cost_ct += X_stacked.col(num_timesteps-1).transpose() * Qf_full * X_stacked.col(num_timesteps-1);
            ctSoln.cost = cost_ct(0,0);  

            ctSoln.controls = U_stacked;

            return ctSoln;
		}

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