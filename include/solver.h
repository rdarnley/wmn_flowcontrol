#pragma once
#ifndef SOLVER_H
#define SOLVER_H

#include "factor.h"
#include "centralized.h"

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

// void processResult(const vector<vector<SolverOutput>>& result);

// template<class SolverOutput>
// void PrintResultTerminal(const vector<SolverOutput>& output);

class LqrSolver {

    public:

        LqrSolver();

        template<class SimulationEnvironment>
        void Init(SimulationEnvironment env){
            num_timesteps = env.num_timesteps;
            timestep = env.timestep;

            state_cost = env.state_cost;
            statef_cost = env.statef_cost;
            control_cost = env.control_cost;
        }

        void CreateScenario(int num_nodes_, int num_control_, int state_space_size_, int queue_init_, int rate_init_, int queue_final_, int rate_final_);

        SolverOutput SolveFG(int orderType);

        SolverOutput SolveDARE();

		void SolvePartial();

        template<int stateSpaceSize, int numControlCT>
		SolverOutput SolveCT(){

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
		matrixToOutput(ctSoln, X_stacked);

		return ctSoln;
		}

    private:

        int num_timesteps;
        double timestep;
        int num_nodes;
        int num_control;

        int state_space;
        int state_space_total;

        int queue_init;
        int rate_init;
        int queue_final;
        int rate_final;

        double state_cost;
        double statef_cost;
        double control_cost;

        gtsam::Vector Q_node, Qf_node, R, U_est;
        MatrixXd qmat_node, qfmat_node, rmat;

        noiseModel::Diagonal::shared_ptr prior_noise;
        noiseModel::Diagonal::shared_ptr dyn_noise;

		// noiseModel::Gaussian::shared_ptr state_cost_node;
		// noiseModel::Gaussian::shared_ptr statef_cost_node;
		// noiseModel::Gaussian::shared_ptr control_cost_node;

        gtsam::Vector prior_state;
        gtsam::Vector final_state;

        vector<vector<Key>> X;
        vector<vector<Key>> U;

        gtsam::Ordering order;
        // NonlinearFactorGraph graph;
        GaussianFactorGraph graph;

		// std::pair< boost::shared_ptr<typename gtsam::EliminateableFactorGraph<gtsam::FactorGraph>::BayesNetType>. boost::shared_ptr<gtsam::FactorGraph>> pairing;

		std::pair< boost::shared_ptr< gtsam::GaussianBayesNet >, boost::shared_ptr< gtsam::GaussianFactorGraph > > pairing;

        Values initialEstimate;

        MatrixXd A_full, B_full, Q_full, Qf_full, R_full, X_init;

        unordered_map<int, int> driver_idx;

        void matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat);

        vector<SolverOutput> outputs;

};


#endif
