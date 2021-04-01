#pragma once
#ifndef SOLVER_H
#define SOLVER_H

#include "factors.h"

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

class SolverOutput{
public:
	Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic> states;
	MatrixXd controls;
	double cost;
	double runtime;
	int orderingType; // 0 - COLAMD, 1 - Serial, 2 - Interwoven
	int num_time_steps;
	int solverType; // 0 - FG, 1 - FG sim, 2 - CT, 3 - LQR

	SolverOutput(){}

	SolverOutput(int n_nodes, int n_times, int n_controls){		
		states = Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic>(n_nodes, n_times);
		controls = MatrixXd(n_controls, n_times);		
	}
};

void processResult(const vector<vector<SolverOutput>>& result);

class FGSolver{
    private:

        int numTimeSteps;
        int numNodes;
        int numControl;
        int initValue;
        int finalValue;

        int numStates;
        double timeStep;

        double q;
        double q_f;
        double r;

        gtsam::Vector Q_node, Qf_node, R, U_est;
        MatrixXd qmat_node, qfmat_node, rmat;

        noiseModel::Diagonal::shared_ptr priorNoise;
        noiseModel::Diagonal::shared_ptr dynNoise;

        gtsam::Vector priorState;
        gtsam::Vector finalState;

        vector<vector<Key>> X;
        vector<vector<Key>> U;

        gtsam::Ordering order;
        NonlinearFactorGraph graph;

        Values initialEstimate;

        MatrixXd A_full, B_full, Q_full, Qf_full, R_full, X_init;

        unordered_map<int, int> driverIdx;

        void matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat);

    public:

        FGSolver();

        void init(int numTimeSteps_, int numNodes_, int numControl_, int initValue_, int finalValue_);

        SolverOutput FGSolve(int orderingType);

        SolverOutput FGSim(const SolverOutput& fgSoln);

        SolverOutput DARESolve();

        // SolverOutput CTSolve();      

        // template<int stateSpaceSize, int numControlCT>
        // SolverOutput CTSolve(){
        // assert(numControl == numControlCT);

        // cout<<"numControlCT is "<<numControlCT<<", stateSpaceSize is "<<stateSpaceSize<<endl;

        //     /**** Control toolbox for benchmarking ****/
        // MatrixXd X_stacked = MatrixXd::Zero(numNodes * numStates, numTimeSteps);
        // MatrixXd U_stacked = MatrixXd::Zero(numControlCT, numTimeSteps-1);

        // shared_ptr<ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>> intermCost(
        //     new ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>(Q_full, R_full));

        // shared_ptr<ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>> finCost(
        //     new ct::optcon::TermQuadratic<stateSpaceSize, numControlCT>(Qf_full, R_full));

        // shared_ptr<ct::optcon::CostFunctionQuadratic<stateSpaceSize, numControlCT>> quadraticCost(
        //     new ct::optcon::CostFunctionAnalytical<stateSpaceSize, numControlCT>());

        // quadraticCost->addIntermediateTerm(intermCost);
        // quadraticCost->addFinalTerm(finCost);

        // std::chrono::time_point<std::chrono::system_clock> start, end;
        // start = std::chrono::system_clock::now(); 

        // ct::optcon::FHDTLQR<stateSpaceSize, numControlCT> lqrSolver(quadraticCost);
        // ct::core::FeedbackArray<stateSpaceSize, numControlCT> K_ct;

        // ct::core::StateVectorArray<stateSpaceSize> x_ref_init(numTimeSteps, 
        //         ct::core::StateVector<stateSpaceSize>::Zero() );

        // ct::core::ControlVectorArray<numControlCT> u0_ff(numTimeSteps-1, ct::core::ControlVector<numControlCT>::Zero());

        // ct::core::StateMatrix<stateSpaceSize> A_ct(A_full);
        // ct::core::StateMatrixArray<stateSpaceSize> A_ct_vec(numTimeSteps-1, A_ct);
        // ct::core::StateControlMatrix<stateSpaceSize, numControlCT> B_ct(B_full);
        // ct::core::StateControlMatrixArray<stateSpaceSize, numControlCT> B_ct_vec(numTimeSteps-1, B_ct);

        // lqrSolver.designController( x_ref_init, u0_ff, A_ct_vec, B_ct_vec, p.dt, K_ct);
        // end = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end - start; 
        // std::cout << "elapsed time for Control Toolbox is: " << elapsed_seconds.count() << "s\n";

        // SolverOutput ctSoln(numNodes, numTimeSteps, numControl);
        // ctSoln.runtime = elapsed_seconds.count();
        // ctSoln.solverType=2;

        // X_stacked.col(0) = X_init;
        // MatrixXd cost_ct = MatrixXd::Zero(1,1);
        // for(int i1=0; i1<numTimeSteps-1; i1++){

        //     U_stacked.col(i1) = K_ct[i1] * X_stacked.col(i1);
        //         X_stacked.col(i1+1) = A_full * X_stacked.col(i1) + B_full * U_stacked.col(i1);

        //         cost_ct += X_stacked.col(i1).transpose() * Q_full * X_stacked.col(i1);
        //         cost_ct += U_stacked.col(i1).transpose() * R_full * U_stacked.col(i1);		

        // }
        // cost_ct += X_stacked.col(numTimeSteps-1).transpose() * Qf_full * X_stacked.col(numTimeSteps-1);
        // ctSoln.cost = cost_ct(0,0);  
        // cout<<"Cost for Control Toolbox is "<<cost_ct<<endl;

        // ctSoln.controls = U_stacked;
        // matrixToOutput(ctSoln, X_stacked);

        // return ctSoln;
        // }

};


#endif
