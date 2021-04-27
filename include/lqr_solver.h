#ifndef LQR_SOLVER_H
#define LQR_SOLVER_H

#include <iostream>

#define _USE_MATH_DEFINES
#define EIGEN_STACK_ALLOCATION_LIMIT 0

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

// using namespace gtsam;
using namespace std;

class LqrSolver{

    public:

        LqrSolver(){}

        template<class Environment>
        void createScenario(Environment env, int _initBuffer, int _initRate, int _finalBuffer, int _finalRate)
        {
            std::cout << "Creating Dynamical Scenario" << std::endl;

            // Initialize All Other Variables
            timeStep = env.timeStep;
            numTimeSteps = env.numTimeSteps;
            numNodes = env.numNodes;
            numControl = env.numControl;
            numStates = 2;
            qCost = env.qCost;
            qfCost = env.qfCost;
            rCost = env.rCost;

            stateSpaceSize = numStates * numNodes;

            // Assign Control Nodes
            controlIdx = unordered_map<int, int>();
            for(int i=0; i<numControl; i++)
            {
                controlIdx[i*numNodes / numControl] = i;
            }

            // Initialize Full Matrix Version
            std::cout << "Setting Up Full Matrices For Benchmark" << std::endl;

            A = Eigen::MatrixXd::Zero(stateSpaceSize, stateSpaceSize);
            B = Eigen::MatrixXd::Zero(stateSpaceSize, numControl);
            Q = Eigen::MatrixXd::Zero(stateSpaceSize, stateSpaceSize);
            Qf = Eigen::MatrixXd::Zero(stateSpaceSize, stateSpaceSize);
            R = Eigen::MatrixXd::Zero(numControl, numControl);

            X_init = Eigen::MatrixXd::Zero(stateSpaceSize, 1);

            for(int i=0; i<numNodes; i++)
            {
                X_init(2*i, 0) = _initBuffer;
                X_init(2*i+1, 0) = _initRate;
            }

            // Create Constrained Noise Models (Constrained NM = 0)
            std::cout << "Setting Up Factor Graph" << std::endl;

            priorNoise = gtsam::noiseModel::Constrained::All(numStates);
            dynNoise = gtsam::noiseModel::Constrained::All(numStates);

            graph = gtsam::GaussianFactorGraph();

            // Create State and Control Keys/Nodes
            vector<gtsam::Key> temp;
            X = vector<vector<gtsam::Key>>(numNodes, temp);

            vector<gtsam::Key> tempU;
            U = vector<vector<gtsam::Key>>(numControl, tempU);

            // Add Keys/Nodes To Graph
            for(int i1=0; i1<numNodes; i1++)
            {
                for(int i2=0; i2<numTimeSteps; i2++)
                {
                    X[i1].push_back(gtsam::LabeledSymbol('x', i1, i2));
                } // end inner
            } // end outer

            for(int i1=0; i1<numControl; i1++){
                for(int i2=0; i2<numTimeSteps; i2++){
                    U[i1].push_back(gtsam::LabeledSymbol('u', i1, i2));
                } // end inner
            } // end outer

            // Create Constraints
            // Outer Loop --> Spatial
            // Inner Loop --> Temporal

            Eigen::MatrixXd posOne = Eigen::MatrixXd::Zero(1,1);
            posOne << 1;

            Eigen::MatrixXd controlCost = Eigen::MatrixXd::Zero(1,1);
            controlCost << rCost;

            for(int i1 = 0; i1<numNodes; i1++){

                // Check if node is active or passive
                bool activeNode = false; int activeIdx = -1;

                if(controlIdx.find(i1) != controlIdx.end()){
                    activeNode = true;
                    activeIdx = controlIdx[i1];
                } // end if statement

                // Incrementally Fill Full Matrices
                if (i1 == 0){
                    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(1,2);
                    a << 1, -1;;
                    A.block<1,2>(0, 0) = a;
                } else {
                    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(1,3);
                    a << 1, 1, -1;
                    A.block<1,3>(numStates*i1, numStates*i1-1) = a;
                }

                A.block<1,1>(numStates*i1+1, numStates*i1+1) = posOne;

                Q(numStates*i1, numStates*i1) = qCost;
                Q(numStates*i1+1, numStates*i1+1) = qCost;
                Qf(numStates*i1, numStates*i1) = qfCost;
                Qf(numStates*i1+1, numStates*i1+1) = qfCost;

                if(activeNode){
                    B.block<1,1>(numStates*i1+1, activeIdx) = posOne;

                    R.block<1,1>(activeIdx, activeIdx) = controlCost;
                }

                // Iterate Through Time Steps
                for(int i2=0; i2<numTimeSteps-1; i2++){

                    if(i1==0){
                        if(activeNode){
                            graph.emplace_shared<active_end>(X[i1][i2], U[activeIdx][i2], X[i1][i2+1], dynNoise);
                        } else if(!activeNode) {
                            graph.emplace_shared<passive_end>(X[i1][i2], X[i1][i2+1], dynNoise);
                        } else {
                            std::cout << "Error: Incorrect Factor 1" << std::endl;
                        }
                    } else {
                        if(activeNode){
                            graph.emplace_shared<active>(X[i1-1][i2], X[i1][i2], U[activeIdx][i2], X[i1][i2+1], dynNoise);
                        } else if(!activeNode) {
                            graph.emplace_shared<passive>(X[i1-1][i2], X[i1][i2], X[i1][i2+1], dynNoise);
                        } else {
                            std::cout << "Error: Incorrect Factor 2" << std::endl;
                        } // end inner if/else
                    } // end outer if/else
                } // end temporal for loop
 
            } // end spatial for loop


        } // end createScenario function

        // Control Toolbox To Solve LQR Problem
        template<class Environment>
        SolverOutput solveCT(Environment env)
        {
            const int stateSpace = stateSpaceSize;
            const int numControlled = numControl;

            std::cout << "Entered Control Toolbox Solve" << std::endl;

            // template<int stateSpaceSize, int numControlCT>

            // assert(numControl == numControlCT);

                /**** Control toolbox for benchmarking ****/
            MatrixXd X_stacked = MatrixXd::Zero(stateSpace, numTimeSteps);
            MatrixXd U_stacked = MatrixXd::Zero(stateSpace, numTimeSteps-1);

            shared_ptr<ct::optcon::TermQuadratic<stateSpace, numControl2>> intermCost(
                new ct::optcon::TermQuadratic<stateSpace, numControl2>(Q_full, R_full));

            shared_ptr<ct::optcon::TermQuadratic<stateSpace, numControl2>> finCost(
                new ct::optcon::TermQuadratic<stateSpace, numControl2>(Qf_full, R_full));

            shared_ptr<ct::optcon::CostFunctionQuadratic<stateSpace, numControl2>> quadraticCost(
                new ct::optcon::CostFunctionAnalytical<stateSpace, numControl2>());

            quadraticCost->addIntermediateTerm(intermCost);
            quadraticCost->addFinalTerm(finCost);

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now(); 

            ct::optcon::FHDTLQR<stateSpace, numControl2> lqrSolver(quadraticCost);
            ct::core::FeedbackArray<stateSpace, numControl2> K_ct;

            ct::core::StateVectorArray<stateSpace> x_ref_init(numTimeSteps, 
            ct::core::StateVector<numNodes2>::Zero() );

            ct::core::ControlVectorArray<numControl2> u0_ff(numTimeSteps-1, ct::core::ControlVector<numControl2>::Zero());

            ct::core::StateMatrix<stateSpace> A_ct(A_full);
            ct::core::StateMatrixArray<stateSpace> A_ct_vec(numTimeSteps-1, A_ct);
            ct::core::StateControlMatrix<stateSpace, numControl2> B_ct(B_full);
            ct::core::StateControlMatrixArray<stateSpace, numControl2> B_ct_vec(numTimeSteps-1, B_ct);

            lqrSolver.designController( x_ref_init, u0_ff, A_ct_vec, B_ct_vec, 0.05, K_ct);
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start; 
            std::cout << "elapsed time for Control Toolbox is: " << elapsed_seconds.count() << "s\n";

            SolverOutput ctSoln(stateSpace, numTimeSteps, numControl2);
            ctSoln.runtime = elapsed_seconds.count();
            ctSoln.solverType=2;

            X_stacked.col(0) = X_init;
            MatrixXd cost_ct = MatrixXd::Zero(1,1);
            for(int i1=0; i1<numTimeSteps-1; i1++){

                U_stacked.col(i1) = K_ct[i1] * X_stacked.col(i1);
                X_stacked.col(i1+1) = A_full * X_stacked.col(i1) + B_full * U_stacked.col(i1);

                cost_ct += X_stacked.col(i1).transpose() * Q_full * X_stacked.col(i1);
                cost_ct += U_stacked.col(i1).transpose() * R_full * U_stacked.col(i1);		

            }
            cost_ct += X_stacked.col(numTimeSteps-1).transpose() * Qf_full * X_stacked.col(numTimeSteps-1);
            ctSoln.cost = cost_ct(0,0);  
            cout<<"Cost for Control Toolbox is "<<cost_ct<<endl;

            ctSoln.controls = U_stacked;
            matrixToOutput(ctSoln, X_stacked);

            return ctSoln;
        }

    private:

        // Dynamics Variables
        double timeStep;
        int numTimeSteps;
        int numStates;
        int numNodes;
        int numControl;
        int stateSpaceSize;

        // LQR Values
        double qCost;
        double qfCost;
        double rCost;

        // Factor Graph Variables
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
        gtsam::noiseModel::Diagonal::shared_ptr dynNoise;
        gtsam::GaussianFactorGraph graph;

        vector<vector<gtsam::Key>> X;
        vector<vector<gtsam::Key>> U;

        // Full Matrix Variables
        Eigen::MatrixXd X_init, A, B, Q, Qf, R;

        // Other Values
        unordered_map<int, int> controlIdx;

};



#endif