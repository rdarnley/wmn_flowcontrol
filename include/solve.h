#pragma once
#ifndef SOLVE_H
#define SOLVE_H

#include "factor.h"


using namespace gtsam;
using namespace std;

class SolverOutput{

    public:

        Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic> states;
        MatrixXd controls;
        double cost;
        double runtime;
        int orderingType;
        int numTimeSteps;
        int solverType;

        SolverOutput(){}

        SolverOutput(int numNodes, int numTimes, int numControls){
            states = Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic>(numNodes, numTimes);
            controls = MatrixXd(numControls, numTimes);
        }

};

void processResult(const vector<vector<SolverOutput>>& result);

class FGSolver{

    private:

        // Operational Values
        uint32_t numTimeSteps;
        uint32_t numNodes;
        uint32_t numControl;
        uint32_t initBuffer;
        uint32_t initRate;
        uint32_t finalBuffer;
        uint32_t finalRate;

        uint32_t numStates;
        double timeStep;

        // Cost Metrics
        double qCost;
        double qfCost;
        double rCost;

        gtsam::Vector qNode, qfNode, rNode, uEstimate;
        MatrixXd qMatrix, qfMatrix, rMatrix;

        // Noise Models
        noiseModel::Diagonal::shared_ptr priorNoise;
        noiseModel::Diagonal::shared_ptr dynNoise;

        // Other Graph Info
        gtsam::Vector priorState;
        gtsam::Vector finalState;

        vector<vector<Key>> X;
        vector<vector<Key>> U;

        gtsam::Ordering order;
        NonlinearFactorGraph graph;

        Values initialEstimate;

        // Non-GTSAM Variation
        MatrixXd A_full, B_full, Q_full, Qf_full, R_full, X_init;

        // Other
        unordered_map<int, int> driverIdx;

        void matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat);

    public:

        FGSolver();

        void init(uint32_t _numTimeSteps, uint32_t _numNodes, uint32_t _numControl, uint32_t _initBuffer, uint32_t _initRate, uint32_t _finalBuffer, uint32_t _finalRate);

        SolverOutput FGSolve(int orderingType);

        SolverOutput FGSim(const SolverOutput& fgSoln);

        SolverOutput DARESolve();

        // Control Toolbox To Solve LQR Problem
        template<int numNodes2, int numControl2>
        SolverOutput CTSolve()
        {
            // constexpr int numNodes = _numNodes;
            // constexpr int numControl = _numControl;
            std::cout << "Entered Control Toolbox Solve" << std::endl;

            // template<int stateSpaceSize, int numControlCT>

            // assert(numControl == numControlCT);

                /**** Control toolbox for benchmarking ****/
            MatrixXd X_stacked = MatrixXd::Zero(numNodes2, numTimeSteps);
            MatrixXd U_stacked = MatrixXd::Zero(numControl2, numTimeSteps-1);

            shared_ptr<ct::optcon::TermQuadratic<numNodes2, numControl2>> intermCost(
                new ct::optcon::TermQuadratic<numNodes2, numControl2>(Q_full, R_full));

            shared_ptr<ct::optcon::TermQuadratic<numNodes2, numControl2>> finCost(
                new ct::optcon::TermQuadratic<numNodes2, numControl2>(Qf_full, R_full));

            shared_ptr<ct::optcon::CostFunctionQuadratic<numNodes2, numControl2>> quadraticCost(
                new ct::optcon::CostFunctionAnalytical<numNodes2, numControl2>());

            quadraticCost->addIntermediateTerm(intermCost);
            quadraticCost->addFinalTerm(finCost);

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now(); 

            ct::optcon::FHDTLQR<numNodes2, numControl2> lqrSolver(quadraticCost);
            ct::core::FeedbackArray<numNodes2, numControl2> K_ct;

            ct::core::StateVectorArray<numNodes2> x_ref_init(numTimeSteps, 
            ct::core::StateVector<numNodes2>::Zero() );

            ct::core::ControlVectorArray<numControl2> u0_ff(numTimeSteps-1, ct::core::ControlVector<numControl2>::Zero());

            ct::core::StateMatrix<numNodes2> A_ct(A_full);
            ct::core::StateMatrixArray<numNodes2> A_ct_vec(numTimeSteps-1, A_ct);
            ct::core::StateControlMatrix<numNodes2, numControl2> B_ct(B_full);
            ct::core::StateControlMatrixArray<numNodes2, numControl2> B_ct_vec(numTimeSteps-1, B_ct);

            lqrSolver.designController( x_ref_init, u0_ff, A_ct_vec, B_ct_vec, 0.05, K_ct);
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start; 
            std::cout << "elapsed time for Control Toolbox is: " << elapsed_seconds.count() << "s\n";

            SolverOutput ctSoln(numNodes2, numTimeSteps, numControl2);
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

};

#endif