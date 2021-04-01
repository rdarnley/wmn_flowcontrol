#include "solver.h"

void processResult(const vector<vector<SolverOutput>>& result){

    std::ofstream myFileResults("experimentResults.csv");
    int expCt = 0;
    for (auto solns: result){

        std::cout << "For experiment " << expCt << ", results are - " << std::endl;

        for (auto soln: solns){

            std::cout << "Solver Type - " << soln.solverType << ", Ordering is " << soln.orderingType;
            std::cout << ", Time steps are " << soln.num_time_steps << ", Runtime = " << soln.runtime;
            std::cout << ", Cost = " << soln.cost << std::endl;

            myFileResults << expCt << "," << soln.solverType << "," << soln.orderingType << "," << soln.num_time_steps << ",";
            myFileResults << soln.runtime << "," << soln.cost << "\n";
        } // end inner for loop

        myFileResults << "\n";
        expCt++;
    } // end outer for loop

    myFileResults.close();

} // end processResult

FGSolver::FGSolver(){

} // end FGSolver Constructor

void FGSolver::init(int numTimeSteps_, int numNodes_, int numControl_, int initValue_, int finalValue_) {
    // Parse Input
    numTimeSteps = numTimeSteps_;
    numNodes = numNodes_;
    numControl = numControl_;
    initValue = initValue_;
    finalValue = finalValue_;

    std::cout << "Initializing Solver" << std::endl;

    // Initialize Other Variables
    numStates = 1;
    timeStep = 0.05;
    q = 10;
    q_f = 100;
    r = 1;
    U_est = gtsam::Vector(1); U_est << 0.1;

    // Create Constrained Noise Models
    std::cout << "Creating Constrained Noise Models" << std::endl;
    priorNoise = noiseModel::Constrained::All(numStates);
    dynNoise = noiseModel::Constrained::All(numStates);

    // Initialize Prior
    priorState = gtsam::Vector(numStates); priorState << initValue;
    finalState = gtsam::Vector(numStates); finalState << finalValue;

    // Initialize Full Matrices
    std::cout << "Initializing Full Matrices" << std::endl;
    A_full = MatrixXd::Zero(numNodes, numNodes);
    B_full = MatrixXd::Zero(numNodes, numControl);
    Q_full = MatrixXd::Zero(numNodes, numNodes);
    Qf_full = MatrixXd::Zero(numNodes, numNodes);
    R_full = MatrixXd::Zero(numControl, numControl);

    // Assign Which Nodes Are Drivers ??? Others Fixed Rate ???
    std::cout << "Assign Drivers/Fixed Rate???" << std::endl;

    driverIdx = unordered_map<int, int>();
    for(int i=0; i<numControl; i++){
        driverIdx[i*numNodes/numControl] = i;
    } // end driverIdx for loop

    // std::cout << "Driver Idx" << std::endl;
    // std::cout << driverIdx << std::endl;

    // Create Initial States
    std::cout << "Created X Initial Vector" << std::endl;
    X_init = MatrixXd::Zero(numNodes, 1);

    std::cout << "Populating X Initial Vector" << std::endl;
    for(int i = 0; i < numNodes; i++) {
        X_init(i, 0) = priorState(0);
    } // end Initialization for loop

    std::cout << X_init << std::endl;

    // Initialize Factor Graph
    std::cout << "Initializing Nonlinear Factor Graph" << std::endl;
    graph = NonlinearFactorGraph();
    initialEstimate = Values();

    vector<Key> temp;
    X = vector<vector<Key>>(numNodes, temp);

    vector<Key> tempU;
    U = vector<vector<Key>>(numControl, tempU);

    std::cout << "Creating X Labeled Symbols. Pushing to Graph" << std::endl;

    for(int j=0; j<numNodes; j++) {
        for(int i=0; i<numTimeSteps; i++) {
            X[j].push_back(gtsam::LabeledSymbol('x', j, i));
        } // end inner for loop
    } // end outer for loop

    std::cout << "Creating U Labeled Symbols. Pushing to Graph" << std::endl;

    for(int j=0; j<numControl; j++) {
        for(int i=0; i<numTimeSteps; i++) {
            U[j].push_back(gtsam::LabeledSymbol('u', j, i));
        } // end inner for loop
    } // end outer for loop

    std::cout << "Set Initial State As Prior" << std::endl;

    for (int i=0; i<numNodes; i++) {
        graph.add(PriorFactor<gtsam::Vector>(X[i][0], priorState, priorNoise));
    } // end prior factor for loop

    std::cout << "Setting Individual Node Dynamics --> Linear Chain For Now" << std::endl;

    std::cout << "Adding Ternary Factors -- Dynamics Constraints" << std::endl;

    std::cout << "Outside Loop: Spatial" << std::endl;
    std::cout << "Inside Loop: Temporal" << std::endl;

    for(int i2 = 0; i2 < numNodes - 1; i2++) {

        // Check if node is active or passive
        bool applyControl = false; int controlIdx = -1; 
        bool lastApplyControl = false; int lastControlIdx = -1;

        if(driverIdx.find(i2) != driverIdx.end()){
            applyControl = true;
            controlIdx = driverIdx[i2];
        } // end apply control if statement

        if(driverIdx.find(i2-1) != driverIdx.end()){
            lastApplyControl = true;
            lastControlIdx = driverIdx[i2-1];
        }

        // Populate Full Matrices
        MatrixXd a = MatrixXd::Zero(1,1);
        a << 1;
        
        if (i2 == 0){
            A_full.block<1,1>(0,0) = a;     
        } else {                            // end first condition
            A_full.block<1,1>(i2, i2) = a;
        } // end else condition

        std::cout << "Check 1" << std::endl;

        std::cout << "I2 " << i2 << std::endl;
        std::cout << "Control IDX " << controlIdx << std::endl;
        std::cout << "Last Control IDX " <<lastControlIdx << std::endl;

        if(applyControl && lastApplyControl) {

            MatrixXd b = MatrixXd::Zero(1,2);
            b << 1, -1;
            B_full.block<1,2>(i2, controlIdx-1) = b;

        } else if (applyControl && !lastApplyControl) {

            MatrixXd b = MatrixXd::Zero(1,1);
            b << -1;
            B_full.block<1,1>(i2, controlIdx) = b;

        } else if (!applyControl && lastApplyControl) {

            MatrixXd b = MatrixXd::Zero(1,1);
            b << 1;
            B_full.block<1,1>(i2, lastControlIdx) = b;

        } else if (!applyControl && !lastApplyControl) {

            MatrixXd b = MatrixXd::Zero(1,1);

        } else {
            std::cout << "ERROR: Wrong node dynamics/constraints" << std::endl;
        }

        Q_full(i2, i2) = q;
        Qf_full(i2, i2) = q_f;

        std::cout << "Check 2" << std::endl;

        // Iterate Through TimeSteps
        for (int i1 = 0; i1<numTimeSteps-1; i1++) {

            // Add 5-Factor (X[i2][i1], X[i2+1][i1], X[i2][i1+1], X[i2+1][i1+1], U[controlIdx][i1], dynNoise);
            if(applyControl && lastApplyControl) {
                // Node Dynamics + Input From Prev Node Control + Output From Curr Node Control
                graph.emplace_shared<Factor1>(X[i2][i1], U[lastControlIdx][i1], U[controlIdx][i1], X[i2][i1+1], dynNoise);
            } else if (applyControl && !lastApplyControl) {
                // Node Dynamics + Output From Curr Node Control
                graph.emplace_shared<Factor2>(X[i2][i1], U[controlIdx][i1], X[i2][i1+1], dynNoise);
            } else if (!applyControl && lastApplyControl) {
                // Node Dynamics + Input From Prev Node Control
                graph.emplace_shared<Factor3>(X[i2][i1], U[lastControlIdx][i1], X[i2][i1+1], dynNoise);
            } else if (!applyControl && !lastApplyControl) {
                // Node Dynamics
                graph.emplace_shared<Factor4>(X[i2][i1], X[i2][i1+1], dynNoise);
            } else {
                std::cout << "ERROR: NO FACTOR WAS CREATED FOR THESE NODE CONDITIONS" << std::endl;
            }

        } // end inner for loop
    } // end outer for loop

    std::cout << "Initialize Factor Graph Costs" << std::endl;

    Q_node = gtsam::Vector(numStates); Q_node << q;
    Qf_node = gtsam::Vector(numStates); Qf_node << q_f;

    R = gtsam::Vector(1); R << r;

    qmat_node = Q_node.array().matrix().asDiagonal();
    qfmat_node = Qf_node.array().matrix().asDiagonal();
    rmat = R.array().matrix().asDiagonal();

    auto q_node = noiseModel::Gaussian::Information(qmat_node);
    auto qf_node = noiseModel::Gaussian::Information(qfmat_node);
    auto r_noise = noiseModel::Gaussian::Information(rmat);

    std::cout << "Add State Costs To Factor Graph" << std::endl;

    for(int i=0; i<numNodes; i++){
        for(int j=0; j<numTimeSteps; j++) {
            if(j==numTimeSteps - 1){
                graph.add(PriorFactor<gtsam::Vector>(X[i][j], finalState, qf_node));
            } else {
                graph.add(PriorFactor<gtsam::Vector>(X[i][j], finalState, q_node));
            }

            initialEstimate.insert(X[i][j], finalState);
        }

    }

    std::cout << "Add Control Costs To Factor Graph" << std::endl;

    for(int i=0; i<numControl; i++) {
        for(int j=0; j<numTimeSteps-1; j++) {
            graph.add(PriorFactor<gtsam::Vector>(U[i][j], U_est, r_noise));
            initialEstimate.insert(U[i][j], U_est);
        }
    }

    return;
    
}

// Solve Factor Graph With Different Variable Orderings
SolverOutput FGSolver::FGSolve(int orderingType) {
    std::cout << "Entered FG Solve" << std::endl;

    // Create Some Sort of Order
    std::cout << "Using COLAMD Ordering" << std::endl;
    order = graph.orderingCOLAMD();

    // Create Instance of Optimizer
    GaussNewtonOptimizer optimizer(graph, initialEstimate, order);

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    std::cout << "Before optimize call" << std::endl;
    // Optimize Factor Graph
    Values resultLQR = optimizer.optimize();

    std::cout << "After optimize call" << std::endl;

    // End Timer
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Factor Graph Took " << duration.count() << " seconds\n" << std::endl;

    SolverOutput fgSoln(numNodes, numTimeSteps, numControl);
    fgSoln.runtime = duration.count();
    fgSoln.solverType = 0;
    fgSoln.num_time_steps = numTimeSteps;
    // fgSoln.orderingType = ordering

    // Calculate Cost of Factor Graph
    double cost_fg = 0.;

    std::cout << "Calculating Total Cost of FG" << std::endl;

    for(int i1 = 0; i1 < numTimeSteps; i1++) {

        // Add Costs For States
        for(int i2=0; i2 < numNodes; i2++) {

            fgSoln.states(i2, i1) = ( gtsam::Vector(1) << resultLQR.at<VectorXd>(X[i2][i1])(0) ).finished();

            if (i1 < numTimeSteps - 1){
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*Q_node(0);
            } else {
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*Qf_node(0);
            }

        }

        // Add Costs For Controls
        if(i1 < numTimeSteps - 1) {
            for(int i4=0; i4<numControl; i4++) {

                fgSoln.controls(i4, i1) = resultLQR.at<VectorXd>(U[i4][i1])(0);
                cost_fg += pow(fgSoln.controls(i4,i1),2.)*R_full(i4,i4);
            } 
        }
    }     

    std::cout << "Total Cost for FG: " << cost_fg << std::endl;
    fgSoln.cost = cost_fg;

    return fgSoln;
}


SolverOutput FGSolver::FGSim(const SolverOutput& fgSoln)
{
    MatrixXd X_fg = MatrixXd::Zero(numNodes*numStates, numTimeSteps);

    MatrixXd cost_fgsim = MatrixXd::Zero(1,1);

    SolverOutput fgsimSoln = fgSoln;
    fgsimSoln.solverType=1;

    // Forward Simulation
    X_fg.col(0) = X_init;
    for (int i = 0; i < numTimeSteps - 1; i++){

        X_fg.col(i+1) = A_full * X_fg.col(i) + B_full * fgSoln.controls.col(i);

        cost_fgsim += X_fg.col(i).transpose() * Q_full * X_fg.col(i);
        cost_fgsim += fgSoln.controls.col(i).transpose()*R_full*fgSoln.controls.col(i);
    }

    cost_fgsim += X_fg.col(numTimeSteps-1).transpose() * Qf_full * X_fg.col(numTimeSteps-1);

    std::cout << "Cost for FG Sim is " << cost_fgsim << std::endl;

    fgsimSoln.cost = cost_fgsim(0,0);

    matrixToOutput(fgsimSoln, X_fg);

    return fgsimSoln;
}

// // Control Toolbox To Solve LQR Problem
// SolverOutput FGSolver::CTSolve()
// {
//     std::cout << "Entered Control Toolbox Solve" << std::endl;

//     // template<int stateSpaceSize, int numControlCT>

//     // assert(numControl == numControlCT);

// 		/**** Control toolbox for benchmarking ****/
//     MatrixXd X_stacked = MatrixXd::Zero(numNodes*numStates, numTimeSteps);
//     MatrixXd U_stacked = MatrixXd::Zero(numControl, numTimeSteps-1);

//     shared_ptr<ct::optcon::TermQuadratic<numNodes, numControl>> intermCost(
// 	  	new ct::optcon::TermQuadratic<numNodes, numControl>(Q_full, R_full));

//     shared_ptr<ct::optcon::TermQuadratic<numNodes, numControl>> finCost(
// 	  	new ct::optcon::TermQuadratic<numNodes, numControl>(Qf_full, R_full));

//     shared_ptr<ct::optcon::CostFunctionQuadratic<numNodes, numControl>> quadraticCost(
// 	  	new ct::optcon::CostFunctionAnalytical<numNodes, numControl>());

//     quadraticCost->addIntermediateTerm(intermCost);
//     quadraticCost->addFinalTerm(finCost);

//     std::chrono::time_point<std::chrono::system_clock> start, end;
//     start = std::chrono::system_clock::now(); 

//     ct::optcon::FHDTLQR<numNodes, numControl> lqrSolver(quadraticCost);
//     ct::core::FeedbackArray<numNodes, numControl> K_ct;

//     ct::core::StateVectorArray<numNodes> x_ref_init(numTimeSteps, 
//     ct::core::StateVector<numNodes>::Zero() );

//     ct::core::ControlVectorArray<numControl> u0_ff(numTimeSteps-1, ct::core::ControlVector<numControl>::Zero());

//     ct::core::StateMatrix<numNodes> A_ct(A_full);
//     ct::core::StateMatrixArray<numNodes> A_ct_vec(numTimeSteps-1, A_ct);
//     ct::core::StateControlMatrix<numNodes, numControl> B_ct(B_full);
//     ct::core::StateControlMatrixArray<numNodes, numControl> B_ct_vec(numTimeSteps-1, B_ct);

//     lqrSolver.designController( x_ref_init, u0_ff, A_ct_vec, B_ct_vec, p.dt, K_ct);
//     end = std::chrono::system_clock::now();
//     std::chrono::duration<double> elapsed_seconds = end - start; 
//     std::cout << "elapsed time for Control Toolbox is: " << elapsed_seconds.count() << "s\n";

//     SolverOutput ctSoln(numNodes, numTimeSteps, numControl);
//     ctSoln.runtime = elapsed_seconds.count();
//     ctSoln.solverType=2;

//     X_stacked.col(0) = X_init;
//     MatrixXd cost_ct = MatrixXd::Zero(1,1);+++
//     for(int i1=0; i1<numTimeSteps-1; i1++){

//         U_stacked.col(i1) = K_ct[i1] * X_stacked.col(i1);
//         X_stacked.col(i1+1) = A_full * X_stacked.col(i1) + B_full * U_stacked.col(i1);

//         cost_ct += X_stacked.col(i1).transpose() * Q_full * X_stacked.col(i1);
//         cost_ct += U_stacked.col(i1).transpose() * R_full * U_stacked.col(i1);		

//     }
//     cost_ct += X_stacked.col(numTimeSteps-1).transpose() * Qf_full * X_stacked.col(numTimeSteps-1);
//     ctSoln.cost = cost_ct(0,0);  
//     cout<<"Cost for Control Toolbox is "<<cost_ct<<endl;

//     ctSoln.controls = U_stacked;
//     matrixToOutput(ctSoln, X_stacked);

//     return ctSoln;
// }

// Ricatti Equation To Solve LQR Problem
SolverOutput FGSolver::DARESolve()
{
    std::cout << "Entered DARE Solve" << std::endl;

    vector<MatrixXd> P_val(numTimeSteps, Qf_full);
    MatrixXd X_lqr = MatrixXd::Zero(numNodes, numTimeSteps);
    MatrixXd U_lqr = MatrixXd::Zero(numControl, numTimeSteps-1);
    MatrixXd cost_lqr = MatrixXd::Zero(1,1);

    SolverOutput lqrSoln(numNodes, numTimeSteps, numControl);
    lqrSoln.solverType = 3;

    std::cout << "Starting Timer" << std::endl;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Backward Pass With Ricatti
    for (int i=numTimeSteps-1; i>0; i--) {
        P_val[i-1] = Q_full + A_full.transpose()*P_val[i]*A_full - 
        A_full.transpose()*P_val[i]*B_full*(R_full + B_full.transpose()*
        P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);
    }

    std::cout << "Check 1" << std::endl;

    // Forward Pass
    X_lqr.col(0) = X_init;

    for (int i=0; i<numTimeSteps-1; i++) {

        // std::cout << "Check a" << std::endl;
        // MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

        // std::cout << "Check b" << std::endl;
        // U_lqr.col(i) = k_mat * X_lqr.col(i);

        // std::cout << "Check b2" << std::endl;
        // X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

        // std::cout << "Check c" << std::endl; 
        // cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
        // cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);

		MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full)
			.colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

        // std::cout << k_mat << std::endl;

        // std::cout << "\n" << std::endl;

        // std::cout << X_lqr.col(i) << std::endl;

		U_lqr.col(i) = k_mat * X_lqr.col(i);
		X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

	 	cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
	 	cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);


    }

    std::cout << "Check 2" << std::endl;

    cost_lqr += X_lqr.col(numTimeSteps-1).transpose() * Qf_full * X_lqr.col(numTimeSteps-1);

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsedTime = end - start;

    std::cout << "The DARE Solution Took " << elapsedTime.count() << " seconds\n" << std::endl;

    std::cout << "DARE Cost: " << cost_lqr(0,0) << std::endl;

    lqrSoln.cost = cost_lqr(0,0);
    lqrSoln.controls = U_lqr;
    lqrSoln.runtime = elapsedTime.count();

    std::cout << "before matrix out" << std::endl;

    matrixToOutput(lqrSoln, X_lqr);

    std::cout << "after matrix out" << std::endl;

    return lqrSoln;
}

// Data Manipulation
void FGSolver::matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat)
{
    for (int i1=0; i1 < numTimeSteps; i1++){

        for (int i2=0; i2 < numNodes; i2++){
            soln.states(i2, i1) = (gtsam::Vector(1) << X_mat(i2, i1)).finished();
        }
    }

    return;

}