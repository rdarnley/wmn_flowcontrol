#include "solver.h"
#include "centralized.h"

LqrSolver::LqrSolver(){

} // end LqrSolver Constructor

void LqrSolver::CreateScenario(int num_nodes_, int num_control_, int state_space_total_, int queue_init_, int rate_init_, int queue_final_, int rate_final_) {
    
    std::cout << "Entered CreateScenario" << std::endl;

    // Parse Input
    queue_init = queue_init_;
    rate_init = rate_init_;
    queue_final = queue_final_;
    rate_final = rate_final_;

    num_nodes = num_nodes_;
    num_control = num_control_;

    state_space = 2;
    state_space_total = state_space_total_;

    // Create Constrained Noise Models
    prior_noise = noiseModel::Constrained::All(state_space);
    dyn_noise = noiseModel::Constrained::All(state_space);

    // Initialize Prior/Final
    prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
    final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final;

    // Initialize Full Matrices
    A_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    B_full = MatrixXd::Zero(state_space * num_nodes, num_control);
    Q_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    Qf_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    R_full = MatrixXd::Zero(num_control, num_control);

    // Assign Which Nodes Are Drivers ??? Others Fixed Rate ???
    driver_idx = unordered_map<int, int>();
    for(int i=0; i<num_control; i++){
        driver_idx[i*num_nodes/num_control] = i;
    } // end driver_idx for loop

    // Create Initial States
    X_init = MatrixXd::Zero(state_space * num_nodes, 1);

    for(int i = 0; i < num_nodes; i++) {
        X_init(2*i, 0) = prior_state(0);
        X_init(2*i+1,0) = prior_state(1);
    } // end Initialization for loop

    // Initialize Factor Graph
    graph = GaussianFactorGraph();

    vector<Key> temp;
    X = vector<vector<Key>>(num_nodes, temp);

    vector<Key> tempU;
    U = vector<vector<Key>>(num_control, tempU);

    for(int j=0; j<num_nodes; j++) {
        for(int i=0; i<num_timesteps; i++) {
            X[j].push_back(gtsam::LabeledSymbol('x', j, i));
        } // end inner for loop
    } // end outer for loop

    for(int j=0; j<num_control; j++) {
        for(int i=0; i<num_timesteps; i++) {
            U[j].push_back(gtsam::LabeledSymbol('u', j, i));
        } // end inner for loop
    } // end outer for loop

    for (int i=0; i<num_nodes; i++) {
        graph.add(X[i][0], MatrixXd::Identity(2,2), prior_state, prior_noise);
    } // end prior factor for loop

    MatrixXd positive_one = MatrixXd::Zero(1,1);
    positive_one << 1;

    MatrixXd control_cost_mat = MatrixXd::Zero(1,1);
    control_cost_mat << control_cost;

    for(int i2 = 0; i2 < num_nodes; i2++) {

        // Check if node is active or passive
        bool applyControl = false; int controlIdx = -1; 
        bool lastApplyControl = false; int lastControlIdx = -1;

        if(driver_idx.find(i2) != driver_idx.end()){
            applyControl = true;
            controlIdx = driver_idx[i2];
        } // end apply control if statement

        // Fill A Matrix
        if (i2 == 0){
            MatrixXd a = MatrixXd::Zero(1,2);   a << 1, -1;
            A_full.block<1,2>(0,0) = a;
        } else {
            MatrixXd a = MatrixXd::Zero(1,3);   a << 1, 1, -1;
            A_full.block<1,3>(2*i2, 2*i2-1) = a;
        }

        A_full.block<1,1>(2*i2+1, 2*i2+1) = positive_one;

        // Fill B Matrix
        if(applyControl){
            B_full.block<1,1>(2*i2+1, controlIdx) = positive_one;

            // R_full.block<1,1>(controlIdx, controlIdx) = control_cost_mat;
            R_full(controlIdx, controlIdx) = control_cost;
        }

        // Fill Q Matrix
        Q_full(2*i2, 2*i2) = state_cost;
        Q_full(2*i2+1, 2*i2+1) = state_cost;
        Qf_full(2*i2, 2*i2) = statef_cost;
        Qf_full(2*i2+1, 2*i2+1) = statef_cost;

        MatrixXd a_end = MatrixXd::Zero(2,2);   a_end << 1, -1, 0, 1;
        MatrixXd a = MatrixXd::Zero(2,2);       a << 0, 1, 0, 0;
        MatrixXd b = MatrixXd::Zero(2,1);       b << 0, 1;

        // Iterate Through TimeSteps (For FG Method)
        for (int i1=0; i1<num_timesteps-1; i1++) {

            if(i2 == 0){
                if(applyControl){
                    graph.add(X[i2][i1], a_end, U[controlIdx][i1], b, X[i2][i1+1], -gtsam::Matrix::Identity(2,2), gtsam::Matrix::Zero(2,1), dyn_noise);
                } else if(!applyControl) {
                    graph.add(X[i2][i1], a_end, X[i2][i1+1], -gtsam::Matrix::Identity(2,2), gtsam::Matrix::Zero(2,1), dyn_noise);
                } else {
                    std::cout << "Error: Incorrect Factor 1" << std::endl;
                }
            } else {
                if(applyControl){
                    vector<std::pair<Key, MatrixXd>> terms;
                    terms.push_back(std::make_pair(X[i2-1][i1], a));
                    terms.push_back(std::make_pair(X[i2][i1], a_end));
                    terms.push_back(std::make_pair(U[controlIdx][i1], b));
                    terms.push_back(std::make_pair(X[i2][i1+1], -gtsam::Matrix::Identity(2,2)));
                    // graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);
                } else if(!applyControl) {
                    graph.add(X[i2-1][i1], a, X[i2][i1], a_end, X[i2][i1+1], -gtsam::Matrix::Identity(2,2), gtsam::Matrix::Zero(2,1), dyn_noise);
                } else {
                    std::cout << "Error: Incorrect Factor 2" << std::endl;
                }
            }

        } // end inner/temporal for loop
    } // end outer for loop

    Q_node = gtsam::Vector(state_space); Q_node << state_cost, state_cost;
    Qf_node = gtsam::Vector(state_space); Qf_node << statef_cost, statef_cost;

    R = gtsam::Vector(1); R << control_cost;

    qmat_node = Q_node.array().matrix().asDiagonal();
    qfmat_node = Qf_node.array().matrix().asDiagonal();
    rmat = R.array().matrix().asDiagonal();

    // auto state_cost_node = noiseModel::Gaussian::Information(qmat_node);
    // auto statef_cost_node = noiseModel::Gaussian::Information(qfmat_node);
    // auto control_cost_node = noiseModel::Gaussian::Information(rmat);

    noiseModel::Diagonal::shared_ptr state_cost_node = noiseModel::Diagonal::Sigmas(Q_node);
    noiseModel::Diagonal::shared_ptr statef_cost_node = noiseModel::Diagonal::Sigmas(Qf_node);
    noiseModel::Diagonal::shared_ptr control_cost_node = noiseModel::Diagonal::Sigmas(R);

    for(int i=0; i<num_nodes; i++){
        for(int j=0; j<num_timesteps; j++) {
            if(j==num_timesteps - 1){
                // graph.add(GaussianFactor<gtsam::Vector>(X[i][j], final_state, statef_cost_node));
                graph.add(X[i][j], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);

            } else {
                // graph.add(GaussianFactor<gtsam::Vector>(X[i][j], final_state, state_cost_node));
                graph.add(X[i][j], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);
       
            }
        }
    }

    for(int i=0; i<num_control; i++) {
        for(int j=0; j<num_timesteps-1; j++) {
            graph.add(U[i][j], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
        }
    }
    return;   
}

// Solve Factor Graph With Different Variable Orderings
SolverOutput LqrSolver::SolveFG(int orderingType) {
    
    std::cout << "Entered FG Solve" << std::endl;

    // Create Some Sort of Order
    std::cout << "Using COLAMD Ordering" << std::endl;
    // order = graph.orderingCOLAMD();

    // Create Instance of Optimizer
    // GaussNewtonOptimizer optimizer(graph, initialEstimate, order);

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Optimize Factor Graph
    // Values resultLQR = optimizer.optimize();
    VectorValues resultLQR = graph.optimize();

    // End Timer
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end - start;

    SolverOutput fgSoln(num_nodes, num_timesteps, num_control);
    fgSoln.runtime = duration.count();
    fgSoln.solverType = "Factor Graph";
    fgSoln.num_timesteps = num_timesteps;
    // fgSoln.orderingType = ordering

    // Calculate Cost of Factor Graph
    double cost_fg = 0.;

    for(int i1 = 0; i1 < num_timesteps; i1++) {

        // Add Costs For States
        for(int i2=0; i2 < num_nodes; i2++) {

    //         fgSoln.states(i2, i1) = ( gtsam::Vector(1) << resultLQR.at<VectorXd>(X[i2][i1])(0) ).finished();
            fgSoln.states(i2,i1) = ( gtsam::Vector(1) << resultLQR.at(X[i2][i1])(0) ).finished();

            if (i1 < num_timesteps - 1){
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*Q_node(0);
            } else {
                cost_fg += pow(fgSoln.states(i2,i1)(0),2.)*Qf_node(0);
            }

        }

        // Add Costs For Controls
        if(i1 < num_timesteps - 1) {
            for(int i4=0; i4<num_control; i4++) {

                fgSoln.controls(i4, i1) = resultLQR.at(U[i4][i1])(0);
                cost_fg += pow(fgSoln.controls(i4,i1),2.)*R_full(i4,i4);
            } 
        }
    }     

    fgSoln.cost = cost_fg;

    // output.push_back(fgSoln);
    // PrintResultTerminal(fgSoln);

    return fgSoln;
}


// SolverOutput FGSolver::FGSim(const SolverOutput& fgSoln)
// {
//     MatrixXd X_fg = MatrixXd::Zero(num_nodes*numStates, num_timesteps);

//     MatrixXd cost_fgsim = MatrixXd::Zero(1,1);

//     SolverOutput fgsimSoln = fgSoln;
//     fgsimSoln.solverType=1;

//     // Forward Simulation
//     X_fg.col(0) = X_init;
//     for (int i = 0; i < num_timesteps - 1; i++){

//         X_fg.col(i+1) = A_full * X_fg.col(i) + B_full * fgSoln.controls.col(i);

//         cost_fgsim += X_fg.col(i).transpose() * Q_full * X_fg.col(i);
//         cost_fgsim += fgSoln.controls.col(i).transpose()*R_full*fgSoln.controls.col(i);
//     }

//     cost_fgsim += X_fg.col(num_timesteps-1).transpose() * Qf_full * X_fg.col(num_timesteps-1);

//     std::cout << "Cost for FG Sim is " << cost_fgsim << std::endl;

//     fgsimSoln.cost = cost_fgsim(0,0);

//     matrixToOutput(fgsimSoln, X_fg);

//     return fgsimSoln;
// }

// // Ricatti Equation To Solve LQR Problem
// SolverOutput LqrSolver::SolveDARE()
// {
//     std::cout << "Entered DARE Solve" << std::endl;

//     vector<MatrixXd> P_val(num_timesteps, Qf_full);
//     MatrixXd X_lqr = MatrixXd::Zero(state_space * num_nodes, num_timesteps);
//     MatrixXd U_lqr = MatrixXd::Zero(num_control, num_timesteps-1);
//     MatrixXd cost_lqr = MatrixXd::Zero(1,1);

//     SolverOutput lqrSoln(num_nodes, num_timesteps, num_control);
//     lqrSoln.solverType = "DARE";

//     std::chrono::time_point<std::chrono::system_clock> start, end;
//     start = std::chrono::system_clock::now();

//     // Backward Pass With Ricatti
//     for (int i=num_timesteps-1; i>0; i--) {
//         P_val[i-1] = Q_full + A_full.transpose()*P_val[i]*A_full - 
//         A_full.transpose()*P_val[i]*B_full*(R_full + B_full.transpose()*
//         P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);
//     }

//     // Forward Pass
//     X_lqr.col(0) = X_init;

//     for (int i=0; i<num_timesteps-1; i++) {

//         // std::cout << "Check a" << std::endl;
//         // MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

//         // std::cout << "Check b" << std::endl;
//         // U_lqr.col(i) = k_mat * X_lqr.col(i);

//         // std::cout << "Check b2" << std::endl;
//         // X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

//         // std::cout << "Check c" << std::endl; 
//         // cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
//         // cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);

// 		MatrixXd k_mat = -1.0*(R_full + B_full.transpose()*P_val[i]*B_full)
// 			.colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);

//         // std::cout << k_mat << std::endl;

//         // std::cout << "\n" << std::endl;

//         // std::cout << X_lqr.col(i) << std::endl;

// 		U_lqr.col(i) = k_mat * X_lqr.col(i);
// 		X_lqr.col(i+1) = A_full * X_lqr.col(i) + B_full * U_lqr.col(i);

// 	 	cost_lqr += X_lqr.col(i).transpose() * Q_full * X_lqr.col(i);
// 	 	cost_lqr += U_lqr.col(i).transpose() * R_full * U_lqr.col(i);


//     }

//     cost_lqr += X_lqr.col(num_timesteps-1).transpose() * Qf_full * X_lqr.col(num_timesteps-1);

//     end = std::chrono::system_clock::now();

//     std::chrono::duration<double> elapsedTime = end - start;

//     lqrSoln.cost = cost_lqr(0,0);
//     lqrSoln.controls = U_lqr;
//     lqrSoln.runtime = elapsedTime.count();

//     // PrintResultTerminal(lqrSoln);

//     // output.push_back(lqrSoln);

//     matrixToOutput(lqrSoln, X_lqr);

//     return lqrSoln;
// }

// // Data Manipulation
// void LqrSolver::matrixToOutput(SolverOutput& soln, const MatrixXd& X_mat)
// {
//     for (int i1=0; i1 < num_timesteps; i1++){

//         for (int i2=0; i2 < num_nodes; i2++){
//             soln.states(i2, i1) = (gtsam::Vector(1) << X_mat(i2, i1)).finished();
//         }
//     }

//     return;

// }