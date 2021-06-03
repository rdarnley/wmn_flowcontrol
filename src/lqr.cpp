// lqr.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: File contains many different versions of LQR solvers

#include "lqr.h"

LqrSolver::LqrSolver(YAML::Node lqr_info){ //, WirelessNetwork network_){

    // network = network_;

    num_timesteps = lqr_info["num_timesteps"].as<int>();
    timestep = lqr_info["timestep"].as<double>();

    state_cost = lqr_info["state_cost"].as<double>();
    statef_cost = lqr_info["statef_cost"].as<double>();
    control_cost = lqr_info["control_cost"].as<double>();

    queue_init = lqr_info["queue_init"].as<int>();
    rate_init = lqr_info["rate_init"].as<double>();
    queue_final = lqr_info["queue_final"].as<int>();
    rate_final = lqr_info["rate_final"].as<double>();

} // end Constructor

void LqrSolver::CreateLqrFg(){

    int state_space = 2;

    // Set Start And Goal States
    prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
    final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final; 

    prior_noise = noiseModel::Constrained::All(state_space);
    dyn_noise = noiseModel::Constrained::All(state_space);

    // Initialize Factor Graph
    graph = GaussianFactorGraph();

    // Create Keys To Identify Nodes
    for (auto& n : network.map_of_nodes){
        int node_id = n.first;
        bool controllable = n.second.controllable;
        for(int i=0; i<num_timesteps-1; i++){
            X[node_id].push_back(gtsam::LabeledSymbol('x', node_id, i));
            if(controllable){
                U[node_id].push_back(gtsam::LabeledSymbol('u', node_id, i));
            }
        }
        X[node_id].push_back(gtsam::LabeledSymbol('x', node_id, num_timesteps-1));
    }

    // Add Priors
    for (auto& n : network.map_of_nodes){
        int node_id = n.first;
        graph.add(X[node_id][0], MatrixXd::Identity(2,2), prior_state, prior_noise);
    }

    MatrixXd a_end = MatrixXd::Zero(2,2);   a_end << 1, -1, 0, 1;
    MatrixXd a = MatrixXd::Zero(2,2);       a << 0, 1, 0, 0;
    MatrixXd b = MatrixXd::Zero(2,1);       b << 0, 1;
    
    // Iterate Through Nodes
    for (auto& n : network.map_of_nodes){

        int node_id = n.first;
        float distance_sink = n.second.distance_to_sink;
        bool controllable = n.second.controllable;

        // Iterate Through Time
        for(int i=0; i<num_timesteps-1; i++){

            std::vector<int> adjacent_nodes = n.second.neighbor_ids;

            std::vector<std::pair<Key, MatrixXd>> terms;

            // Iterate Through Neighbors
            for(int idx : adjacent_nodes){

                // Verify Node Is "Farther" From Sink Than Current Node
                if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
                    terms.push_back(std::make_pair(X[idx][i], a));
                
                }
            }

            terms.push_back(std::make_pair(X[node_id][i], a_end));

            if(controllable){
                terms.push_back(std::make_pair(U[node_id][i], b));
            }

            terms.push_back(std::make_pair(X[node_id][i+1], -gtsam::Matrix::Identity(2,2)));

            graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);

        } // end temporal loop
    } // end spatial loop

    Q_node = gtsam::Vector(state_space); Q_node << 1.0/state_cost, 1.0/state_cost;
    Qf_node = gtsam::Vector(state_space); Qf_node << 1.0/statef_cost, 1.0/statef_cost;
    R_node = gtsam::Vector(1); R_node << 1.0/control_cost;

    noiseModel::Diagonal::shared_ptr state_cost_node = noiseModel::Diagonal::Variances(Q_node);
    noiseModel::Diagonal::shared_ptr statef_cost_node = noiseModel::Diagonal::Variances(Qf_node);
    noiseModel::Diagonal::shared_ptr control_cost_node = noiseModel::Diagonal::Variances(R_node);

    // Add State + Control Costs
    for(auto& n : network.map_of_nodes){

        int node_id = n.first;
        bool controllable = n.second.controllable;

        for(int j = 0; j<num_timesteps-1; j++){
            graph.add(X[node_id][j], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);
            if(controllable){
                graph.add(U[node_id][j], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
            }
        }
        graph.add(X[node_id][num_timesteps-1], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);

    }

    return;

} // end CreateLqrFg()

SolverOutput LqrSolver::SolveLqrFg(){

    std::cout << "Entered SolveLqrFg" << std::endl;

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    VectorValues resultLqr = graph.optimize();

    // End Timer
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end - start;

    SolverOutput fgSoln(network.num_nodes, num_timesteps, network.num_control);
    fgSoln.runtime = duration.count();
    fgSoln.solverType = "Factor Graph";
    fgSoln.num_timesteps = num_timesteps;

    double cost = 0.;

    for(auto& n : network.map_of_nodes){

        int node_id = n.first;
        bool controllable = n.second.controllable;

        for(int i = 0; i<num_timesteps; i++){

            fgSoln.states(node_id-1, i) = ( gtsam::Vector(2) << resultLqr.at(X[node_id][i])(0), resultLqr.at(X[node_id][i])(1) ).finished();

            if(i < num_timesteps - 1){

                cost += pow(fgSoln.states(node_id-1, i)(0),2.)*state_cost + pow(fgSoln.states(node_id-1, i)(1), 2.)*state_cost;

                if(controllable){
                    fgSoln.controls(node_id-1, i) = resultLqr.at(U[node_id][i])(0);
                    cost += pow(fgSoln.controls(node_id-1, i),2.)*control_cost;
                } 

            } else {
                cost += pow(fgSoln.states(node_id-1, i)(0),2.)*statef_cost + pow(fgSoln.states(node_id-1,i)(1),2.)*statef_cost;
            }

        }

    }

    fgSoln.cost = cost;

    return fgSoln;

} // end SolveLqrFg()

SolverOutput LqrSolver::LqrMatrix(){

    int state_space = 2;
    int num_nodes = network.num_nodes;
    int num_control = network.num_control;

    int control_counter = 0;

    // Initialize Full Matrices
    A_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    B_full = MatrixXd::Zero(state_space * num_nodes, num_control);
    Q_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    Qf_full = MatrixXd::Zero(state_space * num_nodes, state_space * num_nodes);
    R_full = MatrixXd::Zero(num_control, num_control);

    X_init = MatrixXd::Zero(state_space * num_nodes, 1);

    for(int i = 0; i < num_nodes; i++) {
        X_init(2*i, 0) = queue_init;
        X_init(2*i+1,0) = rate_init;
    }

    // Iterate Across Nodes
    for(auto& n : network.map_of_nodes){

        int node_id = n.first;
        int matrix_id = node_id - 1;
        bool controllable = n.second.controllable;
        int distance_sink = n.second.distance_to_sink;
        std::vector<int> adjacent_nodes = n.second.neighbor_ids;

        // Iterate Through Neighbors
        for(int idx : adjacent_nodes){

            // Verify Node Is "Farther" From Sink Than Current Node
            if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            
                int other_matrix_id = idx - 1;

                MatrixXd a = MatrixXd::Zero(1,1); a << 1;
                A_full.block<1,1>(state_space * matrix_id, state_space*other_matrix_id+1) = a;                            
            }
        }

        // Add Self-Dynamics
        MatrixXd a = MatrixXd::Zero(2,2); a << 1, -1, 0, 1;
        A_full.block<2,2>(2*matrix_id, 2*matrix_id) = a;

        // Add Control
        if(controllable){
            MatrixXd b = MatrixXd::Zero(1,1); b << 1;
            B_full.block<1,1>(2*matrix_id+1, control_counter) = b;
            
            R_full(control_counter, control_counter) = control_cost;

            control_counter++;
        }

        // Fill Q Matrix
        Q_full(2*matrix_id, 2*matrix_id) = state_cost;
        Q_full(2*matrix_id+1, 2*matrix_id+1) = state_cost;
        Qf_full(2*matrix_id, 2*matrix_id) = statef_cost;
        Qf_full(2*matrix_id+1, 2*matrix_id+1) = statef_cost;

    }

    // Solve

    vector<MatrixXd> P_val(num_timesteps, Qf_full);
    MatrixXd X_lqr = MatrixXd::Zero(state_space * num_nodes, num_timesteps);
    MatrixXd U_lqr = MatrixXd::Zero(num_control, num_timesteps-1);
    MatrixXd cost_lqr = MatrixXd::Zero(1,1);

    SolverOutput lqrSoln(num_nodes, num_timesteps, num_control);
    lqrSoln.solverType = "DARE";

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Backward Pass With Ricatti
    for (int i=num_timesteps-1; i>0; i--) {
        P_val[i-1] = Q_full + A_full.transpose()*P_val[i]*A_full - 
        A_full.transpose()*P_val[i]*B_full*(R_full + B_full.transpose()*
        P_val[i]*B_full).colPivHouseholderQr().solve(B_full.transpose()*P_val[i]*A_full);
    }

    // Forward Pass
    X_lqr.col(0) = X_init;

    for (int i=0; i<num_timesteps-1; i++) {

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

    cost_lqr += X_lqr.col(num_timesteps-1).transpose() * Qf_full * X_lqr.col(num_timesteps-1);

    end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsedTime = end - start;

    lqrSoln.cost = cost_lqr(0,0);
    lqrSoln.controls = U_lqr;
    lqrSoln.runtime = elapsedTime.count();

    return lqrSoln;

}

/////
