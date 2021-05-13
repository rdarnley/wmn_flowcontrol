// lqr.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: File contains many different versions of LQR solvers

#include "lqr.h"

LqrSolver::LqrSolver(YAML::Node lqr_info){

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

void LqrSolver::CreateLqrFg(WirelessNetwork& network){

    int state_space = 2;

    // Set Start And Goal States
    prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
    final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final; 

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
                
                    std::cout << "Adding Node " << idx << " to Node " << node_id << " dynamics" << std::endl;
                    std::cout << "Distance To Sink : " << distance_sink << std::endl;
                    std::cout << "Longer Distance : " << network.map_of_nodes[idx].distance_to_sink << std::endl;
                
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

    Q_node = gtsam::Vector(state_space); Q_node << state_cost, state_cost;
    Qf_node = gtsam::Vector(state_space); Qf_node << statef_cost, statef_cost;
    R_node = gtsam::Vector(1); R_node << control_cost;

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

    //
    graph.print();
    //

    return;

} // end CreateLqrFg()

void LqrSolver::SolveLqrFg(WirelessNetwork& network){

    std::cout << "Entered SolveLqrFg" << std::endl;

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    VectorValues resultLqr = graph.optimize();

    // End Timer
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = end - start;

    SolverOutput fgSoln(10, num_timesteps, 10);
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

                cost += pow(fgSoln.states(node_id-1, i)(0),2.)*Q_node(0) + pow(fgSoln.states(node_id-1, i)(1), 2.)*Q_node(1);

                if(controllable){
                    fgSoln.controls(node_id-1, i) = resultLqr.at(U[node_id][i])(0);
                    cost += pow(fgSoln.controls(node_id-1, i),2.)*R_node(0);
                } 

            } else {
                cost += pow(fgSoln.states(node_id-1, i)(0),2.)*Qf_node(0) + pow(fgSoln.states(node_id-1,i)(1),2.)*Qf_node(1);
            }

        }

    }

    fgSoln.cost = cost;

    std::cout << "Cost : " << cost << std::endl;

    return;

} // end SolveLqrFg()