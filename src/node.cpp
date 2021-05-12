#include "node.h"

void WirelessNode::Mesh(std::map<int, WirelessNode> &map_of_nodes){

    std::cout << "Entered Mesh" << std::endl;

    // Current Node id
    int current_id = this->id;

    // Iterate through nodes
    for (auto& n : map_of_nodes){

        int node_id = n.first;
        WirelessNode other_node = n.second;

        if(current_id - node_id == -1 || current_id - node_id == 1){
            neighbor neigh;
            neigh.id = node_id;
            neigh.x = other_node.loc.x;
            neigh.y = other_node.loc.y;

            this->list_neighbors.push_back(neigh);

            neighbor neigh2;
            neigh2.id = current_id;
            neigh2.x = this->loc.x;
            neigh2.y = this->loc.y;

            n.second.list_neighbors.push_back(neigh2);
        }

    }

    return;
} // End Mesh()

void WirelessNode::CreateLocalFG(Environment env){

    std::cout << "Entered Create Local FG" << std::endl;

    // Parse Input
    queue_init = env.queue_init;
    rate_init = env.rate_init;
    queue_final = env.queue_final;
    rate_final = env.rate_final;

    // For Now All Controllable

    // Super Hardcoded //
    // int num_nodes = find number of neighbors + other direction criteria
    if (this->id == 1){
        num_nodes = 1;
    } else {
        num_nodes = 2;
    }
    num_control = 1;

    state_space = 2;
    state_space_total = num_nodes * state_space;

    // Create Constrained Noise Models
    prior_noise = gtsam::noiseModel::Constrained::All(state_space);
    dyn_noise = gtsam::noiseModel::Constrained::All(state_space);

    // Initialize Prior/Final States
    prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
    final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final;

    // Create Gaussian Factor Graph
    graph = gtsam::GaussianFactorGraph();

    vector<gtsam::Key> temp;
    X = vector<vector<gtsam::Key>>(num_nodes, temp);

    // Assigning all control right now
    vector<gtsam::Key> tempU;
    U = vector<vector<gtsam::Key>>(num_control, tempU);

    // Create Symbols For State
    for(int j=0; j<num_nodes; j++) {
        for(int i=0; i<num_timesteps; i++){
            X[j].push_back(gtsam::LabeledSymbol('x', j, i));
        }
    }

    // Create Symbols For Control
    for(int j=0; j<num_control; j++){
        for(int i=0; i<num_timesteps; i++){
            U[j].push_back(gtsam::LabeledSymbol('u', j, i));
        }
    }

    // Add Priors - Just do it for current node -- super hardcoded right now
    // Find Index of Key node
    node_index = num_nodes - 1;
    graph.add(X[node_index][0], MatrixXd::Identity(2,2), prior_state, prior_noise);
    // add priors from previous solver???

    MatrixXd a_end = MatrixXd::Zero(2,2);       a_end << 1, -1, 0, 1;
    MatrixXd a = MatrixXd::Zero(2,2);           a << 0, 1, 0, 0;
    MatrixXd b = MatrixXd::Zero(2,1);           b << 0, 1;

    // Iterate over time (not space)
    for(int i = 0; i < num_timesteps-1; i++){

        // Get Mesh Information - Hardcode for right now
        if(this->id == 1){
            graph.add(X[node_index][i], a_end, U[0][i], b, X[node_index][i+1], -gtsam::Matrix::Identity(2,2), gtsam::Matrix::Zero(2,1), dyn_noise);
        } else {
            vector<std::pair<gtsam::Key, MatrixXd>> terms;
            terms.push_back(std::make_pair(X[node_index-1][i], a));
            terms.push_back(std::make_pair(X[node_index][i], a_end));
            terms.push_back(std::make_pair(U[0][i], b));
            terms.push_back(std::make_pair(X[node_index][i+1], -gtsam::Matrix::Identity(2,2)));
            graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);
        }

    } // end temporal for loop

    Q_node = gtsam::Vector(state_space); Q_node << state_cost, state_cost;
    Qf_node = gtsam::Vector(state_space); Qf_node << statef_cost, statef_cost;

    R = gtsam::Vector(1); R << control_cost;

    gtsam::noiseModel::Diagonal::shared_ptr state_cost_node = gtsam::noiseModel::Diagonal::Variances(Q_node);
    gtsam::noiseModel::Diagonal::shared_ptr statef_cost_node = gtsam::noiseModel::Diagonal::Variances(Qf_node);
    gtsam::noiseModel::Diagonal::shared_ptr control_cost_node = gtsam::noiseModel::Diagonal::Variances(R);

    // Add Costs - Just need to do it on eliminating node?? -- combine with above?
    for(int i=0; i<num_timesteps; i++){
        if(i == num_timesteps-1){
            graph.add(X[node_index][i], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);
        } else {
            graph.add(X[node_index][i], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);
        }

        // Control costs
        graph.add(U[0][i], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
    }

    return;

} // End CreateLocalFG()

void WirelessNode::ConvertFG(){

    std::cout << "Entered Convert FG" << std::endl;

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Create Order (Just this node)
    for(int i1=num_timesteps-1; i1>-1; i1--){
        order.push_back(X[node_index][i1]);
    }

    // Perform Partial FG Solving --> <Gaussian Bayes Net, Gaussian Factor Graph>
    pairing = graph.eliminatePartialSequential(order);

    return;

} // End ConvertFG()