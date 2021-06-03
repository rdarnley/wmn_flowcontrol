// simulator.cpp

#include "simulator.h"

void DecentralizedSimulator::ForwardPass(Environment env, WirelessNetwork& network, int curr_id, int last_id){

    // Create Local Factor Graph
    this->CreateLocalFg(env, network, curr_id, last_id);

    // Partially Solve FG --> Convert To Bayes Net
    this->ConvertLocalFg(env, network, curr_id, last_id);

    // Recurse Forward Pass To Next Node (i.e. Find next, transmit, etc)
    // Or Enter BackwardPass

    // Current Node
    WirelessNode curr_node = network.map_of_nodes[curr_id];
    std::vector<int> neighbor_nodes = curr_node.neighbor_ids;
    double distance_sink = curr_node.distance_to_sink;

    bool last = true;

    for(int idx: neighbor_nodes){
        if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            last = false;
        }
    }

    if(last){
        std::cout << "Done" << std::endl;
        this->BackwardPass(env, network, curr_id, -1);
        return;
    } else {
        // Propagation Environment
        // Identity NExt node
        this->ForwardPass(env, network, curr_id-1, curr_id);
    }

    return;

} // end ForwardPass()



void DecentralizedSimulator::BackwardPass(Environment env, WirelessNetwork& network, int curr_id, int last_id){

    this->SolveLocalBayesNet(env, network, curr_id, last_id);

    // Get Current Node
    WirelessNode curr_node = network.map_of_nodes[curr_id];

    // Determine if last node or not
    std::vector<int> neighbor_nodes = curr_node.neighbor_ids;
    double distance_sink = curr_node.distance_to_sink;

    bool first = true;

    for(int idx : neighbor_nodes){
        if(distance_sink > network.map_of_nodes[idx].distance_to_sink){
            first = false;
        }
    }

    if(first){
        std::cout << "Done Again" << std::endl;
        this->DisplayResults(env, network);
        return;
    } else {
        // Add propagation environment
        // Identify next node
        this->BackwardPass(env, network, curr_id+1, curr_id);
    }

    return;

} // end BackwardPass()



// void DecentralizedSimulator::NodeSelector(WirelessNetwork& network){

//     std::cout << "Entered NodeSelector" << std::endl;

//     // Need to enumerate all conditions before doing this




// } // end NodeSelector()



void DecentralizedSimulator::CreateLocalFg(Environment env, WirelessNetwork& network, int curr_id, int last_id){

    std::cout << "Entered CreateLocalFg" << std::endl;

    int state_space = 2;

    // Start and Goal States
    gtsam::Vector prior_state = gtsam::Vector(state_space); prior_state << env.queue_init, env.rate_init;
    gtsam::Vector final_state = gtsam::Vector(state_space); final_state << env.queue_final, env.rate_final; 

    // Noise Models
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Constrained::All(state_space);
    gtsam::noiseModel::Diagonal::shared_ptr dyn_noise = gtsam::noiseModel::Constrained::All(state_space);

    // Initialize Factor Graph
    gtsam::GaussianFactorGraph graph = gtsam::GaussianFactorGraph();

    // Get Current Solving Node
    WirelessNode curr_node = network.map_of_nodes[curr_id];
    std::vector<int> neighbor_nodes = curr_node.neighbor_ids;
    double distance_sink = curr_node.distance_to_sink;
    bool controllable = curr_node.controllable;

    // Get Previously Solved Node
    WirelessNode prev_node;
    if(last_id != -1){
        prev_node = network.map_of_nodes[last_id];
    }

    // Initialize Keys
    std::map<int, vector<gtsam::Key>> X;
    std::map<int, vector<gtsam::Key>> U;

    // Create State Keys For Neighbor Nodes
    for(int idx : neighbor_nodes){
        if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            for(int i1=0; i1<env.num_timesteps-1; i1++){
                X[idx].push_back(gtsam::LabeledSymbol('x', idx, i1));
            }
        }
    }

    // Create State Keys For This Node
    for(int i1=0; i1<env.num_timesteps; i1++){
        X[curr_id].push_back(gtsam::LabeledSymbol('x', curr_id, i1));
    }

    // Create Control Keys (If Applicable) For This Node
    if(controllable){
        for(int i1=0; i1<env.num_timesteps-1; i1++){
            U[curr_id].push_back(gtsam::LabeledSymbol('u', curr_id, i1));
        }
    }

    // Add Prior To This Node
    graph.add(X[curr_id][0], MatrixXd::Identity(2,2), prior_state, prior_noise);

    // Tie Down Other Nodes With Priors
    for(int idx : neighbor_nodes){
        if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            for(int i1=0; i1<env.num_timesteps-1; i1++){
                graph.add(X[idx][i1], MatrixXd::Identity(2,2), prior_state, prior_noise);
            }
        }
    }

    // Add Shared Factor From prev_node (If Applicable)
    if(prev_node.id != -1){
        // std::cout << "Hello" << std::endl;
        graph.add(prev_node.shared_factor);
    }

    //
    // Abstract Dynamics???
    //

    // Iterate Through Time + Neighbors
    // Create Dynamical/Hard Constraint Nodes
    MatrixXd a_end = MatrixXd::Zero(2,2);   a_end << 1, -1, 0, 1;
    MatrixXd a = MatrixXd::Zero(2,2);       a << 0, 1, 0, 0;
    MatrixXd b = MatrixXd::Zero(2,1);       b << 0, 1;

    for(int i1=0; i1<env.num_timesteps-1; i1++){

        std::vector<std::pair<gtsam::Key, MatrixXd>> terms;

        for(int idx : neighbor_nodes){
            // Verify Neighbor Node Is "Farther" From Sink Than Current Node
            if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
                terms.push_back(std::make_pair(X[idx][i1], a));
            } 
        }

        // Add Isolated Dynamics Term
        terms.push_back(std::make_pair(X[curr_id][i1], a_end));

        // Add Control (If Applicable)
        if(controllable){
            terms.push_back(std::make_pair(U[curr_id][i1], b));
        }

        // Add Next Timestep State (i.e. RHS)
        terms.push_back(std::make_pair(X[curr_id][i1+1], -gtsam::Matrix::Identity(2,2)));

        // Add Factor To Graph
        graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);

    } // end temporal loop

    // Create Cost/Minimization Vectors
    gtsam::Vector Q_vector = gtsam::Vector(state_space);    Q_vector << 1.0/env.state_cost, 1.0/env.state_cost;
    gtsam::Vector Qf_vector = gtsam::Vector(state_space);   Qf_vector << 1.0/env.statef_cost, 1.0/env.statef_cost;
    gtsam::Vector R_vector = gtsam::Vector(1);              R_vector << 1.0/env.control_cost;

    // Create Cost/Minimization Nodes
    gtsam::noiseModel::Diagonal::shared_ptr Q_node = gtsam::noiseModel::Diagonal::Variances(Q_vector);
    gtsam::noiseModel::Diagonal::shared_ptr Qf_node = gtsam::noiseModel::Diagonal::Variances(Qf_vector);
    gtsam::noiseModel::Diagonal::shared_ptr R_node = gtsam::noiseModel::Diagonal::Variances(R_vector);

    // Add Costs To Graph
    for(int i1=0; i1<env.num_timesteps-1; i1++){
        graph.add(X[curr_id][i1], gtsam::Matrix::Identity(2,2), final_state, Q_node);
        if(controllable){
            graph.add(U[curr_id][i1], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), R_node);
        }
    }
    graph.add(X[curr_id][env.num_timesteps-1], gtsam::Matrix::Identity(2,2), final_state, Qf_node);

    std::cout << "\n\n\n\n" << std::endl;

    graph.print();

    // Copy Appropriate Material To Network.Node Instance
    network.map_of_nodes[curr_id].graph = graph;
    network.map_of_nodes[curr_id].X = X;
    network.map_of_nodes[curr_id].U = U;

    return;

} // end CreateLocalFg()



void DecentralizedSimulator::ConvertLocalFg(Environment env, WirelessNetwork& network, int curr_id, int last_id){

    std::cout << "Entered ConvertLocalFg" << std::endl;

    // Get Current Node
    WirelessNode curr_node = network.map_of_nodes[curr_id];
    bool controllable = curr_node.controllable;
    std::vector<int> neighbor_nodes = curr_node.neighbor_ids;
    double distance_sink = curr_node.distance_to_sink;

    // Create Variable Elimination Order
    gtsam::Ordering order;
    for(int i1=env.num_timesteps-1; i1>=0; i1--){
        if(i1 != env.num_timesteps-1 && controllable){
            order.push_back(curr_node.U[curr_id][i1]);
        }
        order.push_back(curr_node.X[curr_id][i1]);
    }

    // Perform Partial Solving
    network.map_of_nodes[curr_id].pairing = curr_node.graph.eliminatePartialSequential(order);

    size_t fg_size = network.map_of_nodes[curr_id].pairing.second->size();

    bool last = true;
    for(int idx : neighbor_nodes){
        if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            last = false;
        }
    }

    gtsam::GaussianFactor::shared_ptr shared_factor;

    if(!last){
        shared_factor = network.map_of_nodes[curr_id].pairing.second->at(fg_size-1);
        network.map_of_nodes[curr_id].shared_factor = shared_factor;
    }

    return;

} // end ConvertLocalFg()



void DecentralizedSimulator::SolveLocalBayesNet(Environment env, WirelessNetwork& network, int curr_id, int last_id){

    std::cout << "Entered SolveLocalBayesNet" << std::endl;

    std::cout << "\n\n\n\n" << std::endl;

    network.map_of_nodes[curr_id].pairing.second->print();

    // Current Node
    WirelessNode curr_node = network.map_of_nodes[curr_id];
    std::vector<int> neighbor_nodes = curr_node.neighbor_ids;
    double distance_sink = curr_node.distance_to_sink;

    // Previous Node
    WirelessNode prev_node;
    if(last_id != -1){
        prev_node = network.map_of_nodes[last_id];
    }

    std::cout << " 1 " << std::endl;

    gtsam::VectorValues bayes_sol;
    if(last_id != -1){
        prev_node.msg_bayes.print();
        bayes_sol = network.map_of_nodes[curr_id].pairing.first->optimize(prev_node.msg_bayes);
    } else {
        bayes_sol = network.map_of_nodes[curr_id].pairing.first->optimize();

        std::cout << bayes_sol.size() << std::endl;
    }

    std::cout << " 2" << std::endl;

    gtsam::VectorValues msg_bayes;

    // Extract Message To Share
    for(int i1=0; i1<env.num_timesteps-1; i1++){
        msg_bayes.insert(std::make_pair(curr_node.X[curr_id][i1], bayes_sol.at(curr_node.X[curr_id][i1])));
    }
    std::cout << "ddkdkd" << std::endl;
    msg_bayes.print();

    // Copy Values To Other Entitties
    network.map_of_nodes[curr_id].msg_bayes = msg_bayes;
    network.map_of_nodes[curr_id].bayes_sol = bayes_sol;

    return;

} // end SolveLocalBayesNet()



void DecentralizedSimulator::DisplayResults(Environment env, WirelessNetwork& network){

    std::cout << "Entered Display Results" << std::endl;

    // Calculate Cost of Factor Graph
    double cost_fg = 0.;

    // Iterate over each node
    for (auto &x : network.map_of_nodes){

        WirelessNode node = x.second;

        for(int i1=0; i1<env.num_timesteps; i1++){

            gtsam::Vector state = gtsam::Vector(2); 
            state << node.bayes_sol.at(node.X[node.id][i1])(0), node.bayes_sol.at(node.X[node.id][i1])(1);

            if(i1 < env.num_timesteps-1){
                cost_fg += pow(state(0),2.)*env.state_cost + pow(state(1),2.)*env.state_cost;
            } else {
                cost_fg += pow(state(0),2.)*env.statef_cost + pow(state(1),2.)*env.statef_cost;
            }

            if(i1 < env.num_timesteps-1){
                gtsam::Vector controls = gtsam::Vector(1);
                controls << node.bayes_sol.at(node.U[node.id][i1])(0);
                cost_fg += pow(controls(0),2.)*env.control_cost;
            }

        }

    }

    std::cout << "Total Cost : " << cost_fg << std::endl;

    return;
}
