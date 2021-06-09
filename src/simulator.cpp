// simulator.cpp

#include "simulator.h"

int DecentralizedSimulator::NodeSelector(WirelessNetwork& network, int current_id, bool forward, std::vector<int>& visited_nodes){

    // Get Current Node
    WirelessNode curr_node = network.map_of_nodes[current_id];
    int next_id;

    // Get One of Eligible Neighbors
    if (forward){
        return curr_node.all_potential_ids[0];
    } else {
        auto it = find(visited_nodes.begin(), visited_nodes.end(), current_id);
        // Find index of current id in vector of nodes visited
        if(it != visited_nodes.end()){
            int index = it - visited_nodes.begin();
            next_id = visited_nodes[index-1];
        } else {
            std::cout << "Problem" << std::endl;
            next_id == -1;
        }
        
        // Choose element right before
        return next_id;
    }

} // end NodeSelector()

void DecentralizedSimulator::ForwardPass(Environment env, WirelessNetwork& network, int curr_id, int last_id, std::vector<int>& visited_nodes){

    // Add Current Id To Vector Of Visited Nodes
    visited_nodes.push_back(curr_id);

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
        std::cout << "Current ID" << curr_id << std::endl;
        this->BackwardPass(env, network, curr_id, -1, visited_nodes);
        return;
    } else {
        // Propagation Environment
        // Identity NExt node
        int next_node_id = this->NodeSelector(network, curr_id, true, visited_nodes);
        // this->ForwardPass(env, network, curr_id-1, curr_id);
        this->ForwardPass(env, network, next_node_id, curr_id, visited_nodes);
    }

    return;

} // end ForwardPass()



void DecentralizedSimulator::BackwardPass(Environment env, WirelessNetwork& network, int curr_id, int last_id, std::vector<int>& visited_nodes){

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
        int next_node_id = this->NodeSelector(network, curr_id, false, visited_nodes);
        // this->BackwardPass(env, network, curr_id+1, curr_id);
        this->BackwardPass(env, network, next_node_id, curr_id, visited_nodes);
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
    gtsam::Vector prior_state = gtsam::Vector(state_space);
    if(curr_id == 1){
        prior_state << env.queue_init, env.rate_init;
    } else {
        prior_state << 0, 0;
    }
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
    std::vector<int> eligible_neighbor_ids;
    std::vector<int> all_potential_ids;

    // Create State Keys For Neighbor Nodes
    for(int idx : neighbor_nodes){
        if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
            for(int i1=0; i1<env.num_timesteps-1; i1++){
                X[idx].push_back(gtsam::LabeledSymbol('x', idx, i1));
            }
            eligible_neighbor_ids.push_back(idx);
            all_potential_ids.push_back(idx);
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
    if(last_id != -1){

        std::cout << "Keys" << std::endl;

        boost::shared_ptr<gtsam::GaussianFactor> sh_factor;

        sh_factor = prev_node.shared_factor;

        gtsam::KeyVector sh_keys = sh_factor->keys();
        sh_factor->printKeys();

        // Add New Keys For Non-Neighbors If Necessary
        for(int id : prev_node.eligible_neighbor_ids){

            if(std::find(eligible_neighbor_ids.begin(), eligible_neighbor_ids.end(), id) != eligible_neighbor_ids.end()){
                std::cout << "Found" << std::endl;
            } else {
                all_potential_ids.push_back(id);
                for(int i1=0; i1<env.num_timesteps-1; i1++){
                    X[id].push_back(gtsam::LabeledSymbol('x', id, i1));
                }
            }

        }
        
        graph.add(sh_factor);
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

        ///// Add Inequality
        // std::cout << "1" << std::endl;
        double ineq = 200.0;
        MatrixXd a_ineq = MatrixXd::Zero(1,2);  a_ineq << 1, 0;
        gtsam::Key new_key = gtsam::LabeledSymbol('x', curr_id, i1+100);
        gtsam::LinearInequality lin_ineq = gtsam::LinearInequality(X[curr_id][i1], a_ineq, ineq, new_key);
        // graph.add(lin_ineq);
        
        X[curr_id].push_back(new_key);

        ineq = -1.0;
        a_ineq << -1, 0;
        new_key = gtsam::LabeledSymbol('x', curr_id, i1+250);
        gtsam::LinearInequality lin_ineq1 = gtsam::LinearInequality(X[curr_id][i1], a_ineq, ineq, new_key);
        // graph.add(lin_ineq1);

        ineq = 25.0;
        a_ineq << 0, 1;
        new_key = gtsam::LabeledSymbol('x', curr_id, i1+200);
        gtsam::LinearInequality lin_ineq2 = gtsam::LinearInequality(X[curr_id][i1], a_ineq, ineq, new_key);
        graph.add(lin_ineq2);

        X[curr_id].push_back(new_key);


        ineq = -10.0;
        a_ineq << 0, -1;
        new_key = gtsam::LabeledSymbol('x', curr_id, i1+300);
        gtsam::LinearInequality lin_ineq3 = gtsam::LinearInequality(X[curr_id][i1], a_ineq, ineq, new_key);
        graph.add(lin_ineq3);

        X[curr_id].push_back(new_key);


        // std::cout << "2" << std::endl;
        /////

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
    network.map_of_nodes[curr_id].eligible_neighbor_ids = eligible_neighbor_ids;
    network.map_of_nodes[curr_id].all_potential_ids = all_potential_ids;

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

    std::cout << "-----------" << std::endl;
    bayes_sol.print();

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

void DecentralizedSimulator::OutputFile(Environment env, WirelessNetwork& network){

    ofstream outdata;

    outdata.open("output.txt");

    for(auto &x : network.map_of_nodes){

        outdata << "Node " << x.first << std::endl;

        for(int i1=0; i1<env.num_timesteps-1; i1++){

            outdata << x.second.bayes_sol.at(x.second.U[x.first][i1])(0) << std::endl;

        }

    }

    outdata << "States" << std::endl;

    for(auto &x : network.map_of_nodes){

        outdata << "Node " << x.first << std::endl;

        for(int i1=0; i1<env.num_timesteps; i1++){

            outdata << x.second.bayes_sol.at(x.second.X[x.first][i1])(0) << " " << 
            x.second.bayes_sol.at(x.second.X[x.first][i1])(1) << std::endl;

        }
    }

    // std::cout << network.map_of_nodes[2].bayes_sol.at(network.map_of_nodes[2].X[2][301])(0) << std::endl;

    return;

} // end OutputFile()
