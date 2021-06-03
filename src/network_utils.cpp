// network_utils.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: Represents a wireless node at an (x,y) location

#include "network_utils.h"

// Distance Function
float GetDistance(Point2D loc1, Point2D loc2){

    float dx = loc1.x - loc2.x;
    float dy = loc1.y - loc2.y;

    return sqrt(dx*dx + dy*dy);
} // end GetDistance()

// Constructor
WirelessNode::WirelessNode(int id_, float x_, float y_, bool controllable_, Point2D sink_location_){

    id = id_;
    location.x = x_;
    location.y = y_;
    controllable = controllable_;

    distance_to_sink = GetDistance(location, sink_location_);

}

void WirelessNode::Mesh(std::map<int, WirelessNode> &map_of_nodes){

    // Get Current Node Info
    int current_id = this->id;
    Point2D current_location = this->location;

    // Iterate Through All Nodes
    for (auto& n : map_of_nodes){

        // Get Node Information
        int node_id = n.first;
        WirelessNode other_node = n.second;

        // Get Distance Metric
        float distance = GetDistance(current_location, other_node.location);

        // Compare Distance To Mesh Distance Threshold
        if(distance < 15.0){

            // Add Neighbor To This Node
            this->neighbor_ids.push_back(node_id);

            // Add Neighbor To Other Node
            n.second.neighbor_ids.push_back(current_id);

        }
    }

    return;
} // end Mesh()

// void WirelessNode::CreateLocalFg(int prev_id, WirelessNetwork& network){

//     std::cout << "Entered Create Decentralized FG" << std::endl;

//     int state_space = 2;

//     // Set Start And Goal States
//     prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
//     final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final; 

//     prior_noise = noiseModel::Constrained::All(state_space);
//     dyn_noise = noiseModel::Constrained::All(state_space);

//     // Initialize Factor Graph
//     graph = GaussianFactorGraph();

//     // Get Mesh Information
//     std::vector<int> adjacent_nodes = this->neighbor_ids;
//     double distance_sink = this->distance_to_sink;
//     bool controllable = this->controllable;

//     WirelessNode prev_node;
//     if(prev_id != -1){
//         prev_node = network.map_of_nodes[prev_id];
//     } 

//     // Create State Keys For Other Nodes
//     for(int idx : adjacent_nodes){
//         if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
//             for(int i=0; i<num_timesteps-1; i++){
//                 X[idx].push_back(gtsam::LabeledSymbol('x', idx, i));
//             }
//         }
//     }

//     // Create State Keys For This Node
//     for(int i=0; i<num_timesteps; i++){
//         X[node_id].push_back(gtsam::LabeledSymbol('x', node_id, i));
//     }

//     // Create Control Keys For This Node
//     if(controllable){
//         for(int i=0; i<num_timesteps-1; i++){
//             U[node_id].push_back(gtsam::LabeledSymbol('u', node_id, i));
//         }
//     }

//     // Add Priors For This Node
//     graph.add(X[node_id][0], MatrixXd::Identity(2,2), prior_state, prior_noise);

//     // Add Priors To Tie Down Other Nodes
//     prior_state << 1, 2;
//     for (int idx : adjacent_nodes){
//         if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
//             for(int i=0; i<num_timesteps-1; i++){
//                 graph.add(X[idx][i], MatrixXd::Identity(2,2), prior_state, prior_noise);
//             }
//         }
//     }

//     // Add New Constraint From Previous Solver (If One Exists)
//     if(prev_node.id != -1){
//         graph.add(prev_node.shared_factor);
//     }

//     MatrixXd a_end = MatrixXd::Zero(2,2);   a_end << 1, -1, 0, 1;
//     MatrixXd a = MatrixXd::Zero(2,2);       a << 0, 1, 0, 0;
//     MatrixXd b = MatrixXd::Zero(2,1);       b << 0, 1;
    
//     // Iterate Through Time
//     for(int i=0; i<num_timesteps-1; i++){

//         std::vector<std::pair<Key, MatrixXd>> terms;

//         // Iterate Through Neighbors
//         for(int idx : adjacent_nodes){

//             // Verify Node Is "Farther" From Sink Than Current Node
//             if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
//                 terms.push_back(std::make_pair(X[idx][i], a));
            
//             }
//         }

//         terms.push_back(std::make_pair(X[node_id][i], a_end));

//         if(controllable){
//             terms.push_back(std::make_pair(U[node_id][i], b));
//         }

//         terms.push_back(std::make_pair(X[node_id][i+1], -gtsam::Matrix::Identity(2,2)));

//         graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);

//     } // end temporal loop

//     double state_cost = network.

//     Q_node = gtsam::Vector(state_space); Q_node << 1.0/state_cost, 1.0/state_cost;
//     Qf_node = gtsam::Vector(state_space); Qf_node << 1.0/statef_cost, 1.0/statef_cost;

//     R_node = gtsam::Vector(1); R_node << 1.0/control_cost;

//     noiseModel::Diagonal::shared_ptr state_cost_node = noiseModel::Diagonal::Variances(Q_node);
//     noiseModel::Diagonal::shared_ptr statef_cost_node = noiseModel::Diagonal::Variances(Qf_node);
//     noiseModel::Diagonal::shared_ptr control_cost_node = noiseModel::Diagonal::Variances(R_node);

//     // Add State + Control Costs
//     for(int j=0; j<num_timesteps-1; j++){
//         graph.add(X[node_id][j], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);
//         if(controllable){
//             graph.add(U[node_id][j], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
//         }
//     }
//     graph.add(X[node_id][num_timesteps-1], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);

//     std::cout << "\n\n\n\n" << std::endl;
//     graph.print();

//     return;


// } // end CreateLocalFg()



// void WirelessNode::ConvertLocalFg(){

//    std::cout << "Entered Convert Lqr Fg Decentralized" << std::endl;

//     WirelessNode node = network.map_of_nodes[node_id];
//     bool controllable = node.controllable;
//     std::vector<int> adjacent_nodes = node.neighbor_ids;
//     double distance_sink = node.distance_to_sink;

//     // Start Timer
//     std::chrono::time_point<std::chrono::system_clock> start, end;
//     start = std::chrono::system_clock::now();

//     std::cout << node_id << std::endl;

//     // Create Order (Just this node)
//     gtsam::Ordering order;
//     for(int i1=num_timesteps-1; i1>=0; i1--){
//         if(i1 != num_timesteps-1 && controllable){
//             order.push_back(U[node_id][i1]);
//         }
//         order.push_back(X[node_id][i1]);
//     }

//     std::cout << "hi" << std::endl;

//     network.map_of_nodes[node_id].pairing = graph.eliminatePartialSequential(order);

//     size_t fg_size = network.map_of_nodes[node_id].pairing.second->size();

//     bool last = true;

//     for(int idx : adjacent_nodes){
//         if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
//             last = false;
//         }
//     }

//     gtsam::GaussianFactor::shared_ptr shared_factor;

//     if(!last){
//         shared_factor = network.map_of_nodes[node_id].pairing.second->at(fg_size-1);
//         network.map_of_nodes[node_id].shared_factor = shared_factor;
//     } 

//     return shared_factor;


// } // end ConvertLocalFg()



// void WirelessNode::SolveLocalBayesNet(){

//    std::cout << "Entered Solve Bayes Net Decentralized" << std::endl;

//     bool last = false;
//     WirelessNode node = network.map_of_nodes[node_id];
//     std::vector<int> adjacent_nodes = node.neighbor_ids;
//     double distance_sink = node.distance_to_sink;

//     WirelessNode prev_node;
//     if(prev_id != -1){
//         prev_node = network.map_of_nodes[prev_id];
//     }

//     std::cout << "1" << std::endl;

//     std::cout << node_id << std::endl;
//     std::cout << prev_id << std::endl;

//     gtsam::VectorValues bayes_sol;
//     if(prev_id != -1){
//         bayes_sol = network.map_of_nodes[node_id].pairing.first->optimize(prev_node.msg_bayes);
//     } else {
//         bayes_sol = network.map_of_nodes[node_id].pairing.first->optimize();
//     }

//     std::cout << "2" << std::endl;

//     gtsam::VectorValues msg_bayes;
//     // Extract Message Values From VectorValues
//     for(int i=0; i<num_timesteps-1; i++){
//         msg_bayes.insert(std::make_pair(X[node_id][i], bayes_sol.at(X[node_id][i])));
//     }

//     network.map_of_nodes[node_id].msg_bayes = msg_bayes;
//     network.map_of_nodes[node_id].bayes_sol = bayes_sol;

//     return msg_bayes;

// } // end SolveLocalBayesNet()



//////////////////////////////////////////////



// Constructor
// Input: YAML File Of Wireless Node Locations
WirelessNetwork::WirelessNetwork(YAML::Node node_topology_){

    node_topology = node_topology_;
    num_nodes = 0;
    num_control = 0;


} // end Constructor

void WirelessNetwork::MeshNodes(){

    Point2D sink_location;

    sink_location.x = node_topology["Sink"][0].as<float>();
    sink_location.y = node_topology["Sink"][1].as<float>();

    YAML::Node nodes = node_topology["Nodes"];

    for(YAML::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {

        int id = it->first.as<int>();

        YAML::Node other_data = it->second;

        float x_loc = other_data[0].as<float>();
        float y_loc = other_data[1].as<float>();
        bool controllable = other_data[2].as<bool>();

        WirelessNode n(id, x_loc, y_loc, controllable, sink_location);

        n.Mesh(map_of_nodes);

        map_of_nodes[id] = n;

        num_nodes++;

        if(controllable){
            num_control++;
        }
    
    }

    return;

} // end MeshNodes()

// void WirelessNetwork::ForwardPass(int current_id, int last_id){

//     // Access Node Of Interest
//     this->map_of_nodes[current_id].CreateLocalFg(last_id, this);

//     this->map_of_nodes[current_id].graph.print();

//     // // Partially Solve FG --> Convert To Bayes Net
//     // gtsam::GaussianFactor::shared_ptr shared_factor = this->map_of_nodes[current_id].ConvertLocalFg();

//     // bool last = true;
//     // std::vector<int> adjacent_nodes = this->map_of_nodes[current_id].neighbor_ids;
//     // double distance_sink = this->map_of_nodes[current_id].distance_to_sink;

//     // for(int idx : adjacent_nodes){
//     //     if(distance_sink < network.map_of_nodes[idx].distance_to_sink){
//     //         last = false;
//     //     }
//     // }

//     // //
//     // this->map_of_nodes[current_id].pairing.second->print();
//     // //

//     // if(last){
//     //     std::cout << "Done" << std::endl;
//     //     // this->BackwardPass(node_id, -1);
//     //     return;
//     // } else {
//     //     // Propagation Environment this->RFEnvironment();
//     //     // Identify Next Node this->SelectNextNode();
//     //     this->ForwardPass(node_id-1, node_id);    // to be fixed immediately
//     // }

//     return;

// } // end ForwardPass()






// void WirelessNetwork::BackwardPass(){

//     gtsam::VectorValues msg_bayes = this->SolveBayesNetDecentralized(node_id, prev_id);

//     bool first = true;
//     WirelessNode node = network.map_of_nodes[node_id];
//     std::vector<int> adjacent_nodes = node.neighbor_ids;
//     double distance_sink = node.distance_to_sink;

//     for(int idx : adjacent_nodes){
//         if(distance_sink > network.map_of_nodes[idx].distance_to_sink){
//             first = false;
//         }
//     }

//     if(first){
//         std::cout << "Done" << std::endl;
//         return;
//     } else {
//         // Add propagation environment this->RFEnvironment();
//         // Identify next node   this->SelectNextNode();
//         this->BackwardPass(node_id+1, node_id);
//     }

//     return;

// } // end BackwardPass()






// void WirelessNetwork::DisplayResults(){

//     std::cout << "Entered Display Results" << std::endl;

//     // Calculate Cost of Factor Graph
//     double cost_fg = 0.;

//     // Iterate over each node
//     for (auto &x : network.map_of_nodes){

//         WirelessNode node = x.second;

//         node.bayes_sol.print();

//         for(int i1=0; i1<num_timesteps; i1++){

//             gtsam::Vector state = gtsam::Vector(2); 
//             state << node.bayes_sol.at(node.X[node.id][i1])(0), node.bayes_sol.at(node.X[node.id][i1])(1);

//             if(i1 < num_timesteps-1){
//                 cost_fg += pow(state(0),2.)*state_cost + pow(state(1),2.)*state_cost;
//             } else {
//                 cost_fg += pow(state(0),2.)*statef_cost + pow(state(1),2.)*statef_cost;
//             }

//             if(i1 < num_timesteps-1){
//                 gtsam::Vector controls = gtsam::Vector(1);
//                 controls << node.bayes_sol.at(node.U[0][i1])(0);
//                 cost_fg += pow(controls(0),2.)*control_cost;
//             }

//         }

//     }

//     std::cout << "Total Cost : " << cost_fg << std::endl;

//     return;


// } // end DisplayResults()