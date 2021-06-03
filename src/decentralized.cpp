#include "decentralized.h"
// #include "node.h"

// Decentralized Constructor
DecentralizedSimulator::DecentralizedSimulator(){

    std::cout << "DecentralizedSimulator Constructor" << std::endl;
    this->Init();

}

void DecentralizedSimulator::Init(){

    // Load YAML File
    std::cout << "Parsing Input" << std::endl;

    YAML::Node config = YAML::LoadFile("../config/lqr_info.yaml");

    Environment env;

    // Dynamics Information
    env.num_timesteps = config["num_timesteps"].as<int>();
    env.timestep = config["timestep"].as<double>();

    // LQR Information
    env.state_cost = config["state_cost"].as<double>();
    env.statef_cost = config["statef_cost"].as<double>();
    env.control_cost = config["control_cost"].as<double>();

    // State Information
    env.queue_init = config["queue_init"].as<int>();
    env.rate_init = config["rate_init"].as<int>();
    env.queue_final = config["queue_final"].as<int>();
    env.rate_final = config["rate_final"].as<int>();

    // Experiment Information
    // env.experiment_type = config["experiment_type"].as<int>();
    // env.max_mesh_distance = config["max_mesh_distance"].as<double>();

    this->CreateTopology(env);

    return;

}

void DecentralizedSimulator::CreateTopology(Environment env){

    // Load YAML File
    std::cout << "Parsing Topology Input" << std::endl;

    YAML::Node config_topology = YAML::LoadFile("../config/node_topology.yaml");
    YAML::Node nodes = config_topology["Nodes"];

    location sink_location;
    sink_location.x = config_topology["Sink"][0].as<float>();
    sink_location.y = config_topology["Sink"][1].as<float>();

    std::map<int, WirelessNode> map_of_nodes;

    int max_id = 0;

    for (YAML::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {

        int id = it->first.as<int>();
        
        YAML::Node location_data = it->second;

        float x_loc = location_data[0].as<float>();
        float y_loc = location_data[1].as<float>();
        bool controllable = location_data[2].as<bool>();
        WirelessNode n(id, x_loc, y_loc, controllable, sink_location);

        n.Mesh(map_of_nodes);

        map_of_nodes[id] = n;

        if(id>max_id){
            max_id = id;
        }
    }

    // Temporary -- Working Correctly
    for (auto& x : map_of_nodes)
    {
        int first = x.first;
        std::vector<neighbor> neighbors = x.second.list_neighbors;
        for (int i=0; i < neighbors.size(); i++)
        {
            std::cout << first << " : " << neighbors.at(i).id << std::endl;
        }
    }
    // End Temporary

    // Select Node To Begin Forward Pass
    // int node_id = this->NodeSelector(map_of_nodes);

    // Recursive Function (Give Rightmost Node Right Now)
    // this->ForwardPass(env, map_of_nodes.rbegin()->first, map_of_nodes);
    // this->ForwardPass(env, node_id, map_of_nodes);
    this->ForwardPass(env, max_id, map_of_nodes);

    return;

}

void DecentralizedSimulator::NodeSelector(std::map<int, WirelessNode>& map_of_nodes){

    std::cout << "Entered Node Selector" << std::endl;

    // Strategy One: Random Number

    // Strategy Two: Highest ID


    // Stategy Three: Farthest From Sink

    // Stategy Four: From Source
    

    return;

}

void DecentralizedSimulator::ForwardPass(Environment env, int id, std::map<int, WirelessNode> &map_of_nodes){

    std::cout << "Entered Forward Pass" << std::endl;

    // Create Local FG
    map_of_nodes[id].CreateLocalFG(env, map_of_nodes);

    // Partially Solve Local FG --> Convert To Bayes Net
    map_of_nodes[id].ConvertFG();

    std::cout << "\n\n\n\n" << std::endl;


    // Pass To Next Node (If Not End) -- Hardcode solution right now
    if(id == 1){
        std::cout << "Entering Backwards Pass - Solving Bayes Net" << std::endl;
        this->BackwardPass(env, 1, map_of_nodes);
        // map_of_nodes[id].pairing.first->print();
        return;
    } else {
        // Something for propagation environment //
        this->ForwardPass(env, id-1, map_of_nodes);
    }

    // if(id==3){
    //     std::cout << "Doing single forward pass" << std::endl;
    //     this->ForwardPass(env, 2, map_of_nodes);
    // }

    return;

} // End ForwardPass()

void DecentralizedSimulator::BackwardPass(Environment env, int id, std::map<int, WirelessNode> &map_of_nodes){

    std::cout << "Entered Backward Pass" << std::endl;

    // Solve Bayes Net
    map_of_nodes[id].SolveBayesNet(map_of_nodes);

    if(id>1){
        map_of_nodes[id].shared_factor->print();
    }

    // Pass To Next Node (If Not End) -- Hardcode Solution Right Now
    if(id == 3){
        std::cout << "Display Results Then Done" << std::endl;
        this->DisplayResults(env, map_of_nodes);
        return;
    } else {
        this->BackwardPass(env, id+1, map_of_nodes);
    }

    return;

} // End BackwardPass()

// void DecentralizedSimulator::DisplayResults(std::map<int, WirelessNode> map_of_nodes){

//     int total_cost = 0.0;




//     return;

// } // End DisplayResults()




////////////////

void WirelessNode::Mesh(std::map<int, WirelessNode> &map_of_nodes){

    std::cout << "Entered Mesh" << std::endl;

    // Strategy One : Very Simple - Only Adjacent Nodes
    // // Current Node id
    // int current_id = this->id;

    // // Iterate through nodes
    // for (auto& n : map_of_nodes){

    //     int node_id = n.first;
    //     WirelessNode other_node = n.second;

    //     if(current_id - node_id == -1 || current_id - node_id == 1){
    //         neighbor neigh;
    //         neigh.id = node_id;
    //         neigh.x = other_node.loc.x;
    //         neigh.y = other_node.loc.y;

    //         this->list_neighbors.push_back(neigh);

    //         neighbor neigh2;
    //         neigh2.id = current_id;
    //         neigh2.x = this->loc.x;
    //         neigh2.y = this->loc.y;

    //         n.second.list_neighbors.push_back(neigh2);
    //     }

    // }

    // Strategy Two : Actual Strategy : Do solely based on distance right now
    // Get Current Node ID
    int current_id = this->id;
    location current_loc = this->loc;

    // Iterate through nodes
    for (auto& n : map_of_nodes){

        // Get Important Node Information
        int node_id = n.first;
        WirelessNode other_node = n.second;

        // Get Distance Metric
        float distance = this->Norm(current_loc, other_node.loc);

        // Compare Distance To Distance Threshold
        if(distance < 15.0){

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

double WirelessNode::Norm(location loc1, location loc2){

    double dx = loc1.x - loc2.x;
    double dy = loc1.y - loc2.y;

    return sqrt(dx*dx + dy*dy);

} // End Norm()

void WirelessNode::CreateLocalFG(Environment env, std::map<int, WirelessNode>& map_of_nodes){

    std::cout << "Entered Create Local FG" << std::endl;

    std::cout << "\n\n\n\n\n" << std::endl;
 
    // Parse Input
    queue_init = env.queue_init;
    rate_init = env.rate_init;
    queue_final = env.queue_final;
    rate_final = env.rate_final;
    num_timesteps = env.num_timesteps;

    state_cost = env.state_cost;
    statef_cost = env.statef_cost;
    control_cost = env.control_cost;

    // Get Mesh Information
    // Length of this->list_neighbors --> max possible value for num_nodes
    // Check if others solved yet in list of neighbors
    // 

    num_nodes = 2;
    if(this->id == 1){
        num_nodes = 1;
    }

    num_control = 1;

    starting_index = this->id;
    prev_index = starting_index-1;

    state_space = 2;

    // Create Constrained Noise Models
    prior_noise = noiseModel::Constrained::All(state_space);
    dyn_noise = noiseModel::Constrained::All(state_space);

    // Initialize Prior/Final
    prior_state = gtsam::Vector(state_space); prior_state << queue_init, rate_init;
    final_state = gtsam::Vector(state_space); final_state << queue_final, rate_final;

    // Initialize Factor Graph
    graph = GaussianFactorGraph();

    ////// Find better way to add keys /////
    // Add prev index node keys
    if(starting_index>1){
        for(int i=0; i<num_timesteps-1; i++){
            X[prev_index].push_back(gtsam::LabeledSymbol('x', prev_index, i));
        }
    }
    // Add curr index node keys
    for(int i=0; i<num_timesteps; i++){
        X[starting_index].push_back(gtsam::LabeledSymbol('x', starting_index, i));
    }
    // Add control keys
    for(int j=0; j<num_control; j++) {
        for(int i=0; i<num_timesteps-1; i++) {
            U[j].push_back(gtsam::LabeledSymbol('u', j, i));
        } // end inner for loop
    } // end outer for loop
    /////////////////////////////////////////

    ////// Find Better Way To Add Priors /////

    // Add prior for actual node
    graph.add(X[starting_index][0], MatrixXd::Identity(2,2), prior_state, prior_noise);
    
    // Add New Constraint From Last FG Solve
    if(starting_index != 3){
        graph.add(map_of_nodes[starting_index+1].shared_factor);
    }

    // Add priors for previous nodes. 1 per each. Tie constraints down
    if(starting_index>1){
        prior_state << 3, 8;
        for (int i=0; i<num_timesteps-1; i++){
            graph.add(X[prev_index][i], MatrixXd::Identity(2,2), prior_state, prior_noise);
        }
    }
    //////////////////////////////////////////

    MatrixXd a_end = MatrixXd::Zero(2,2);   a_end << 1, -1, 0, 1;
    MatrixXd a = MatrixXd::Zero(2,2);       a << 0, 1, 0, 0;
    MatrixXd b = MatrixXd::Zero(2,1);       b << 0, 1;

    for(int i2 = 0; i2 < num_nodes; i2++) {

        // Iterate Through TimeSteps (For FG Method)
        for (int i1=0; i1<num_timesteps-1; i1++) {

            if(i2 == 0 && starting_index!=1){
                std::cout << "Skipping" << std::endl;
            } else if(i2 == 0 && starting_index==1){
                graph.add(X[starting_index][i1], a_end, U[0][i1], b, X[starting_index][i1+1], -gtsam::Matrix::Identity(2,2), gtsam::Matrix::Zero(2,1), dyn_noise);

            } else {
   
                vector<std::pair<Key, MatrixXd>> terms;
                terms.push_back(std::make_pair(X[prev_index][i1], a));
                terms.push_back(std::make_pair(X[starting_index][i1], a_end));
                terms.push_back(std::make_pair(U[0][i1], b));
                terms.push_back(std::make_pair(X[starting_index][i1+1], -gtsam::Matrix::Identity(2,2)));
                graph.add(terms, gtsam::Matrix::Zero(2,1), dyn_noise);
                
            }

        } // end inner/temporal for loop
    } // end outer/spatial for loop

    Q_node = gtsam::Vector(state_space); Q_node << 1.0/state_cost, 1.0/state_cost;
    Qf_node = gtsam::Vector(state_space); Qf_node << 1.0/statef_cost, 1.0/statef_cost;

    R = gtsam::Vector(1); R << 1.0/control_cost;

    noiseModel::Diagonal::shared_ptr state_cost_node = noiseModel::Diagonal::Variances(Q_node);
    noiseModel::Diagonal::shared_ptr statef_cost_node = noiseModel::Diagonal::Variances(Qf_node);
    noiseModel::Diagonal::shared_ptr control_cost_node = noiseModel::Diagonal::Variances(R);

    // Add State + Control Costs
    for(int j=0; j<num_timesteps-1; j++){
        graph.add(X[starting_index][j], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);
        graph.add(U[0][j], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
    }
    graph.add(X[starting_index][num_timesteps-1], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);

    // for(int j=0; j<num_timesteps; j++) {
    //     if(j==num_timesteps - 1){
    //         graph.add(X[starting_index][j], gtsam::Matrix::Identity(2,2), final_state, statef_cost_node);
    //     } else {
    //         graph.add(X[starting_index][j], gtsam::Matrix::Identity(2,2), final_state, state_cost_node);

    //     }
    // }

    // for(int i=0; i<num_control; i++) {
    //     for(int j=0; j<num_timesteps-1; j++) {
    //         graph.add(U[i][j], gtsam::Matrix::Identity(1,1), gtsam::Matrix::Zero(1,1), control_cost_node);
    //     }
    // }
    return;   
}

void WirelessNode::ConvertFG(){

    std::cout << "Entered Convert FG" << std::endl;

    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    // Create Order (Just this node)
    for(int i1=num_timesteps-1; i1>=0; i1--){
        if(i1 != num_timesteps-1){
            order.push_back(U[0][i1]);
        }
        order.push_back(X[starting_index][i1]);
    }

    // Perform Partial FG Solving --> <Gaussian Bayes Net, Gaussian Factor Graph>
    graph.print();

    pairing = graph.eliminatePartialSequential(order);

    std::cout << "-----------------------------------------------" << std::endl;

    pairing.second->print();

    size_t fg_size = pairing.second->size();
    std::cout << "Number Factors: " << fg_size << std::endl;

    std::cout << "-----------------------------------------------" << std::endl;
    
    if(starting_index>1){
        shared_factor = pairing.second->at(fg_size-1);

        shared_factor->print();

        std::cout << "-------------------------------------------------" << std::endl;

        gtsam::KeyVector ky = shared_factor->keys();

        for(auto& i : ky){
            std::cout << "key = " << i << std::endl;
        }
    }

    return;

} // End ConvertFG()

void WirelessNode::SolveBayesNet(std::map<int, WirelessNode>& map_of_nodes){

    std::cout << "Entered Solve Bayes Net" << std::endl;

    // this->pairing.first->print();

    // Recreate Bayes Net If Necessary
    if(starting_index>1){
        // Remove X[prev_index][:] from Bayes Net. Add new at X[prev_index][0]
        // Solve Bayes Net using shared_value at X[prev_index][0]
        // for(int i=1; i<num_timesteps-1; i++){
        //     this->pairing.first->remove(X[prev_index][i]);
        // }
        // this->pairing.first->replace
        gtsam::VectorValues prev_bayes = map_of_nodes[prev_index].msg_bayes;

        // prev_bayes.print();

        // std::cout << "---------------" << std::endl;

        // gtsam::KeyVector ky = this->pairing.first->keyVector();

        // for(auto& i : ky){
        //     std::cout << "key = " << i << std::endl;
        // }

        bayes_sol = this->pairing.first->optimize(prev_bayes);
    } else {
        bayes_sol = this->pairing.first->optimize();
    }

    // Extract Message Value From VectorValues
    // bayes_sol.print();
    std::cout << "ID# : " << starting_index << std::endl;

    for(int i=0; i<num_timesteps-1; i++){
        msg_bayes.insert(std::make_pair(X[starting_index][i], bayes_sol.at(X[starting_index][i])));
    }
    // gtsam::VectorValues msg_bayes = bayes_sol.at(X[starting_index][0]);

    // std::cout << msg_bayes[0] << std::endl;
    // std::cout << msg_bayes[1] << std::endl;

    return;
}

void DecentralizedSimulator::DisplayResults(Environment env, std::map<int, WirelessNode>& map_of_nodes){

    std::cout << "Entered Display Results" << std::endl;

    // Calculate Cost of Factor Graph
    double cost_fg = 0.;

    // Iterate over each node
    for (auto &x : map_of_nodes){

        WirelessNode node = x.second;

        node.bayes_sol.print();

        for(int i1=0; i1<env.num_timesteps; i1++){

            gtsam::Vector state = gtsam::Vector(2); 
            state << node.bayes_sol.at(node.X[node.starting_index][i1])(0), node.bayes_sol.at(node.X[node.starting_index][i1])(1);

            if(i1 < env.num_timesteps-1){
                cost_fg += pow(state(0),2.)*env.state_cost + pow(state(1),2.)*env.state_cost;
            } else {
                cost_fg += pow(state(0),2.)*env.statef_cost + pow(state(1),2.)*env.statef_cost;
            }

            if(i1 < env.num_timesteps-1){
                gtsam::Vector controls = gtsam::Vector(1);
                controls << node.bayes_sol.at(node.U[0][i1])(0);
                cost_fg += pow(controls(0),2.)*env.control_cost;
            }

        }

    }

    std::cout << "Total Cost : " << cost_fg << std::endl;

    return;
}













// Decentralized::Run

//// Load YAML / Create Topology

//// Create Node Instances Per Node

//// Mesh Nodes

//// Loop through nodes

////// Create FG

////// Solve FG

////// Transmit other data to next - just save factors



// Node Constructor

// Create FG

////



// PropagationEnvironment -- to be handled fully later. for now can just be a link





// Focus of code:

// Create many node instances
//          Each Node Represents A Comms Node
//          They will identify their peers / mesh strengths, etc -- eventually noise added there
//          They will create an LQR based factor graph based on their local problem
//          They will solve local problem leaving behind priors
//          Priors sent as message to next node which generates factor graph, solves, etc



// Architecture:

// Class DecentralizedSimulator
    // Create Topology (/ Load YAML With Locations)

// Class Node
    // Number / ID
    // Location

    // Create FG
    // Solve FG

// Class Link
    // Node 1
    // Node 2

    // Transmit










