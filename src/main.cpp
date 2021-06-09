// main.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: Main File To Be Used For Comparison Of LQR Methods
// Factor Graph, Matrix Math (Householder QR), Control Toolbox (DP)
// Node Topologies Will Be 2D In Space. Will Also Have Time Component.

#include "simulator.h"
#include "network.h"

// void PrintResultTerminal(const vector<SolverOutput>& outputs){

//     for (SolverOutput output : outputs) {

//         std::cout << "----------" << output.solverType << "---------" << std::endl;
//         if (output.solverType == "Factor Graph"){
//             std::cout << "Number Nodes : " << output.num_nodes << std::endl;
//             std::cout << "Number Control : " << output.num_control << std::endl;
//         }
//         std::cout << "Number Time Steps : " << output.num_timesteps << std::endl;
//         std::cout << "Run Time (s) : " << output.runtime << std::endl;
//         std::cout << "Cost : " << output.cost << std::endl;

//     }
// }

int main(const int argc, const char* argv[]){

    std::cout << "Beginning LQR Comparison" << std::endl;

    // Load Node Topology
    YAML::Node node_topology = YAML::LoadFile("../config/node_topology.yaml");

    // Load LQR Info
    YAML::Node lqr_info = YAML::LoadFile("../config/lqr_info.yaml");

    // Create Environment
    Environment env = Environment(lqr_info);

    // Create Instance of Network
    WirelessNetwork network(node_topology);

    // Mesh Comms Nodes
    network.MeshNodes();

    // Create DecentralizedSimulator Instance
    DecentralizedSimulator sim;

    // Begin Sim
    std::vector<int> visited_nodes;
    sim.ForwardPass(env, network, 3, -1, visited_nodes);
    
    // // Print Results Terminal
    // PrintResultTerminal(outputs);
    sim.OutputFile(env, network);

}