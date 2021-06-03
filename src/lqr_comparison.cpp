// lqr_comparison.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: Main File To Be Used For Comparison Of LQR Methods
// Factor Graph, Matrix Math (Householder QR), Control Toolbox (DP)
// Node Topologies Will Be 2D In Space. Will Also Have Time Component.

#include "network_utils.h"
#include "lqr.h"

void PrintResultTerminal(const vector<SolverOutput>& outputs){

    for (SolverOutput output : outputs) {

        std::cout << "----------" << output.solverType << "---------" << std::endl;
        if (output.solverType == "Factor Graph"){
            std::cout << "Number Nodes : " << output.num_nodes << std::endl;
            std::cout << "Number Control : " << output.num_control << std::endl;
        }
        std::cout << "Number Time Steps : " << output.num_timesteps << std::endl;
        std::cout << "Run Time (s) : " << output.runtime << std::endl;
        std::cout << "Cost : " << output.cost << std::endl;

    }
}

// int main(const int argc, const char* argv[]){

//     std::cout << "Beginning LQR Comparison" << std::endl;

//     // Load Node Topology
//     YAML::Node node_topology = YAML::LoadFile("../config/node_topology.yaml");

//     // Load LQR Info
//     YAML::Node lqr_info = YAML::LoadFile("../config/lqr_info.yaml");

//     // Create Instance of Network
//     WirelessNetwork network(node_topology, lqr_info);

//     // Mesh Comms Nodes
//     network.MeshNodes();
    

//     // Add Centralized Later

//     // Perform Initial Forward Pass For Decentralized Solver
//     network.ForwardPass(10, -1);

//     return;

// }

int main(const int argc, const char* argv[]){

    std::cout << "Beginning LQR Comparison" << std::endl;

    // Load Node Topology
    YAML::Node node_topology = YAML::LoadFile("../config/node_topology.yaml");

    // Create Instance of Network
    WirelessNetwork network(node_topology);

    // Mesh Comms Nodes
    network.MeshNodes();
    
    // Load LQR Info
    YAML::Node lqr_info = YAML::LoadFile("../config/lqr_info.yaml");

    // Create Instance Of LqrSolvers
    // LqrSolver solver(lqr_info, network);
    LqrSolver solver(lqr_info);

    bool centralized = false;
    // if(centralized){

        // Create LQR FG
        solver.CreateLqrFg();

        // Solve LQR FG
        std::vector<SolverOutput> outputs;
        outputs.push_back(solver.SolveLqrFg());

        // Create + Solve Matrix Math LQR
        outputs.push_back(solver.LqrMatrix());

        // Create + Solve Control Toolbox LQR
        // outputs.push_back(solver.LqrCt<info[0], info[1]>());

        // Any Sort Of Prints / Plots of Results
        PrintResultTerminal(outputs);

    // } else {

    //     // solver.ForwardPass(10, -1);
    //     solver.ForwardPass(3, -1, network);

    // }

}