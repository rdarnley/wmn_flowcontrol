// lqr_comparison.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: Main File To Be Used For Comparison Of LQR Methods
// Factor Graph, Matrix Math (Householder QR), Control Toolbox (DP)
// Node Topologies Will Be 2D In Space. Will Also Have Time Component.

#include "network_utils.h"
#include "lqr.h"

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
    LqrSolver solver(lqr_info, network);

    // Create LQR FG
    solver.CreateLqrFg();

    // Solve LQR FG
    solver.SolveLqrFg();

    // Create + Solve Matrix Math LQR
    solver.LqrMatrix();

    // // Create + Solve Control Toolbox LQR
    // solver.LqrCt();

    // Any Sort Of Prints / Plots of Results


}