#include "decentralized.h"

int main(const int argc, const char* argv[]){

    std::cout << "Beginning Main" << std::endl;

    // Create Instance of Centralized Scenario Creator
    DecentralizedSimulator sim;

}



// Outline

// class Node

//// Identity neighbors --> have established convention as to direction of traffic flow

//// Create Local FG

//// Solve FG for specific node --> produce GaussianBayesNet, GaussianFactorGraph

//// Transmit GaussianFactorGraph Factors? To Next Node (PropagationEnvironment can just be delay right now???)

//// Receive Back-Sub Solution From Previous Node + Solve Local Bayes Net

