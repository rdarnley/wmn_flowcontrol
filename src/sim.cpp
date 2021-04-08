/**
 * @file main.cpp
 * @author Ryan Darnley
 * @date 4/1/21
 * @brief LQR Using Factor Graphs with GTSAM. Distributed Control of Wireless Mesh Network
 */

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "solve.h"

// constexpr int numCo;
// constexpr int numNo;

template<int numN, int numC>
class Simulator{

    public:

        Simulator(const uint32_t &_numTimeSteps, const uint32_t &_numNodes, const uint32_t &_numControl, const uint32_t &_initBuffer, const uint32_t &_initRate, const uint32_t &_finalBuffer, const uint32_t &_finalRate) : numTimeSteps(_numTimeSteps), numNodes(_numNodes), numControl(_numControl), initBuffer(_initBuffer), initRate(_initRate), finalBuffer(_finalBuffer), finalRate(_finalRate)
        {
            this->initialize();
        }

        void run()
        {
            std::cout << "Entered Run Method" << std::endl;

            // Create Instance of Solver
            FGSolver solver;

            // Initialize Solver
            solver.init(numTimeSteps, numNodes, numControl, initBuffer, initRate, finalBuffer, finalRate);

            // Call Factor Graph Solver
            solver.FGSolve(2);

            // Call Factor Graph Sim Solver
            // solver.FGSim();

            // Call DARE Solver
            solver.DARESolve();

            // Call Control-Toolbox Solver
            // solver.CTSolve<numNodes, numControl>();
            // constexpr int numN = numNodes;
            // constexpr int numC = numControl;
            solver.CTSolve<numN, numC>();

            // Print/Save Results
            // outputResults();

        }

    private:

        void initialize()
        {
            std::cout << "Initializing Anything If Necessary" << std::endl;
        }

        uint32_t numTimeSteps;
        uint32_t numNodes;
        uint32_t numControl;
        uint32_t initBuffer;
        uint32_t initRate;
        uint32_t finalBuffer;
        uint32_t finalRate;
};

// constexpr int numControl = 

// template<int stateSpaceSize, int numControl>
// class Simulation{
//     private:

//     public:
//         Simulation(){}

//         void run(){

//             std::cout << "Entered Run Method" << std::endl;

//             // Create Instance of Solver
//             FGSolver solver;

//             // Initialize Solver
//             solver.init(numTimeSteps, numNodes, numControl, initBuffer, initRate, finalBuffer, finalRate);

//             // Call Factor Graph Solver
//             solver.FGSolve(0);

//             // Call Factor Graph Sim Solver
//             // solver.FGSim();

//             // Call DARE Solver
//             solver.DARESolve();

//             // Call Control-Toolbox Solver
//             solver.CTSolve<numNodes, numControl>();
//             // solver.CTSolve(numNodes, numControl);

//             // Print/Save Results
//             // outputResults();

//         }
// }


// template<int i>
// struct loop{
//     static void doit(){
//         Experiment<stateSpaceSize, numControl> exp;
//         exp.runSolve();
//     }
// }

// Main Function
int main(const int argc, const char* argv[]){

    // Parse Command Line Parameters
    std::cout << "Parsing Input" << std::endl;

    // Load YAML File
    std::cout << "Loading YAML" << std::endl;

    YAML::Node config = YAML::LoadFile("../config/config.yaml");

    const uint32_t numTimeSteps = config["numTimeSteps"].as<uint32_t>();
    // const int numNodes = config["numNodes"].as<int>();
    // const int numControl = config["numControl"].as<int>();
    const uint32_t initBuffer = config["initValue"][0].as<uint32_t>();
    const uint32_t initRate = config["initValue"][1].as<uint32_t>();
    const uint32_t finalBuffer = config["finalValue"][0].as<uint32_t>();
    const uint32_t finalRate = config["finalValue"][1].as<uint32_t>();
    
    const uint32_t numNodes = 50;
    const uint32_t numControl = 25;

    const int stateSize = 2*numNodes;

    Simulator<stateSize, numControl> sim(numTimeSteps, numNodes, numControl, initBuffer, initRate, finalBuffer, finalRate);

    // Run Simulator Instance
    sim.run();

    return 0;
}


// 0.018 / 5
// 0.019 / 10