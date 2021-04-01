/**
 * @file main.cpp
 * @author Ryan Darnley
 * @date 3/11/21
 * @brief LQR Using Factor Graphs With GTSAM. Distributed Control of Wireless Mesh Networks.
 */

// Headers/Libraries
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "solver.h"

// Constants/Macros


/**
 * @brief
 * @param
 */
class Simulator{

    public:

        Simulator(uint32_t &_numTimeSteps, uint32_t &_numNodes, uint32_t &_numControl, uint32_t &_initValue, uint32_t &_finalValue) : numTimeSteps(_numTimeSteps), numNodes(_numNodes), numControl(_numControl), initValue(_initValue), finalValue(_finalValue)
        {
            this->initialize();
        }

    private:

        /**
         * @brief Initialize System
         * @param
         */
        void initialize()
        {
            std::cout << "Initializing" << std::endl;

            std::cout << "Calling Run" << std::endl;

            this->run();
        }

        /**
         * @brief Run Simulation
         * @param
         */
        void run()
        {
            std::cout << "Running" << std::endl;

            // Do any sort of loops necessary

            // Create Instance of Solver
            FGSolver solver;
            solver.init(numTimeSteps, numNodes, numControl, initValue, finalValue);

            // Call Factor Graph Solver
            solver.FGSolve(numTimeSteps);

            // Call Factor Graph Sim Solver???
            // solver.FGSim();

            // Call Discrete Algebraic Ricatti Equation Solver
            solver.DARESolve();

            // Call Control-ToolBox Solver
            // solver.CTSolve<numNodes, numControl>();

            // Call Any Other Solvers, Etc

            // Print/Save Results
            // outputResults();
        }

        uint8_t numTimeSteps, numNodes, numControl, initValue, finalValue;

};

// Main Function
int main(const int argc, const char* argv[]){
	
    // Parse Command Line Parameters As Necessary
    std::cout << "Parsing Input" << std::endl;

    // Load Yaml File

    // Simulation Parameters
    uint32_t numTimeSteps = 10;
    uint32_t numNodes = 50;
    uint32_t numControl = 50;
    uint32_t initValue = 30;
    uint32_t finalValue = 20;

    // Create Instance of Simulator Class/Run
    Simulator sim(numTimeSteps, numNodes, numControl, initValue, finalValue);

    // Print/Plot Results
    std::cout << "Plotting Results" << std::endl;

    return 0;
}