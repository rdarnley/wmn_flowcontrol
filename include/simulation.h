#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "lqr_solver.h"

class Environment{

    public:

        // Factor Graph Specifics
        int numNodes;
        int numControl;
        int numTimeSteps;
        double timeStep;

        // LQR Costs (q --> State, r --> Control)
        double qCost;
        double qfCost;
        double rCost;

        Environment(int _numNodes, int _numControl, int _numTimeSteps, double _timeStep, double _qCost, double _qfCost, double _rCost){
            numNodes = _numNodes;
            numControl = _numControl;
            numTimeSteps = _numTimeSteps;
        }

    private:

};

class Simulator{

    public:

        Simulator()
        {
            this->initialize();
        }

    private:

        template<class Environment>
        void run(Environment env, int _initBuffer, int _initRate, int _finalBuffer, int _finalRate)
        {
            std::cout << "Entered Run Function" << std::endl;

            // Create LQR Solver Object
            LqrSolver solver;
            
            // Create FG/Full Matrix Scenarios
            solver.createScenario<Environment>(env, _initBuffer, _initRate, _finalBuffer, _finalRate);

            // Solve FG Scenario
            // solver.solveFG();

            // Solve DARE Scenario
            // solver.solveDARE();

            // Solve Control Toolbox Scenario
            solver.solveCT<Environment>(env);

            std::cout << "Finished Run Function" << std::endl;

        }

        void initialize()
        {

            // Load Config File
            std::cout << "Parsing Input" << std::endl;

            YAML::Node config = YAML::LoadFile("../config/config.yaml");

            const int numNodes = config["numNodes"].as<int>();
            const int numControl = config["numControl"].as<int>();
            const int numTimeSteps = config["numTimeSteps"].as<int>();
            const double timeStep = config["timeStep"].as<double>();
            const double qCost = config["qCost"].as<double>();
            const double qfCost = config["qfCost"].as<double>();
            const double rCost = config["rCost"].as<double>();

            const int initBuffer = config["initValue"][0].as<int>();
            const int initRate = config["initValue"][1].as<int>();
            const int finalBuffer = config["finalValue"][0].as<int>();
            const int finalRate = config["finalValue"][1].as<int>();

            // Create Simulation Environment
            Environment env = Environment(numNodes, numControl, numTimeSteps, timeStep, qCost, qfCost, rCost);

            // Call Run Function
            run<Environment>(env, initBuffer, initRate, finalBuffer, finalRate);

        }
};

#endif