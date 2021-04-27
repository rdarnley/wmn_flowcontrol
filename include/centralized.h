#ifndef CENTRALIZED_H
#define CENTRALIZED_H

#include <yaml-cpp/yaml.h>
#include <iostream>

#include "solver.h"

class SimulationEnvironment{

    public:

        // Variables
        int num_timesteps;
        double timestep;

        double state_cost;
        double statef_cost;
        double control_cost;

        // Constructor
        SimulationEnvironment(int num_timesteps_, int timestep_, double state_cost_, double statef_cost_, double control_cost_);

        // Functions

    private:

};

class CentralizedSimulator{

    public:

        // Variables


        // Constructor
        CentralizedSimulator();

        // Functions
        template<class SimulationEnvironment>
        void Run(SimulationEnvironment env, int experiment_type_, int queue_init_, int rate_init_, int queue_final_, int rate_final_);
        
        template<int num_nodes, int ss_size, int num_control>
        void Solve(int queue_init, int rate_init, int queue_final, int rate_final);


    private:

};

#endif