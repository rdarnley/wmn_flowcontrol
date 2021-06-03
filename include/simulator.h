#ifndef SIMULATOR_H
#define SIMULATOR_H

#pragma once

#include <yaml-cpp/yaml.h>
#include <iostream>

#include <utility>
#include <vector>
#include <map>

#include <iostream>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
#include <algorithm>

#include "network.h"

class SolverOutput{
public:
	Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic> states;
	MatrixXd controls;
	double cost;
	double runtime;
	int orderingType; // 0 - COLAMD, 1 - Serial, 2 - Interwoven
	int num_timesteps;
	std::string solverType; 
    int num_nodes;
    int num_control;

	SolverOutput(){}

	SolverOutput(int n_nodes, int n_times, int n_controls){		
		states = Eigen::Matrix<gtsam::Vector, Dynamic, Dynamic>(n_nodes, n_times);
		controls = MatrixXd(n_controls, n_times);
        num_nodes = n_nodes;
        num_control = n_controls;		
	}
};

struct Environment{

    int num_timesteps;
    double timestep;
    
    double state_cost;
    double statef_cost;
    double control_cost;

    int queue_init;
    double rate_init;
    int queue_final;
    double rate_final;

    Environment(YAML::Node lqr_info){
        num_timesteps = lqr_info["num_timesteps"].as<int>();
        timestep = lqr_info["timestep"].as<double>();

        state_cost = lqr_info["state_cost"].as<double>();
        statef_cost = lqr_info["statef_cost"].as<double>();
        control_cost = lqr_info["control_cost"].as<double>();

        queue_init = lqr_info["queue_init"].as<int>();
        rate_init = lqr_info["rate_init"].as<double>();
        queue_final = lqr_info["queue_final"].as<int>();
        rate_final = lqr_info["rate_final"].as<double>();
    }
};

struct neighbor { 
    int id; 
    float x; 
    float y; 
    float distance; 
};

class DecentralizedSimulator{

    public:

        // Variables


        // Constructor
        DecentralizedSimulator(){};

        // Functions
        // void NodeSelector(std::map<int, WirelessNode> &map_of_nodes);
        void ForwardPass(Environment env, WirelessNetwork& network, int current_id, int last_id);
        void BackwardPass(Environment env, WirelessNetwork& network, int current_id, int last_id);
        void CreateLocalFg(Environment env, WirelessNetwork& network, int current_id, int last_id);
        void ConvertLocalFg(Environment env, WirelessNetwork& network, int current_id, int last_id);
        void SolveLocalBayesNet(Environment env, WirelessNetwork& network, int current_id, int last_id);
        void DisplayResults(Environment env, WirelessNetwork& network);

    private:

};

#endif