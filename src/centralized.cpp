#include "centralized.h"
#include "utils.h"

//// Begin SimulationEnvironment Constructor
SimulationEnvironment::SimulationEnvironment(int num_timesteps_, int timestep_, double state_cost_, double statef_cost_, double control_cost_){


    num_timesteps = num_timesteps_;
    timestep = timestep_;
    state_cost = state_cost_;
    statef_cost = statef_cost_;
    control_cost = control_cost_;

}

//// End Constructor

//// Begin CentralizedSimulator Constructor
CentralizedSimulator::CentralizedSimulator(){

    // Load YAML File
    std::cout << "Parsing Input" << std::endl;

    YAML::Node config = YAML::LoadFile("../config/config.yaml");

    // Dynamics Information
    const int num_timesteps = config["num_timesteps"].as<int>();
    const double timestep = config["timestep"].as<double>();

    // LQR Information
    const double state_cost = config["state_cost"].as<double>();
    const double statef_cost = config["statef_cost"].as<double>();
    const double control_cost = config["control_cost"].as<double>();

    // State Information
    const int queue_init = config["queue_init"].as<int>();
    const int rate_init = config["rate_init"].as<int>();
    const int queue_final = config["queue_final"].as<int>();
    const int rate_final = config["rate_final"].as<int>();

    // Experiment Information
    const int experiment_type = config["experiment_type"].as<int>();

    // Create Simulation Environment
    SimulationEnvironment env = SimulationEnvironment(  num_timesteps, 
                                                        timestep, state_cost, statef_cost, 
                                                        control_cost    );
            
    // Create Simulations / Solve Simulations
    this->Run<SimulationEnvironment>(env, experiment_type, queue_init, rate_init, queue_final, rate_final);

}
//// End Constructor


//// Begin Run
template<class SimulationEnvironment>
void CentralizedSimulator::Run(SimulationEnvironment env, int experiment_type, int queue_init, int rate_init, int queue_final, int rate_final) {

    std::cout << "Entered Run Function" << std::endl;    

    const int num_nodes = 50;
    const int num_control = 25;
    const int ss_size = 2 * num_nodes;

    // Create LQR Solver Instance
    LqrSolver solver;
    vector<SolverOutput> outputs;

    // Create Scenario
    solver.Init<SimulationEnvironment>(env);

    solver.CreateScenario(num_nodes, num_control, ss_size, queue_init, rate_init, queue_final, rate_final);

    // Solve FG
    outputs.push_back(solver.SolveFG(0));

    // Solve DARE
    outputs.push_back(solver.SolveDARE());

    // Solve CT
    outputs.push_back(solver.SolveCT<ss_size, num_control>());

    // Print Results To Terminal
    PrintResultTerminal(outputs);

    std::cout << "Exited Run Function" << std::endl;

}
//// End Run