#include "solver.h"

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