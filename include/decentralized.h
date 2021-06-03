#ifndef DECENTRALIZED_H
#define DECENTRALIZED_H

#include <yaml-cpp/yaml.h>
#include <iostream>

// #include "solver.h"
// #include "node.h"


#include <utility>
#include <vector>
#include <map>

#include "factor.h"

#include <iostream>
#include <Eigen/Dense>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
#include <vector>
#include <algorithm>

using namespace Eigen;
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <bits/stdc++.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <fstream>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/linear/LinearInequality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <cstdlib>
#include <chrono> 
#include <ctime> 
#include <ct/optcon/optcon.h>  
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Rot2.h>
#include <boost/optional.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>

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

    int experiment_type;
    double max_mesh_distance;

};


struct location { float x; float y; };
struct neighbor { int id; float x; float y; float distance; };

class WirelessNode{

    public: 

        // Member Variables
        int id;
        location loc;
        std::pair<int, location> id_to_location;

        std::vector<neighbor> list_neighbors;

        std::pair < boost::shared_ptr < gtsam::GaussianBayesNet >, boost::shared_ptr < gtsam::GaussianFactorGraph > > pairing;
        gtsam::GaussianFactor::shared_ptr shared_factor;
        gtsam::VectorValues msg_bayes;
        gtsam::VectorValues bayes_sol;

        std::map<int, vector<gtsam::Key>> X;
        std::map<int, vector<gtsam::Key>> U;

        int starting_index;


        // Constructor
        WirelessNode(){}
        WirelessNode(int id_, float x_, float y_, bool controllable, location sink_location){
            id = id_;
            loc.x = x_;
            loc.y = y_;
        }

        // Member Functions
        void Mesh(std::map<int, WirelessNode> &map_of_nodes);
        void CreateLocalFG(Environment env, std::map<int, WirelessNode> &map_of_nodes);
        void ConvertFG();
        void SolveBayesNet(std::map<int, WirelessNode> &map_of_nodes);
        double Norm(location loc1, location loc2);

    private:

        int node_index;
        int prev_index;
        // int starting_index;

        int num_timesteps;
        double timestep;
        int num_nodes;
        int num_control;

        int state_space;
        int state_space_total;

        int queue_init;
        double rate_init;
        int queue_final;
        double rate_final;

        double state_cost;
        double statef_cost;
        double control_cost;

        gtsam::Vector Q_node, Qf_node, R;

        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr dyn_noise;

        gtsam::Vector prior_state;
        gtsam::Vector final_state;

        // vector<vector<gtsam::Key>> X;
        // vector<vector<gtsam::Key>> U;

        // std::map<int, vector<gtsam::Key>> X;
        // std::map<int, vector<gtsam::Key>> U;

        gtsam::Ordering order;
        gtsam::GaussianFactorGraph graph;

        
};




class DecentralizedSimulator{

    public:

        // Variables


        // Constructor
        DecentralizedSimulator();

        // Functions
        void Init();

        void CreateTopology(Environment env);

        void NodeSelector(std::map<int, WirelessNode> &map_of_nodes);

        void ForwardPass(Environment env, int id, std::map<int, WirelessNode> &map_of_nodes);
        
        void BackwardPass(Environment env, int id, std::map<int, WirelessNode> &map_of_nodes);

        void DisplayResults(Environment env, std::map<int, WirelessNode>& map_of_nodes);

    private:

};

#endif