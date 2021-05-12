// #pragma once 
#ifndef NODE_H
#define NODE_H

#include <utility>
#include <vector>
#include <map>
#include "decentralized.h"

#include "factor.h"

#include <iostream>
#include <Eigen/Dense>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
#include <vector>

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

struct location { float x; float y; };
struct neighbor { int id; float x; float y; float distance; };

class WirelessNode{

    public: 

        // Member Variables
        int id;
        location loc;
        std::pair<int, location> id_to_location;

        std::vector<neighbor> list_neighbors;

        // Constructor
        WirelessNode(){}
        WirelessNode(int id_, float x_, float y_){
            id = id_;
            loc.x = x_;
            loc.y = y_;
        }

        // Member Functions
        void Mesh(std::map<int, WirelessNode> &map_of_nodes);
        void CreateLocalFG(Environment env);
        void ConvertFG();
        // SolveBayesNet();

    private:

        int node_index;

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

        vector<vector<gtsam::Key>> X;
        vector<vector<gtsam::Key>> U;

        gtsam::Ordering order;
        gtsam::GaussianFactorGraph graph;

        std::pair < boost::shared_ptr < gtsam::GaussianBayesNet >, boost::shared_ptr < gtsam::GaussianFactorGraph > > pairing;

        
};

#endif