#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <yaml-cpp/yaml.h>
#include <iostream>

#include <utility>
#include <vector>
#include <map>

#include <yaml-cpp/yaml.h>
#include <iostream>

#include <utility>
#include <vector>
#include <map>

#include "factor.h"

#include <Eigen/Dense>
#include <stdio.h> 
#include <math.h>  
#include <cmath>
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

struct Point2D{
    float x;
    float y;
};

float GetDistance(Point2D loc1, Point2D loc2);

class WirelessNode{

    public:

        WirelessNode(){}
        WirelessNode(int id_, float x_, float y_, bool controllable_, Point2D sink_location_);

        void Mesh(std::map<int, WirelessNode>& map_of_nodes);

        int id;
        Point2D location;
        bool controllable;
        float distance_to_sink;
        std::vector<int> neighbor_ids;

        std::pair < boost::shared_ptr < gtsam::GaussianBayesNet >, boost::shared_ptr < gtsam::GaussianFactorGraph > > pairing;
        gtsam::GaussianFactor::shared_ptr shared_factor;
        gtsam::VectorValues msg_bayes;
        gtsam::VectorValues bayes_sol;

        std::map<int, vector<gtsam::Key>> X;
        std::map<int, vector<gtsam::Key>> U;

        void CreateLocalFg();
        void ConvertLocalFg();
        void SolveLocalBayesNet(std::map<int, WirelessNode> &map_of_nodes);

    private:

        gtsam::Vector prior_state;
        gtsam::Vector final_state;

        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr dyn_noise;

        gtsam::GaussianFactorGraph graph;

        gtsam::Vector Q_node, Qf_node, R_node;
 

};

class WirelessNetwork{

    public:

        std::map<int, WirelessNode> map_of_nodes;
        int num_nodes;
        int num_control;

        WirelessNetwork(){};
        WirelessNetwork(YAML::Node node_topology_);

        void MeshNodes();

        // void NodeSelector

        // void PropagationEnvironment

        void ForwardPass(int current_id, int last_id);

        void BackwardPass(int current_id, int last_id);

        void DisplayResults();

    private:

        YAML::Node node_topology;

        int num_timesteps;
        double timestep;

        double state_cost;
        double statef_cost;
        double control_cost;

        int queue_init;
        double rate_init;
        int queue_final;
        double rate_final;

};











#endif