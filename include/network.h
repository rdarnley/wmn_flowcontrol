#ifndef NETWORK_H
#define NETWORK_H

#pragma once

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
        std::vector<int> eligible_neighbor_ids;
        std::vector<int> all_potential_ids;

        gtsam::GaussianFactorGraph graph;

        std::map<int, vector<gtsam::Key>> X;
        std::map<int, vector<gtsam::Key>> U;

        std::pair < boost::shared_ptr < gtsam::GaussianBayesNet >, boost::shared_ptr < gtsam::GaussianFactorGraph > > pairing;
        gtsam::GaussianFactor::shared_ptr shared_factor;

        gtsam::VectorValues msg_bayes;
        gtsam::VectorValues bayes_sol;

    private:

};

class WirelessNetwork{

    public:

        std::map<int, WirelessNode> map_of_nodes;
        int num_nodes;
        int num_control;

        WirelessNetwork(){};
        WirelessNetwork(YAML::Node node_topology_);

        void MeshNodes();

    private:

        YAML::Node node_topology;

};


#endif