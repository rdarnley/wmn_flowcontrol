#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <yaml-cpp/yaml.h>
#include <iostream>

#include <utility>
#include <vector>
#include <map>

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


    private:
};

class WirelessNetwork{

    public:

        std::map<int, WirelessNode> map_of_nodes;

        WirelessNetwork(){};
        WirelessNetwork(YAML::Node node_topology_);

        void MeshNodes();

    private:

        YAML::Node node_topology;




};











#endif