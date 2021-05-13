// network_utils.cpp

// Author: Ryan Darnley
// Date: 5/12/21

// Brief: Represents a wireless node at an (x,y) location

#include "network_utils.h"

// Distance Function
float GetDistance(Point2D loc1, Point2D loc2){

    float dx = loc1.x - loc2.x;
    float dy = loc1.y - loc2.y;

    return sqrt(dx*dx + dy*dy);
} // end GetDistance()

// Constructor
WirelessNode::WirelessNode(int id_, float x_, float y_, bool controllable_, Point2D sink_location_){

    id = id_;
    location.x = x_;
    location.y = y_;
    controllable = controllable_;

    distance_to_sink = GetDistance(location, sink_location_);

}

void WirelessNode::Mesh(std::map<int, WirelessNode> &map_of_nodes){

    // Get Current Node Info
    int current_id = this->id;
    Point2D current_location = this->location;

    // Iterate Through All Nodes
    for (auto& n : map_of_nodes){

        // Get Node Information
        int node_id = n.first;
        WirelessNode other_node = n.second;

        // Get Distance Metric
        float distance = GetDistance(current_location, other_node.location);

        // Compare Distance To Mesh Distance Threshold
        if(distance < 15.0){

            // Add Neighbor To This Node
            this->neighbor_ids.push_back(node_id);

            // Add Neighbor To Other Node
            n.second.neighbor_ids.push_back(current_id);

        }
    }

    return;
} // end Mesh()

// Constructor
// Input: YAML File Of Wireless Node Locations
WirelessNetwork::WirelessNetwork(YAML::Node node_topology_){

    node_topology = node_topology_;

} // end Constructor

void WirelessNetwork::MeshNodes(){

    Point2D sink_location;

    sink_location.x = node_topology["Sink"][0].as<float>();
    sink_location.y = node_topology["Sink"][1].as<float>();

    YAML::Node nodes = node_topology["Nodes"];

    for(YAML::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {

        int id = it->first.as<int>();

        YAML::Node other_data = it->second;

        float x_loc = other_data[0].as<float>();
        float y_loc = other_data[1].as<float>();
        bool controllable = other_data[2].as<bool>();

        WirelessNode n(id, x_loc, y_loc, controllable, sink_location);

        n.Mesh(map_of_nodes);

        map_of_nodes[id] = n;
    
    }

} // end meshNodes()