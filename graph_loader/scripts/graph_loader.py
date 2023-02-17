#!/usr/bin/env python

import rospy
import json
import math

from graph import DirectionalWeightedGraph
from a_star import AStar

from geometry_msgs.msg import Point

from drone_coverage_msgs.srv import LoadCoverageGraph, LoadCoverageGraphResponse
from drone_coverage_msgs.srv import ComputeCoveragePath, ComputeCoveragePathResponse
from drone_coverage_msgs.msg import CoveragePath
from drone_coverage_msgs.msg import UpdateCoverageWeights


class CoverageData:

    @staticmethod
    def fromJson(name, data):
        # Getting the data from the json data
        pos = data[name]["position"]
        self.name = name
        self.pos = Point(pos[0], pos[1], pos[2])
        self.difficulty = data[name]["difficulty"]
        # The cost that might be updated dynamically
        self.dynamic_cost = 0

    
    @staticmethod
    def computeDirectionalCost(data0, data1, wind):
        cost = distance(data0.pos, data1.pos)
        cost += data1.difficulty
        cost += data1.dynamic_cost
        # The wind component
        direction = subtract(data1.pos, data0.pos)
        cost += dot_product(direction, wind)
        # Finally, returning the cost
        return max(cost, 0)


class GraphLoader:

    def __init__(self) :
        # Initializing the ros node for handling graphs and nodes
        rospy.init_node("graph_loader")
        # Initializing the data
        self._graph = DirectionalWeightedGraph()
        self._nodes = {}
        self._last_path = None
        self._last_cost = -1
        self._wind = Point(0, 0, 0)
        # A Service for loading a new graph
        self._load_graph_srv = rospy.Service(
            "graph_loader/load_graph", LoadCoverageGraph, 
            handler=self._on_load_graph_service
        )
        # A Service for computing a new path
        self._compute_path_srv = rospy.Service(
            "graph_loader/compute_path", ComputeCoveragePath, 
            handler=self._on_compute_path_service
        )
        # A Subscriber for updating the wind
        self._wind_sub = rospy.Subscriber(
            "graph_loader/wind", Point,
            callback=self._on_update_wind_message
        )
        # A Subscriber for updating the dynamic weights
        self._weights_update_sub = rospy.Subscriber(
            "graph_loader/update_weights", UpdateCoverageWeights,
            callback=self._on_update_weights_message
        )
        # A Publisher for publishing a new coverage path
        self._path_pub = rospy.Publisher(
            "graph_loader/path", CoveragePath, queue_size=1
        )
        rospy.loginfo("Starting to wait for graph...")

    
    def _on_load_graph_service(self, req):
        # Opening the requested graph
        with open(req.location) as file :
            # Loading the data as a json structure
            data = json.load(file)
            # Reinitializing the data
            self._graph = DirectionalWeightedGraph()
            # Creating the nodes
            self._nodes = {}
            for node_name in data["nodes"] :
                node_data = CoverageData.fromJson(node_name, data["nodes"][node_name])
                self._nodes[node_name] = self._graph.add_node(node_data)
            # Creating the connections
            for connection in data["connections"] :
                node0 = self._nodes[connection[0]]
                node1 = self._nodes[connection[1]]
                to_node0_weight = self._compute_weight(node1, node0)
                to_node1_weight = self._compute_weight(node0, node1)
                self._graph.connect(node0, node1, to_node0_weight, to_node1_weight)
        # Logging info
        rospy.loginfo("Received and loaded new graph correctly!")
        return LoadCoverageGraphResponse(True)
    
    
    def _on_update_wind_message(self, msg):
        # Storing the current wind
        self._wind = msg
        # Updating all the weights
        self._graph.update_weights(self._compute_weight)


    def _on_update_weights_message(self, msg):
        # Updating the weights and connections
        for node_name in msg.nodes_names :
            node = self._nodes[node_name]
            # Updating the dynamic weight for the requested node
            node.value.dynamic_cost += msg.value
            # Updating the connection weights
            self._graph.update_connected_weights(node, self._compute_weight(node1))


    def _on_compute_path_service(self, req):
        # Obtaining the requested nodes
        node_start = self._nodes[req.node_start]
        node_goal = self._nodes[req.node_goal]
        # Computing the path with A*
        a_star = AStar(self._graph, self._graph.weight, self._compute_weight)
        self._last_path, self._last_cost = a_star.search(node_start, node_goal)
        # Check if the path was computed successfully
        if self._last_path == None :
            return ComputeCoveragePathResponse(False)
        # Publishing the path on the topic
        rospy.loginfo("Publishing new path...")
        msg = CoveragePath()
        msg.path = list(map(lambda node : node.value, self._last_path))
        self._path_pub.publish(msg)
        return ComputeCoveragePathResponse(True)
    

    def _compute_weight(node0, node1):
        return CoverageData.computeDirectionalCost(node0.data, node1.data, self._wind)


def dot_product(point0, point1):
    cx = point0.x*point1.x
    cy = point0.y*point1.y
    cz = point0.z*point1.z
    return cx+cy+cz


def subtract(point0, point1):
    dx = point0.x-point1.x
    dy = point0.y-point1.y
    dz = point0.z-point1.z
    return Point(dx, dy, dz)


def distance(point0, point1):
    xd = point0.x-point1.x
    yd = point0.y-point1.y
    zd = point0.z-point1.z
    return math.sqrt(xd*xd + yd*yd + zd*zd)



if __name__ == "__main__":
    GraphLoader()
    rospy.spin()