#!/usr/bin/env python

import rospy
import json
import math

from graph import WeightedGraph
from a_star import AStar

from geometry_msgs.msg import Point

from drone_coverage_msgs.srv import LoadCoverageGraph, LoadCoverageGraphResponse
from drone_coverage_msgs.srv import ComputeCoveragePath, ComputeCoveragePathResponse
from drone_coverage_msgs.msg import CoveragePath

class GraphLoader:

    def __init__(self) :
        # Initializing the ros node for handling graphs and nodes
        rospy.init_node("graph_loader")
        # Initializing the data
        self._graph = WeightedGraph()
        self._nodes = {}
        self._last_path = None
        self._last_cost = -1
        # Initializing a service for loading a new graph
        rospy.Service(
            rospy.get_name()+"/load_graph", LoadCoverageGraph, 
            handler = self._load_graph
        )
        # Initializing a service for computing a new path
        rospy.Service(rospy.get_name()+"/compute_path", ComputeCoveragePath, 
            handler = self._compute_path
        )
        # Initializing a publisher for publishing a new coverage path
        self._path_pub = rospy.Publisher(
            rospy.get_name()+"/path", CoveragePath, queue_size=1
        )
        rospy.loginfo("Waiting for graph...")

    
    def _load_graph(self, req):
        # Opening the requested graph
        with open(req.location) as file :
            # Loading the data as a json structure
            data = json.load(file)
            # The nodes are temporarely stored here
            self._nodes = {}
            # Creating the graph based on the nodes
            self._graph = WeightedGraph()
            # Creating the nodes
            for node_name in data["nodes"] :
                coords = data["nodes"][node_name]
                node = self._graph.add_node(Point(coords[0], coords[1], coords[2]))
                self._nodes[node_name] = node
            # Creating the connections
            for connection in data["connections"] :
                node0 = self._nodes[connection[0]]
                node1 = self._nodes[connection[1]]
                weight = GraphLoader._euclidean(node0, node1)
                self._graph.connect(node0, node1, weight)
        
        rospy.loginfo("Received and loaded new graph correctly!")
        
        return LoadCoverageGraphResponse(True)
    

    def _compute_path(self, req):
        # Obtaining the requested nodes
        node_start = self._nodes[req.node_start]
        node_goal = self._nodes[req.node_goal]
        # Computing the path with A*
        a_star = AStar(self._graph, self._graph.weight, GraphLoader._manhattan)
        path, cost = a_star.search(node_start, node_goal)
        self._last_path = path
        self._last_cost = cost
        # Check if the path was computed successfully
        if self._last_path == None :
            return ComputeCoveragePathResponse(False)
        # Publishing the path on the topic
        rospy.loginfo("Publishing new path...")
        msg = CoveragePath()
        msg.path = list(map(lambda node : node.value, self._last_path))
        self._path_pub.publish(msg)
        return ComputeCoveragePathResponse(True)

    
    @staticmethod
    def _manhattan(node0, node1):
        xd = abs(node0.value.x-node1.value.x)
        yd = abs(node0.value.y-node1.value.y)
        zd = abs(node0.value.z-node1.value.z)
        return xd + yd + zd

    
    @staticmethod
    def _euclidean(node0, node1):
        xd = node0.value.x-node1.value.x
        yd = node0.value.y-node1.value.y
        zd = node0.value.z-node1.value.z
        return math.sqrt(xd*xd + yd*yd + zd*zd)



if __name__ == "__main__":
    GraphLoader()
    rospy.spin()