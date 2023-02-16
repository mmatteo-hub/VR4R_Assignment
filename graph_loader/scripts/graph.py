#!/usr/bin/env python

class DirectionalWeightedGraph :

    def __init__(self):
        self.root = None


    def add_node(self, value):
        # Creating a node for the specified value
        node = GraphNode(value)
        # Setting it as root if we don't have a root (first node)
        if self.root == None :
            self.root = node
        return node

    @staticmethod
    def connect(node0, node1, to_node0_weight, to_node1_weight):
        # Storing the connection into the two nodes
        # using a direction weight
        node0.arcs[node1] = to_node1_weight
        node1.arcs[node0] = to_node0_weight
    
    @staticmethod
    def weight(node0, node1):
        # Getting the arc between the two nodes and the related weight
        # Note that this weight is directional
        return node0.arcs[node1]



class GraphNode :

    def __init__(self, value):
        self.value = value
        self.arcs = {}