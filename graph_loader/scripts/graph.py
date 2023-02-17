#!/usr/bin/env python

class DirectionalWeightedGraph :

    def __init__(self):
        self.nodes = []


    def add_node(self, value):
        # Creating a node for the specified value
        node = GraphNode(value)
        # Appending the node to the list of nodes
        self.nodes.append(node)
        # Returning the just created node
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
    

    def update_weights(self, updater):
        # Updating all the weights of the graph with the given updater
        for from_node in self.nodes :
            for to_node in from_node.arcs.keys() :
                from_node.arcs[to_node] = updater(from_node, to_node)
    

    @staticmethod
    def update_connected_weights(to_node, updater):
        # Updating only the weights of the connections to the
        # specified node
        for from_node in to_node.arcs.keys() :
            from_node.arcs[to_node] = updater(from_node, to_node)



class GraphNode :

    def __init__(self, value):
        self.value = value
        self.arcs = {}