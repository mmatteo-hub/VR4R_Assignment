#!/usr/bin/env python

from queue import PriorityQueue

class AStar:
    def __init__(self, graph, cost, heuristic):
        self.graph = graph
        # The cost is the actual cost between two nodes
        self.cost = cost
        # Th heuristic is the estimated cost between two nodes
        self.heuristic = heuristic

    def search(self, start_node, goal_node) :
        # Defining a queue for the next nodes to check
        # The node is initialized with the start_node with priority==cost
        frontier = PriorityQueue()
        frontier.put((0, start_node))
        # This contains the map of the movements
        # It is initialized with the start_node because initially we are there
        came_from = {}        
        came_from[start_node] = None
        # This contains the current cost for reaching a cell
        # The cost of the being in the start_node is 0
        cost_so_far = {}
        cost_so_far[start_node] = 0
        
        goal_found = False
        while not frontier.empty():
            # Taking one of the nodes to analyze
            current = frontier.get()[1]

            # If we are in the goal_node we found the path
            if current == goal_node:
                goal_found = True
                break
            
            # Analyzing every neighbour of the current node
            for neighbour in current.arcs.keys():
                # Computing the cost for going from the current node to its neighbour
                new_cost = cost_so_far[current] + self.cost(current, neighbour) + 10000
                # If the neighbour is not yet being analyzed or if the new computed cost
                # is less than the previos computed one, update it
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour] :
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost + self.heuristic(neighbour, goal_node)
                    frontier.put((priority, neighbour))
                    came_from[neighbour] = current

        # The path has not been found
        if not goal_found :
            return None, -1.0
    
        # Reconstructing the path
        path = AStar._reconstruct_path(goal_node, came_from)
        # Returning the path and the cost for reaching the goal node
        return path, cost_so_far[goal_node]


    @staticmethod
    def _reconstruct_path(goal_node, came_from):
        path = [goal_node]
        while True:
            prev = came_from[path[-1]]
            if prev == None :
                break
            path.append(prev)
        path.reverse()
        return path
