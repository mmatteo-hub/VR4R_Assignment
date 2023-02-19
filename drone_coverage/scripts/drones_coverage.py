#!/usr/bin/env python

import rospy

from coverage_utils import CoverageUtils
from relay_chain_helper import RelayChainHelper

from std_msgs.msg import Float64
from drone_coverage_msgs.msg import CoveragePath
from drone_coverage_msgs.msg import RelayInstruction
from drone_coverage_msgs.srv import GetNodesForDistance
from drone_coverage_msgs.srv import ComputeCoveragePath


class DronesCoverage:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drones_coverage")
        # Getting the list of drones available
        self._drones_names = CoverageUtils.get_drones_names()
        self._path_update_delta_cost = rospy.get_param("~path_update_delta_cost")
        self._path_request_period_sec = rospy.get_param("~path_request_period_sec")
        self._remaining_drones_to_reach_goal = 0
        # Creating helper for the relay chain the data
        self._chain_helper = RelayChainHelper("base_station")
        self._chain_helper.on_goal_callback = self.on_drone_reach_goal
        # Creating interfaces for ros communcation
        self._create_ros_interfaces()
        # Obtaining the nodes for the given distance
        # Also considering that the first node is the "base station node" which will not
        # be taken into account when moving drones
        self._nodes_distance_srv.wait_for_service()
        self._reacheable_nodes = self._nodes_distance_srv(len(self._drones_names)).nodes
        self._base_node = self._nodes_distance_srv(0).nodes[0]
        # Computing the first path
        self._create_path_srv.wait_for_service()
        self._create_path_srv.call(self._base_node, self._reacheable_nodes[0])


    def _create_ros_interfaces(self):
        # Creating a Subscriber for listening for new paths
        self._path_sub = rospy.Subscriber(
            "/graph_knowledge/path", CoveragePath, 
            self._on_path_updated
        )
        # A Subscriber for receiving the data from the chain
        self._chain_sub = rospy.Subscriber(
            "/relay_chain/base_station/backward", RelayInstruction,
            lambda msg: self._chain_helper._on_chain_backward_data(None, msg)
        )
        # Creating a Publisher for communicating with the first drone in the chain
        self._chain_pub = rospy.Publisher(
            "/relay_chain/"+self._drones_names[0]+"/forward", RelayInstruction, queue_size=10, latch=True
        )
        # Creating a Publisher for updating the cost of the paths
        self._cost_pub = rospy.Publisher(
            "/graph_knowledge/update_last_path_weights", Float64, queue_size=10
        )
        # Creating a ServiceProxy for obtaining the nodes with the given distance
        self._nodes_distance_srv = rospy.ServiceProxy(
            "/graph_knowledge/nodes_of_distance", GetNodesForDistance
        )
        # Creating a ServiceProxy for requesting a new path
        self._create_path_srv = rospy.ServiceProxy(
            "/graph_knowledge/compute_path", ComputeCoveragePath
        )
    

    def _on_path_updated(self, msg):
        # Storing the last requested path
        self._last_path = msg.path
        actual_path = self._last_path[1:]
        # Check if the number of drones are enough
        nodes_count = len(actual_path)
        drone_count = len(self._drones_names)
        size = min(nodes_count, drone_count)
        # Moving the drones to the desired position
        # We want to move the last drone in the chain first
        for i in reversed(range(size)) :
            drone_name = self._drones_names[i]
            pos = actual_path[i]
            self._chain_helper.send_move_instruction(self._chain_pub, drone_name, pos)
            rospy.loginfo("["+rospy.get_name()+"] Moving "+drone_name+" to (x:"+str(pos.x)+" y:"+str(pos.y)+ " z:"+str(pos.z)+")")
        # Marking the drones that have yet to reach the position
        self._remaining_drones_to_reach_goal = size
    

    def on_drone_reach_goal(self, drone_name, is_reached):
        # We are only interested in the case a drone reaches a goal
        if not is_reached :
            return
        # Updating the number of drones that need to reach the gaol
        self._remaining_drones_to_reach_goal -= 1
        rospy.loginfo("["+rospy.get_name()+"] "+drone_name+" reached goal position!")
        # Check if all the drones arrived at destination
        if self._remaining_drones_to_reach_goal == 0 :
            # Update the path nodes with the new cost
            self._cost_pub.publish(Float64(self._path_update_delta_cost))
            # Wait for the drones to explore the position
            rospy.loginfo("["+rospy.get_name()+"] Waiting some time for the drones exploration.")
            rospy.sleep(self._path_request_period_sec)
            rospy.loginfo("["+rospy.get_name()+"] Requesting a new path.")
            # Requesting a new path with the new goal
            node = self._reacheable_nodes.pop(0)
            self._reacheable_nodes.append(node)
            self._create_path_srv.call(self._base_node, self._reacheable_nodes[0])



if __name__ == "__main__":
    DronesCoverage()
    rospy.spin()