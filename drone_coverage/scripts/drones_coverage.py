#!/usr/bin/env python

import rospy

from relay_chain_helper import RelayChainHelper

from std_msgs.msg import Float64
from drone_coverage_msgs.msg import CoveragePath
from drone_coverage_msgs.msg import RelayInstruction


class DronesCoverage:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drones_coverage")
        # Getting the list of drones available
        if not self._retrieve_drones_names():
            return
        self._remaining_drones_to_reach_goal = 0
        # Creating helper for the relay chain the data
        self._chain_helper = RelayChainHelper("base_station")
        self._chain_helper.on_goal_callback = self.on_drone_reach_goal
        # Creating interfaces for ros communcation
        self._create_ros_interfaces()


    def _retrieve_drones_names(self):
        # The drones names are passed as an encoded array
        drones_names_str = rospy.get_param("~drones_names")
        self._drones_names = DronesCoverage._drones_names_from_str(drones_names_str)
        # Check if the names are valid
        if self._drones_names == None :
            rospy.logerr("["+rospy.get_name()+"] Invalid drone names array!")
            return False
        rospy.loginfo("["+rospy.get_name()+"] Drones names are: "+str(self._drones_names))
        return True
    

    @staticmethod
    def _drones_names_from_str(string):
        # Removing eventual spaces at the beginning and end
        string = string.strip()
        # Check if there are the parenthesis
        if string[0] != "[" or string[-1] != "]" :
            return None
        # Removing parenthesis
        string = string[1:-1]
        # Splitting with comman
        return list(map(lambda name : name.strip(), string.split(",")))


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
            "/relay_chain/"+self._drones_names[0]+"/forward", RelayInstruction, queue_size=10
        )
        # Creating a Publisher for updating the cost of the paths
        self._cost_pub = rospy.Publisher(
            "/graph_knowledge/update_last_path_weights", Float64, queue_size=10
        )
    

    def _on_path_updated(self, msg):
        # Check if the number of drones are enough
        nodes_count = len(msg.path)
        drone_count = len(self._drones_names)
        if nodes_count > drone_count :
            rospy.logerr("["+rospy.get_name()+"] Not enough drones ("+str(drone_count)+") to cover "+str(nodes_count)+" nodes!")
        size = min(nodes_count, drone_count)
        # Moving the drones to the desired position
        # We want to move the last drone in the chain first
        for i in reversed(range(size)) :
            drone_name = self._drones_names[i]
            pos = msg.path[i]
            self._chain_helper.send_move_instruction(self._chain_pub, drone_name, pos)
            rospy.loginfo("["+rospy.get_name()+"] Moving "+drone_name+" to (x:"+str(pos.x)+" y:"+str(pos.y)+ " z:"+str(pos.z)+")")
        # Marking the drones that have yet to reach the position
        self._remaining_drones_to_reach_goal = size
    

    def on_drone_reach_goal(self, identifier, is_reached):
        # We are only interested in the case a drone reaches a goal
        if not is_reached :
            return
        # Updating the number of drones that need to reach the gaol
        self._remaining_drones_to_reach_goal -= 1



if __name__ == "__main__":
    DronesCoverage()
    rospy.spin()