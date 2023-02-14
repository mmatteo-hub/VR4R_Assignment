#!/usr/bin/env python

import rospy

from drone_coverage_msgs.msg import CoveragePath
from drone_coverage_msgs.msg import RelayInstruction


class DronesCoverage:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drones_coverage")
        # Getting the list of drones available
        if not self._retrieve_drones_names():
            return
        # Creating a subscriber for listening for new paths
        self._path_sub = rospy.Subscriber(
            "/drones_coverage/path", CoveragePath, 
            self._on_path_updated
        )
        # The topic for communicating with the first drone in the chain
        self._chain_pub = rospy.Publisher(
            "/relay_chain/"+self._drones_names[0]+"/forward", RelayInstruction, queue_size=10
        )


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
    

    def _on_path_updated(self, msg):
        # Check if the number of drones are enough
        nodes_count = len(msg.path)
        drone_count = len(self._drones_names)
        if nodes_count > drone_count :
            rospy.logerr("["+rospy.get_name()+"] Not enoguh drones ("+str(drone_count)+") to cover "+str(nodes_count)+" nodes!")
        size = min(nodes_count, drone_count)
        # Moving the drones to the desired position
        # We want to move the last drone in the chain first
        for i in reversed(range(size)) :
            drone_name = self._drones_names[i]
            pos = msg.path[i]
            self._move_drone_to_position(drone_name, pos)
            rospy.loginfo("["+rospy.get_name()+"] Moving "+drone_name+" to (x:"+str(pos.x)+" y:"+str(pos.y)+ " z:"+str(pos.z)+")")
    

    def _move_drone_to_position(self, drone_name, position):
        # Creating the instruction for moving the drone
        msg = RelayInstruction()
        msg.identifier = drone_name
        msg.instruction = 1
        msg.data = str(position.x)+","+str(position.y)+","+str(position.z)
        # Setting the requested position
        self._chain_pub.publish(msg)



if __name__ == "__main__":
    DronesCoverage()
    rospy.spin()