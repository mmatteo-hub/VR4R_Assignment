#!/usr/bin/env python

import rospy

from drone_coverage_msgs.msg import CoveragePath
from drone_coverage_msgs.srv import SetDroneGoalPose


class DroneCoverage:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drone_coverage")
        # Getting the list of drones available
        self._drones_names = DroneCoverage._drones_names_from_str(rospy.get_param("~drones_names"))
        if self._drones_names == None :
            rospy.logerr("Invalid drone names array!")
            return
        rospy.loginfo("Drones names are: "+str(self._drones_names))
        # Creating an action server for listening for new paths
        self._path_sub = rospy.Subscriber("graph_loader/path", CoveragePath, self._on_path_updated)

    
    def _on_path_updated(self, msg):
        # Check if the number of drones are enough
        nodes_count = len(msg.path)
        drone_count = len(self._drones_names)
        if nodes_count > drone_count :
            rospy.logerr("There are not enoguh drones ("+drone_count+") to cover ("+nodes_count+") nodes!")
        size = min(nodes_count, drone_count)
        # Moving the drones to the desired position
        for i in range(size) :
            drone_name = self._drones_names[i]
            pos = msg.path[i]
            self._move_drone_to_position(drone_name, pos)
            rospy.loginfo("Moving "+drone_name+" to (x:"+str(pos.x)+" y:"+str(pos.y)+ " z:"+str(pos.z)+")")
    

    def _move_drone_to_position(self, drone_name, position):
        # Setting the requested position
        move_srv = rospy.ServiceProxy('/airsim_node/'+drone_name+'/local_goal', SetDroneGoalPose)
        move_srv(position.x, position.y, position.z, 0)
        pass

    
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


def main():
    # Creating the node for handling the coverage
    DroneCoverage()
    # Spinning for preventing exit
    rospy.spin()


if __name__ == "__main__":
    main()