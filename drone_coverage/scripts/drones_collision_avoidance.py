#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from drone_coverage_msgs.msg import RelayInstruction

class DronesCollisionAvoidance:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drones_collision_avoidance")
        # Getting the list of drones available
        if not self._retrieve_drones_names():
            return
        # Initializing parameters and variables
        self._halting_th2 = pow(rospy.get_param("~halting_threshold"), 2.0)
        self._restart_th2 = pow(rospy.get_param("~restart_threshold"), 2.0)
        drones_count = len(self._drones_names)
        self._drones_odom_subs = [None]*drones_count
        self._drones_halt_srvs = [None]*drones_count
        self._drones_positions = [None]*drones_count
        self._drones_currently_halting = []
        # Creating interfaces for each drone
        for index in range(len(self._drones_names)):
            drone_name = self._drones_names[index]
            # Creating a subscriber for each drone odometry
            self._drones_odom_subs[index] = rospy.Subscriber(
                "/airsim_node/"+drone_name+"/odom_global_ned", Odometry, 
                lambda msg, index=index : self._on_drone_odometry(msg, index)
            )
        # The topic for communicating with the first drone in the chain
        self._chain_pub = rospy.Publisher(
            "/relay_chain/"+self._drones_names[0]+"/forward", RelayInstruction, queue_size=10
        )
        # Creating a rate for executing the collision check
        rospy.Timer(rospy.Duration(0.01), self._check_drones_collisions)


    def _retrieve_drones_names(self):
        # The drones names are passed as an encoded array
        drones_names_str = rospy.get_param("~drones_names")
        self._drones_names = DronesCollisionAvoidance._drones_names_from_str(drones_names_str)
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
    

    def _on_drone_odometry(self, msg, index):
        # Storing the updated position of the robot
        self._drones_positions[index] = msg.pose.pose.position
    

    def _check_drones_collisions(self, event):
        # Making the drones start halting if needed
        new_halting_drones = []
        for drone_index in self._get_not_halting_drones_indexes():
            # Alting the drone if needes to be
            if self._is_halting_needed(drone_index, self._halting_th2):
                self._drones_halt_srvs[drone_index](True)
                new_halting_drones.append(drone_index)
                rospy.loginfo("["+rospy.get_name()+"] Halting "+self._drones_names[drone_index]+"!")
        # Making the drones stop halting if needed
        for i in reversed(range(len(self._drones_currently_halting))):
            drone_index = self._drones_currently_halting[i]
            # Stop alting the drone if he does not need to
            if not self._is_halting_needed(drone_index, self._restart_th2):
                self._drones_halt_srvs[drone_index](False)
                self._drones_currently_halting.pop(i)
                rospy.loginfo("["+rospy.get_name()+"] Restarting "+self._drones_names[drone_index]+"!")
        # Adding all the drones that have been alted
        self._drones_currently_halting.extend(new_halting_drones)
    

    def _get_not_halting_drones_indexes(self):
        # Returning all the drones that are not halting
        return list(filter(
            lambda index : index not in self._drones_currently_halting, 
            range(len(self._drones_names))
        ))
    

    def _is_halting_needed(self, drone_index, threshold2):
        # The position of the specified drone
        drone_pose = self._drones_positions[drone_index]
        if drone_pose == None :
            return
        # Checking if the drone would collide with other drones
        # This also takes into account precedence by indexes.
        for other_index in range(0, drone_index) :
            other_pose = self._drones_positions[other_index]
            if other_pose == None :
               continue
            distance2 = self._distance_sqrd(drone_pose, other_pose)
            print(distance2)
            print(threshold2)
            if distance2 < threshold2 :
                return True
        return False


    def _distance_sqrd(self, point0, point1):
        dx = point0.x-point1.x
        dy = point0.y-point1.y
        dz = point0.z-point1.z
        return dx*dx + dy*dy + dz*dz



if __name__ == "__main__":
    DronesCollisionAvoidance()
    rospy.spin()