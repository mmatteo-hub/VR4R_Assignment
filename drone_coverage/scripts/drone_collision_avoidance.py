#!/usr/bin/env python
import rospy
import math

from coverage_utils import CoverageUtils
from relay_chain_helper import RelayChainHelper

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from airsim_ros_pkgs.msg import VelCmd
from drone_coverage_msgs.msg import RelayInstruction

class DroneCollisionAvoidance:

    def __init__(self):
        # Creating the node for the coverage algorithm
        rospy.init_node("drones_collision_avoidance")
        # Initializing parameters and variables
        self._drones_names = CoverageUtils.get_drones_names()
        drones_count = len(self._drones_names)
        self._drone_index = self._drones_names.index(rospy.get_param("~drone_name"))
        self._rep_min_th2 = pow(rospy.get_param("~repulsion_min_threshold"), 2.0)
        self._rep_max_th2 = pow(rospy.get_param("~repulsion_max_threshold"), 2.0)
        self._red_vel_th2 = pow(rospy.get_param("~reduced_vel_threshold"), 2.0)
        self._halting_th2 = pow(rospy.get_param("~halting_threshold"), 2.0)
        self._restart_th2 = pow(rospy.get_param("~restart_threshold"), 2.0)
        self._reduced_vel_factor = rospy.get_param("~reduced_vel_factor")
        self._repulsion_max_norm = rospy.get_param("~repulsion_max_norm")
        self._drones_positions = [None]*drones_count
        self._drones_at_goal = [True]*drones_count
        self._should_be_halting = False
        self._is_halting = False
        # Creating helper for receiving the data
        self._chain_helper = RelayChainHelper("base_station")
        self._chain_helper.on_goal_callback = self._on_drone_goal_state
        # Creating interfaces for ros communcation
        self._create_ros_interfaces()
        # Creating a rate for stopping the halting
        rospy.Timer(rospy.Duration(0.01), self._toggle_halt_logic)


    def _create_ros_interfaces(self):
        # The name of this drone
        drone_name = self._drones_names[self._drone_index]
        # A Publisher for sending the data to the chain
        self._chain_pub = rospy.Publisher(
            "/relay_chain/"+self._drones_names[0]+"/forward", RelayInstruction, queue_size=10, latch=True
        )
        # Creating a Publisher for listening to the drone velocity
        self._vel_pub = rospy.Publisher(
            "/airsim_node/"+drone_name+"/vel_cmd_world_frame", VelCmd, queue_size=1
        )
        # A Subscriber for receiving the data from the chain
        self._chain_sub = rospy.Subscriber(
            "/relay_chain/base_station/backward", RelayInstruction,
            lambda msg: self._chain_helper._on_chain_backward_data(None, msg)
        )
        # Creatin a Subscriber for publishing the drone velocity
        self._vel_sub = rospy.Subscriber(
            "/airsim_node/"+drone_name+"/vel_cmd_pid_controller", VelCmd,
            self._on_drone_velocity
        )
        # All Subscribers for the drones positions
        for index in range(len(self._drones_names)):
            # Creating a subscriber for each drone odometry
            drone_name = self._drones_names[index]
            rospy.Subscriber(
                "/airsim_node/"+drone_name+"/odom_global_ned", Odometry, 
                lambda msg, index=index : self._on_drone_odometry_message(msg, index)
            )


    def _on_drone_odometry_message(self, msg, index):
        # Storing the updated position of the robot
        self._drones_positions[index] = msg.pose.pose.position
    

    def _on_drone_goal_state(self, drone_name, is_reached):
        # Getting the index for the drone name
        index = self._drones_names.index(drone_name)
        # Storing the updated position of the robot
        self._drones_at_goal[index] = is_reached
    

    def _on_drone_velocity(self, msg):
        # Storing the velocity message
        target_vel = msg.twist.linear
        should_reduce_velocity = False
        repulsive_vel = Point()
        repulsive_components = 0
        # Updating the velocity based on the sourrounding drones
        drone_pose = self._drones_positions[self._drone_index]
        for other_index in range(len(self._drones_names)):
            # Do not check collision with self
            if other_index == self._drone_index :
                continue
            other_pose = self._drones_positions[other_index]
            dist2 = distance2(drone_pose, other_pose)
            # Check if the velocity should be reduced
            if other_index < self._drone_index and dist2 < self._red_vel_th2 :
                should_reduce_velocity = True
            # Check if the distance is more than the repulsion threshold
            if dist2 > self._rep_min_th2 :
                continue
            # Otherwise avoid collisions with repulsive velocity
            magnitude = remap(dist2, self._rep_max_th2, self._rep_min_th2, self._repulsion_max_norm, 0)
            repulsive_component = scale(normalize(subtract(drone_pose, other_pose)), magnitude)
            repulsive_components += 1
            repulsive_vel = add(repulsive_vel, repulsive_component)
        # Reducing the operational velocity
        if should_reduce_velocity :
            target_vel = scale(target_vel, self._reduced_vel_factor)
            rospy.loginfo("["+rospy.get_name()+"] Reducing velocity!")
        # Normalizing the repulsive velocity
        if repulsive_components > 0:
            repulsive_vel = scale(repulsive_vel, 1/repulsive_components)
            target_vel = add(target_vel, repulsive_vel)
            rospy.loginfo("["+rospy.get_name()+"] Adding repulsive velocity!")
        # Publishing the new velocity
        msg.twist.linear = target_vel
        self._vel_pub.publish(msg)
    

    def _toggle_halt_logic(self, event):
        # Making the drones halt if needed
        if not self._is_halting and self._should_halt_drone(self._halting_th2):
            drone_name = self._drones_names[self._drone_index]
            self._chain_helper.send_halt_instruction(self._chain_pub, drone_name, True)
            self._is_halting = True
            rospy.loginfo("["+rospy.get_name()+"] Halting drone!")
            return
        # Making the drone stop halting if he does not need to
        if self._is_halting and not self._should_halt_drone(self._restart_th2):
            drone_name = self._drones_names[self._drone_index]
            self._chain_helper.send_halt_instruction(self._chain_pub, drone_name, False)
            self._is_halting = False
            rospy.loginfo("["+rospy.get_name()+"] Restarting drone!")
            return
    

    def _should_halt_drone(self, threshold2):
        # The position of this drone
        drone_pose = self._drones_positions[self._drone_index]
        if drone_pose == None :
            return
        # Checking if the drone would collide with other drones
        # This also takes into account precedence by indexes.
        for other_index in range(0, self._drone_index) :
            # Ignoring drones already at goal
            if self._drones_at_goal[other_index]:
                continue
            other_pose = self._drones_positions[other_index]
            if other_pose == None :
               continue
            if distance2(drone_pose, other_pose) < threshold2 :
                return True
        return False




def distance2(point0, point1):
    dx = point0.x-point1.x
    dy = point0.y-point1.y
    dz = point0.z-point1.z
    return dx*dx + dy*dy + dz*dz


def subtract(point0, point1):
    dx = point0.x-point1.x
    dy = point0.y-point1.y
    dz = point0.z-point1.z
    return Point(dx, dy, dz)


def add(point0, point1):
    dx = point0.x+point1.x
    dy = point0.y+point1.y
    dz = point0.z+point1.z
    return Point(dx, dy, dz)


def norm(point0):
    x2 = point0.x*point0.x
    y2 = point0.y*point0.y
    z2 = point0.z*point0.z
    return math.sqrt(x2+y2+z2)


def normalize(point0):
    length = norm(point0)
    if length == 0 :
        return point0
    return scale(point0, 1/length)


def scale(point0, factor):
    return Point(point0.x*factor, point0.y*factor, point0.z*factor)



def remap(value, minInput, maxInput, minOutput, maxOutput):
    value = maxInput if value > maxInput else value
    value = minInput if value < minInput else value

    inputSpan = maxInput - minInput
    outputSpan = maxOutput - minOutput

    scaledThrust = float(value - minInput) / float(inputSpan)

    return minOutput + (scaledThrust * outputSpan)



if __name__ == "__main__":
    DroneCollisionAvoidance()
    rospy.spin()