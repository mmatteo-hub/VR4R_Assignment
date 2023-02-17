#!/usr/bin/env python

import rospy
from relay_chain_helper import RelayChainHelper

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from drone_coverage_msgs.msg import RelayInstruction
from std_srvs.srv import SetBool
from drone_coverage_msgs.srv import SetDroneGoalPose

class RelayNode:

    def __init__(self):
        # Creating a ros node which represents a node in the relay chain
        rospy.init_node("drone_coverage")
        # Read parameters
        self._base_identifier = rospy.get_param("~base_name")
        self._self_identifier = rospy.get_param("~self_name")
        self._has_backward_node = rospy.get_param("~prev_name") != ""
        self._has_forward_node = rospy.get_param("~next_name") != ""
        # Creating a Publisher for the position of the drone
        self._odom_sub = rospy.Subscriber(
            "/airsim_node/self/odom_global_ned", Odometry, 
            callback=self._on_pose_callback
        )
        # Creating a ServiceProxy for making the drone move
        self._move_srv = rospy.ServiceProxy(
            '/airsim_node/self/local_goal', SetDroneGoalPose
        )
        # Creating a ServiceProxy for halting the drone
        self._halt_srv = rospy.ServiceProxy(
            '/airsim_node/self/halt', SetBool
        )
        # Creating a Publisher for the goal state update
        self._goal_state_msg = rospy.Subscriber(
            "/airsim_node/self/goal_state", Bool,
            callback=self._on_goal_state_callback
        )
        # Finally, initializing topics related to the relay chain
        self._initialize_relay_chain()
    

    def _initialize_relay_chain(self):
        # Creating an helper for handling the relay node
        self._helper = RelayChainHelper(self._self_identifier)
        self._helper.on_move_instruction = self._on_move_instruction
        self._helper.on_halt_instruction = self._on_halt_instruction
        # The Publisher for sending the data in the forward direction
        self._forward_pub = rospy.Publisher(
            "/relay_chain/next/forward", RelayInstruction, queue_size=10
        )
        # The Subscriber for receiving the data in the forward direction
        self._forward_sub = rospy.Subscriber(
            "/relay_chain/self/forward", RelayInstruction,
            lambda msg : self._helper._on_chain_forward_data(self._forward_pub, msg)
        )
        # The Publisher for sending the data in the backward direction
        self._backward_pub = rospy.Publisher(
            "/relay_chain/prev/backward", RelayInstruction, queue_size=10
        )
        # The Subscriber for receiving the data in the backward direction
        self._backward_sub = rospy.Subscriber(
            "/relay_chain/self/backward", RelayInstruction,
            lambda msg : self._helper._on_chain_backward_data(self._backward_pub, msg)
        )


    def _on_pose_callback(self, msg):
        pos = msg.pose.pose.position
        self._helper.send_pose_data(self._backward_pub, self._base_identifier, pos)
    

    def _on_goal_state_callback(self, msg):
        self._helper.send_goal_data(self._backward_pub, self._base_identifier, msg.data)
    

    def _on_move_instruction(self, pos):
        self._move_srv(pos[0], pos[1], pos[2], 0)
        rospy.loginfo("["+rospy.get_name()+"] Moving to (x:"+str(pos[0])+" y:"+str(pos[1])+ " z:"+str(pos[2])+")")
    

    def _on_halt_instruction(self, is_halting):
        self._halt_srv(is_halting)
        rospy.loginfo("["+rospy.get_name()+"] "+("Halting" if is_halting else "Restating")+"!")



if __name__ == "__main__":
    RelayNode()
    rospy.spin()