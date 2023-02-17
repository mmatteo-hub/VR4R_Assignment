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
        # Creating interfaces for ros communcation
        self._create_ros_interfaces()
    

    def _create_ros_interfaces(self):
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
        self._create_relay_chain_interfaces()
    

    def _create_relay_chain_interfaces(self):
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
    

    def _on_goal_state_callback(self, msg):
        self._helper.send_goal_data(self._backward_pub, self._base_identifier, msg.data)
    

    def _on_move_instruction(self, pos):
        self._move_srv(pos.x, pos.y, pos.z, 0)
        rospy.loginfo("["+rospy.get_name()+"] Moving to (x:"+str(pos.x)+" y:"+str(pos.y)+ " z:"+str(pos.z)+")")
    

    def _on_halt_instruction(self, is_halting):
        self._halt_srv(is_halting)
        rospy.loginfo("["+rospy.get_name()+"] "+("Halting" if is_halting else "Restating")+"!")



if __name__ == "__main__":
    RelayNode()
    rospy.spin()