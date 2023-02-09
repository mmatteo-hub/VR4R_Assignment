#!/usr/bin/env python

import rospy

from drone_coverage_msgs.msg import RelayInstruction
from drone_coverage_msgs.srv import SetDroneGoalPose

class RelayNode:

    def __init__(self):
        # Creating a ros node which represents a node in the relay chain
        rospy.init_node("drone_coverage")
        # Read parameters
        self._identifier = rospy.get_param("~self_name")
        self._has_prev_node = rospy.get_param("~prev_name") != ""
        self._has_next_node = rospy.get_param("~next_name") != ""
        self._max_node_distance = rospy.get_param("~max_node_distance")
        # The subscriber for receiving the data in the forward direction
        self._forward_data_sub = rospy.Subscriber(
            "/relay_chain/self/forward", RelayInstruction, 
            self._on_relay_chain_forward_data
        )
        # The subscriber for receiving the data in the backward direction
        self._backward_data_sub = rospy.Subscriber(
            "/relay_chain/self/backward", RelayInstruction,
            self._on_relay_chain_backward_data
        )
        # The publisher for sending the data in the forward direction
        self._forward_data_pub = None
        if self._has_next_node :
            self._forward_data_pub = rospy.Publisher(
                "/relay_chain/next/forward", RelayInstruction, queue_size=10
            )
        # The publisher for sending the data in the backward direction
        self._backward_data_pub = None
        if self._has_prev_node :
            self._backward_data_pub = rospy.Publisher(
                "/relay_chain/prev/backward", RelayInstruction, queue_size=10
            )

    
    def _on_relay_chain_forward_data(self, msg):
        # Check if the instruction is to be executed
        if self._check_and_execute_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send forward: "+msg.data)
        # Transmiting the data to the other node
        if self._forward_data_pub != None :
            self._forward_data_pub.publish(msg)


    def _on_relay_chain_backward_data(self, msg):
        # Check if the instruction is to be executed
        if self._check_and_execute_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send backward: "+msg.data)
        # Transmiting the data to the other node
        if self._backward_data_pub != None :
            self._backward_data_pub.publish(msg)
    

    def _check_and_execute_message(self, msg):
        if msg.identifier == self._identifier:
            # This drone has executed the instruction
            if msg.instruction == 0:
                self._receive_message_instruction(msg.data)
            if msg.instruction == 1:
                self._move_to_position_instruction(msg.data)
            return True
        # This drone does not need to execute the instruction
        return False


    def _receive_message_instruction(self, data):
        rospy.loginfo("["+rospy.get_name()+"] Received message: " +data)


    def _move_to_position_instruction(self, data):
        # Parsing the positon from the data string
        pos = list(map(float, data.split(",")))
        # Setting the requested position
        move_srv = rospy.ServiceProxy('/airsim_node/'+self._identifier+'/local_goal', SetDroneGoalPose)
        move_srv(pos[0], pos[1], pos[2], 0)
        rospy.loginfo("["+rospy.get_name()+"] Moving to (x:"+str(pos[0])+" y:"+str(pos[1])+ " z:"+str(pos[2])+")")



def main():
    RelayNode()
    rospy.spin()


if __name__ == "__main__":
    main()