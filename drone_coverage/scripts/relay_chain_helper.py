#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from drone_coverage_msgs.msg import RelayInstruction

MOVE_INSTRUCTION_ID = 0
GOAL_CALLBACK_ID = 1

class RelayChainHelper:

    def __init__(self, self_identifier):
        self._self_identifier = self_identifier
        # Creating empty callbacks
        self.on_move_instruction = lambda *args, **kwargs: None
        self.on_goal_callback = lambda *args, **kwargs: None
    

    def _on_chain_forward_data(self, forward_pub, msg):
        # Check if the instruction is to be executed
        if self._check_and_handle_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send forward: "+msg.data)
        # Transmiting the data to the other node
        forward_pub.publish(msg)


    def _on_chain_backward_data(self, backward_pub, msg):
        # Check if the instruction is to be executed
        if self._check_and_handle_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send backward: "+msg.data)
        # Transmiting the data to the other node
        backward_pub.publish(msg)


    def _check_and_handle_message(self, msg):
        if msg.identifier == self._self_identifier:
            # This drone has executed the instruction
            if msg.instruction == MOVE_INSTRUCTION_ID:
                self._on_move_instruction(msg.data)
            elif msg.instruction == GOAL_CALLBACK_ID:
                self._on_goal_callback(msg.data)
            return True
        # This drone does not need to execute the instruction
        return False


    def send_move_instruction(self, chain_pub, node_identifier, position):
        # Creating the instruction for moving the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = MOVE_INSTRUCTION_ID
        msg.data = str(position.x)+","+str(position.y)+","+str(position.z)
        # Setting the requested position
        chain_pub.publish(msg)


    def _on_move_instruction(self, data):
        # Parsing the position from the message
        pos = list(map(float, data.split(",")))
        # Calling the callback
        self.on_move_instruction(Point(pos[0], pos[1], pos[2]))

    
    def send_goal_data(self, chain_pub, node_identifier, is_reached):
        # Creating the instruction mesage
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = GOAL_CALLBACK_ID
        msg.data = self._self_identifier
        msg.data += ","+str(is_reached)
        # Setting the requested position
        chain_pub.publish(msg)


    def _on_goal_callback(self, data):
        data_array = data.split(",")
        identifier = data_array.pop(0)
        is_reached = data_array[0] == 'True'
        # Calling the callback
        self.on_goal_callback(identifier, is_reached)
    

    def send_halt_instruction(self, chain_pub, node_identifier, is_halting):
        # Creating the instruction for halting the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = HALT_INSTRUCTION_ID
        msg.data = str(is_halting)
        # Setting the requested position
        chain_pub.publish(msg)
    

    def _on_halt_instruction(self, data):
        # Parsing the boolaen
        halting = data == 'True'
        # Calling the callback
        self.on_halt_instruction(halting)