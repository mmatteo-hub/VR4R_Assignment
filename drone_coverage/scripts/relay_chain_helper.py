#!/usr/bin/env python

import rospy

from drone_coverage_msgs.msg import RelayInstruction

MOVE_INSTRUCTION_ID = 0
HALT_INSTRUCTION_ID = 1
POSE_INSTRUCTION_ID = 2


class RelayChainHelper:

    def __init__(self, self_identifier):
        self._self_identifier = self_identifier
        # Creating empty callbacks
        self.move_instruction_callback = lambda *args, **kwargs: None
        self.halt_instruction_callback = lambda *args, **kwargs: None
        self.pose_instruction_callback = lambda *args, **kwargs: None
    

    def _on_chain_forward_data(self, forward_pub, msg):
        # Check if the instruction is to be executed
        if self._check_and_handle_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send forward: "+msg.data)
        # Transmiting the data to the other node
        self.forward_pub.publish(msg)


    def _on_chain_backward_data(self, backward_pub, msg):
        # Check if the instruction is to be executed
        if self._check_and_handle_message(msg) :
            return
        # Logging the message
        rospy.loginfo("["+rospy.get_name()+"] Received instruction to send backward: "+msg.data)
        # Transmiting the data to the other node
        self.backward_pub.publish(msg)


    def _check_and_handle_message(self, msg):
        if msg.identifier == self._self_identifier:
            # This drone has executed the instruction
            if msg.instruction == MOVE_INSTRUCTION_ID:
                self._handle_move_instruction(msg.data)
            elif msg.instruction == HALT_INSTRUCTION_ID:
                self._handle_halt_instruction(msg.data)
            return True
        # This drone does not need to execute the instruction
        return False

    
    def send_pose_instruction(self, chain_pub, node_identifier, position):
        # Creating the instruction for the position of the drone the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = POSE_INSTRUCTION_ID
        msg.data = self._self_identifier
        msg.data += ","+str(position.x)+","+str(position.y)+","+str(position.z)
        # Setting the requested position
        chain_pub.publish(msg)
    
    
    def _handle_pose_instruction(self, data):
        # Parsing the position from the message
        data_array = list(map(float, data.split(",")))
        identifier = data_array.pop(i)
        position = data_array
        # Calling the callback
        self.pose_instruction_callback(identifier, position)


    def send_move_instruction(self, chain_pub, node_identifier, position):
        # Creating the instruction for moving the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = MOVE_INSTRUCTION_ID
        msg.data = str(position.x)+","+str(position.y)+","+str(position.z)
        # Setting the requested position
        chain_pub.publish(msg)
    

    def _handle_move_instruction(self, data):
        # Parsing the position from the message
        position = list(map(float, data.split(",")))
        # Calling the callback
        self.move_instruction_callback(position)
    

    def send_halt_instruction(self, chain_pub, node_identifier, halting):
        # Creating the instruction for halting the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = HALT_INSTRUCTION_ID
        msg.data = str(halting)
        # Setting the requested position
        chain_pub.publish(msg)
    

    def _handle_halt_instruction(self, data):
        # Parsing the boolaen
        halting = bool(data)
        # Calling the callback
        self.halt_instruction_callback(halting)