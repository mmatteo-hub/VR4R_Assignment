#!/usr/bin/env python

import rospy

from drone_coverage_msgs.msg import RelayInstruction

MOVE_INSTRUCTION_ID = 0
HALT_INSTRUCTION_ID = 1

class RelayChainHelper:

    def __init__(self, prev_identifier, identifier, next_identifier):
        # Creating the Publisher for sending data to the chain
        # The topic for communicating with the next node in the
        self._chain_pub = rospy.Publisher(
            "/relay_chain/"+next_identifier+"/forward", RelayInstruction, queue_size=10
        )
        self._chain_sub = rospy.Subscriber(
            "/relay_chian/"
        )
        # Creating the Subscriber for handling data from the chain
        # Creating empty callbacks
        self._move_instruction_callback = lambda *args, **kwargs: None
        self._halt_instruction_callback = lambda *args, **kwargs: None
    

    def _on_receive_instruction(self, msg):
        if msg.identifier == self._identifier:
            # This drone has executed the instruction
            if msg.instruction == MOVE_INSTRUCTION_ID:
                self._handle_move_instruction(msg.data)
            elif msg.instruction == HALT_INSTRUCTION_ID:
                self._handle_halt_instruction(msg.data)
            return True
        # This drone does not need to execute the instruction
        return False


    def _send_move_instruction(self, node_identifier, position):
        # Creating the instruction for moving the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = MOVE_INSTRUCTION_ID
        msg.data = str(position.x)+","+str(position.y)+","+str(position.z)
        # Setting the requested position
        self._chain_pub.publish(msg)
    

    def _handle_move_instruction(self, data):
        # Parsing the position from the message
        position = list(map(float, data.split(",")))
        # Calling the callback
        self._move_instruction_callback(position)
    

    def _send_halt_instruction(self, node_identifier, halting):
        # Creating the instruction for halting the drone
        msg = RelayInstruction()
        msg.identifier = node_identifier
        msg.instruction = HALT_INSTRUCTION_ID
        msg.data = str(halting)
        # Setting the requested position
        self._chain_pub.publish(msg)
    

    def _handle_halt_instruction(self, data):
        # Parsing the boolaen
        halting = bool(data)
        # Calling the callback
        self._halt_instruction_callback(halting)