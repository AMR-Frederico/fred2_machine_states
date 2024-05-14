#!/usr/bin/env python3

import sys

from enum import Enum


class OperationStates(Enum):

    INIT = 0
    MANUAL_MODE = 10
    AUTONOMOUS_MODE = 20
    EMERGENCY_MODE = 30



class OperationModesStateMachine():

    def __init__(self, init_callback = None):

        self.init_callback = init_callback


        # -------------------------------------
        #    Class parameters
        # -------------------------------------

        self.state = OperationStates.INIT.value
        self.last_state = OperationStates.INIT.value

        self.switch_mode = False
        self.robot_safety = False

        



    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------


    def machine_states(self, robot_safety, switch_mode):


        # -------------------------------------
        # Input
        # -------------------------------------

        self.robot_safety = robot_safety
        self.switch_mode = switch_mode


        # -------------------------------------
        # Filter
        # -------------------------------------

        if not self.robot_safety:

            self.state = OperationStates.EMERGENCY_MODE

        # -------------------------------------
        # State Machine
        # -------------------------------------

        match self.state:

            case OperationStates.INIT:
                pass


            case OperationStates.MANUAL_MODE:

                if self.switch_mode:

                    self.state = OperationStates.AUTONOMOUS_MODE


            case OperationStates.AUTONOMOUS_MODE:

                if self.switch_mode:

                    self.state = OperationStates.MANUAL_MODE


            case OperationStates.EMERGENCY_MODE:

                if self.robot_safety:

                    self.state = OperationStates.MANUAL_MODE


        # -------------------------------------
        #    Outputs
        # -------------------------------------


        # -------------------------------------
        #    Reset variables
        # -------------------------------------

        self.robot_safety = False
        self.switch_mode = False
