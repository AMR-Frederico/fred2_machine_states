#!/usr/bin/env python3

import sys

from enum import Enum


class OperationStates(Enum):

    INIT = 0
    MANUAL_MODE = 10
    AUTONOMOUS_MODE = 20
    EMERGENCY_MODE = 30

def execute_if_not_none(function):

    if function is not None:

        function()


class OperationModesStateMachine():

    def __init__(self, 
                init_callback = None,
                manual_mode_callback = None,
                autonomous_mode_callback = None,
                emergency_callback = None):

        self.init_callback = init_callback
        self.manual_mode_callback = manual_mode_callback
        self.autonomous_mode_callback = autonomous_mode_callback
        self.emergency_callback = emergency_callback


        # -------------------------------------
        #    Class parameters
        # -------------------------------------

        self.state = OperationStates.INIT
        self.last_state = OperationStates.INIT
        self.initialized = False

        self.switch_mode = False
        self.robot_safety = False

        



    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------


    def machine_states_routine(self):


        # -------------------------------------
        # Input
        # -------------------------------------


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
                
                if not self.initialized:
                    
                    self.state = OperationStates.MANUAL_MODE


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
        match self.state:

            case OperationStates.INIT:

                execute_if_not_none(self.init_callback)
                
                self.initialized = True


            case OperationStates.MANUAL_MODE:

                execute_if_not_none(self.manual_mode_callback)

            
            case OperationStates.AUTONOMOUS_MODE:

                execute_if_not_none(self.autonomous_mode_callback)


            case OperationStates.EMERGENCY_MODE:

                execute_if_not_none(self.emergency_callback)
                

        # -------------------------------------
        #    Reset variables
        # -------------------------------------

        self.robot_safety = False
        self.switch_mode = False
