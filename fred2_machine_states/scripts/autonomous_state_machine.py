#!/usr/bin/env python3

import sys

from enum import Enum


class AutonomousStates(Enum):

    INIT = 0
    MOVING_TO_GOAL = 10
    AT_WAYPOINT = 30
    AT_GHOST_WAYPOINT = 35
    MISSION_ACCOMPLISHED = 40
    ROBOT_STUCK = 50


def execute_if_not_none(function):

    if function is not None:

        function()



class AutonomousStateMachine():

    def __init__(self, 
                init_callback = None,
                moving_to_goal_callback = None,
                at_waypoint_callback = None,
                at_ghost_waypoint_callback = None,
                mission_accomplished_callback = None,
                robot_stuck_callback = None
                ):
        

        # get from constructor
        self.init_callback = init_callback
        self.moving_to_goal_callback = moving_to_goal_callback
        self.at_waypoint_callback = at_waypoint_callback
        self.at_ghost_waypoint_callback = at_ghost_waypoint_callback
        self.mission_accomplished_callback = mission_accomplished_callback
        self.robot_stuck_callback = robot_stuck_callback

        # -------------------------------------
        #    Class parameters
        # -------------------------------------

        self.state = AutonomousStates.INIT
        self.last_state = AutonomousStates.INIT
        self.initialized = False

        self.robot_stuck = False
        self.at_waypoint = False
        self.no_more_waypoints = False
        self.following_ghost_waypoint = False

        



    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------


    def routine(self):


        # -------------------------------------
        # Input
        # -------------------------------------


        # -------------------------------------
        # Filter
        # -------------------------------------
   
        if self.robot_stuck:

            self.state = AutonomousStates.ROBOT_STUCK


        # -------------------------------------
        # State Machine
        # -------------------------------------

        match self.state:

            case AutonomousStates.INIT:
                
                if not self.initialized:
                    
                    self.state = AutonomousStates.MOVING_TO_GOAL



            case AutonomousStates.MOVING_TO_GOAL:

                if self.at_waypoint and self.following_ghost_waypoint:

                    self.state = AutonomousStates.AT_GHOST_WAYPOINT

                elif self.at_waypoint and not self.following_ghost_waypoint:

                    self.state = AutonomousStates.AT_WAYPOINT



            case AutonomousStates.AT_WAYPOINT:

                if self.no_more_waypoints:

                    self.state = AutonomousStates.MISSION_ACCOMPLISHED
                
                else:

                    self.state = AutonomousStates.MOVING_TO_GOAL



            case AutonomousStates.AT_GHOST_WAYPOINT:

                if self.no_more_waypoints:

                    self.state = AutonomousStates.MISSION_ACCOMPLISHED
                
                else:

                    self.state = AutonomousStates.MOVING_TO_GOAL



            case AutonomousStates.ROBOT_STUCK:

                if not self.robot_stuck:

                    self.state = self.last_state



        # -------------------------------------
        #    Outputs
        # -------------------------------------
        match self.state:

            case AutonomousStates.INIT:

                execute_if_not_none(self.init_callback)
                self.initialized = True


            case AutonomousStates.MOVING_TO_GOAL:

                execute_if_not_none(self.moving_to_goal_callback)


            case AutonomousStates.AT_WAYPOINT:

                execute_if_not_none(self.at_waypoint_callback)


            case AutonomousStates.AT_GHOST_WAYPOINT:

                execute_if_not_none(self.at_ghost_waypoint_callback)


            case AutonomousStates.MISSION_ACCOMPLISHED:

                execute_if_not_none(self.mission_accomplished_callback)
            

            case AutonomousStates.ROBOT_STUCK:

                execute_if_not_none(self.robot_stuck)

            

        # -------------------------------------
        #    Reset variables
        # -------------------------------------

        # store last state
        if self.state != self.last_state:
            self.last_state = self.state 
