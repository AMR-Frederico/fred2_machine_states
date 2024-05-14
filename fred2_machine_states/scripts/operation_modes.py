#!/usr/bin/env python3

import sys

from enum import Enum

from typing import List


class OperationStates(Enum):

    INIT = 0
    MANUAL_MODE = 10
    AUTONOMOUS_MODE = 20
    EMERGENCY_MODE = 30

class OperationModesStateMachine():

    

    # --------------------------------------------------------------------------------
    #    Node Setup
    # --------------------------------------------------------------------------------
        
    def __init__(self):


        self.state = OperationStates.INIT.value


        self.last_change_mode = False
        self.switch_mode = False

        self.robot_safety = False

        self.completed_course = False

        self.reset_robot_state = False

        self.robot_state_msg = Int16()

        self.last_goal_reached = False
        self.goal_reached = False
        self.robot_in_goal = False

        
        self.finish_race = False



        self.load_ros_param()
        self.get_ros_params()

        self.generic_callback = GenericCallback() 
        self.peguei = {}

        # --------------------------------------------------------------------------------
        #    Class parameters
        # --------------------------------------------------------------------------------

        self.change_mode = GenericCallback.data_declare(False)
        self.robot_safety = GenericCallback.data_declare(False)




        self.add_on_set_parameters_callback(self.parameters_callback)


        self.last_safe_status = self.get_clock().now()


    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------
   



    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")

            if param.name == 'manual':
                self.MANUAL = param.value

            elif param.name == 'autonomous':
                self.AUTONOMOUS = param.value

            elif param.name == 'emergency':
                self.EMERGENCY = param.value
            
            elif param.name == 'debug': 
                self.DEBUG = param.name

        return SetParametersResult(successful=True)



    def robotSafety_callback(self, status):
        
        self.robot_safety = status.data

        self.last_safe_status = self.get_clock().now()

    def update_safe_time(self):

        self.last_safe_status = self.get_clock().now()


    def switchMode_callback(self, change_mode):
        

        if change_mode.data > self.last_change_mode:


            if self.robot_mode == self.MANUAL: 

                self.robot_mode = self.AUTONOMOUS



            elif self.robot_mode == self.AUTONOMOUS: 

                self.robot_mode = self.MANUAL         

        self.last_change_mode = change_mode.data


    def machine_states(self):

        current_time = self.get_clock().now()

        change_mode = self.generic_callback.get(self.change_mode)
        robot_safe = self.generic_callback.get(self.robot_safety)

        # p = self.generic_callback.get(self.peguei)
        self.get_logger().info(f"{self.last_safe_status}")

        # -------------------------------------------------------------------------------
        # Filter
        # -------------------------------------------------------------------------------

        # Safety timeout handler
        # if (current_time - self.last_safe_status).nanoseconds > 2e9 and self.robot_safety:

        #     self.robot_safety = False
        #     self.get_logger().warn('Robot safety status set to FALSE due to a timeout (no message received within the last 2 seconds).')
        
        
        # if not self.robot_safety:
            
        #     self.robot_state = self.EMERGENCY


        # else: 
            

        #     if self.robot_mode == self.AUTONOMOUS and not self.completed_course: 
                
        #         self.robot_state = self.AUTONOMOUS



        #         if self.robot_in_goal: 

        #             self.get_logger().warn('ROBOT IN GOAL')
                    
        #             self.robot_state = self.IN_GOAL


        #     if self.completed_course or self.finish_race: 
                
        #         self.finish_race = True 

        #         self.robot_state = self.MISSION_COMPLETED
                
        #         self.get_logger().warn('MISSION COMPLETED')
            
                


        #     elif self.robot_mode == self.MANUAL: 
                
        #         self.robot_state = self.MANUAL
            
            



        # self.robot_state_msg.data = self.robot_state
        # self.robotState_pub.publish(self.robot_state_msg)



        # if debug_mode or self.DEBUG: 
            
        #     self.get_logger().info(f"Robot State: {self.robot_state} | Goal Reached: {self.last_goal_reached} | Mission Completed: {self.completed_course} | Reset: {self.reset_robot_state} | Robot safety: {self.robot_safety}\n")
        
        


if __name__ == '__main__':
    rclpy.init()

    # Create a custom context for single thread and real-time execution
    states_context = rclpy.Context()
    states_context.init()
    states_context.use_real_time = True

    node = Fred_state(
        node_name='main_robot',
        cli_args='--debug',
        context=states_context,
        namespace='machine_states',
        start_parameter_services=True
    )

    # Make the execution in real time
    executor = SingleThreadedExecutor(context=states_context)
    executor.add_node(node=node)

    # Separate thread for callbacks
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10)

    try:
        while rclpy.ok():
            node.machine_states()
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
