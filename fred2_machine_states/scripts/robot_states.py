#!/usr/bin/env python3

import rclpy
import threading
import yaml
import sys
import os

from typing import List

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool, Int16


# Parameters file (yaml)
node_path = '~/ros2_ws/src/fred2_machine_states/config/params.yaml'
node_group = 'main_robot'


debug_mode = '--debug' in sys.argv


class Fred_state(Node):

    def __init__(self,
                 node_name: str,
                 *,
                 context:
                 Context = None,
                 cli_args: List[str] = None,
                 namespace: str = None,
                 use_global_arguments: bool = True,
                 enable_rosout: bool = True,
                 start_parameter_services: bool = True,
                 parameter_overrides: List[Parameter] = None,
                 allow_undeclared_parameters: bool = False,
                 automatically_declare_parameters_from_overrides: bool = False) -> None:


        super().__init__(node_name,
                         context=context,
                         cli_args=cli_args,
                         namespace=namespace,
                         use_global_arguments=use_global_arguments,
                         enable_rosout=enable_rosout,
                         start_parameter_services=start_parameter_services,
                         parameter_overrides=parameter_overrides,
                         allow_undeclared_parameters=allow_undeclared_parameters,
                         automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)


        # quality protocol -> the node must not lose any message 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability= QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )

        self.last_change_mode = False
        self.switch_mode = False

        self.robot_safety = False

        self.completed_course = False

        self.reset_robot_state = False

        self.robot_state_msg = Int16()

        self.last_goal_reached = False
        self.goal_reached = False
        self.robot_in_goal = False


        self.load_params(node_path, node_group)
        self.get_params()



        self.create_subscription(Bool,
                                 '/joy/machine_states/switch_mode',
                                 self.switchMode_callback,
                                 qos_profile)


        self.create_subscription(Bool,
                                 '/robot_safety',
                                 self.robotSafety_callback,
                                 qos_profile)


        self.create_subscription(Bool,
                                 '/goal_manager/goal/mission_completed',
                                 self.missionCompleted_callback,
                                 5)


        self.create_subscription(Bool,
                                 '/goal_manager/goal/reached',
                                 self.goalReached_callback,
                                 qos_profile)


        self.create_subscription(Bool,
                                 '/odom/reset',
                                 self.reset_callback,
                                 qos_profile)


        self.robotState_pub = self.create_publisher(Int16, 'robot_state', 10)


        self.add_on_set_parameters_callback(self.parameters_callback)

    
    def parameters_callback(self, params):
        
        for param in params:
            self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")

            if param.name == 'manual':
                self.MANUAL = param.value

            elif param.name == 'autonomous':
                self.AUTONOMOUS = param.value

            elif param.name == 'in_goal':
                self.IN_GOAL = param.value

            elif param.name == 'mission_completed':
                self.MISSION_COMPLETED = param.value

            elif param.name == 'emergency':
                self.EMERGENCY = param.value

        return SetParametersResult(successful=True)




    def reset_callback(self, reset):
        
        self.reset_robot_state = reset.data




    def goalReached_callback(self, goal):
        
        
        if goal.data and not self.last_goal_reached:
            
            self.robot_in_goal = True

        
        else:
            self.robot_in_goal = False


        self.last_goal_reached = goal.data





    def missionCompleted_callback(self, mission_completed):
        
        self.completed_course = mission_completed.data




    def robotSafety_callback(self, status):
        
        self.robot_safety = status.data




    def switchMode_callback(self, change_mode):
        

        if change_mode.data > self.last_change_mode:


            if self.robot_mode == self.MANUAL: 

                self.robot_mode = self.AUTONOMOUS



            elif self.robot_mode == self.AUTONOMOUS: 

                self.robot_mode = self.MANUAL         

        

        self.last_change_mode = change_mode.data




    def machine_states(self):
        
        
        if not self.robot_safety:
            
            self.robot_state = self.EMERGENCY


        else: 
            

            if self.robot_mode == self.AUTONOMOUS: 
                
                self.robot_state = self.AUTONOMOUS



                if self.robot_in_goal: 

                    self.get_logger().warn('ROBOT IN GOAL')
                    
                    self.robot_state = self.IN_GOAL


                
                if self.completed_course: 
                    
                    self.robot_state = self.MISSION_COMPLETED
                    
                    self.get_logger().warn('MISSION COMPLETED')
            


            elif self.robot_mode == self.MANUAL: 
                
                self.robot_state = self.MANUAL
            
            

            if self.reset_robot_state: 
                                
                self.robot_mode = self.MANUAL
                self.robot_state = self.MANUAL


        self.robot_state_msg.data = self.robot_state
        self.robotState_pub.publish(self.robot_state_msg)



        if debug_mode: 
            
            self.get_logger().info(f"Robot State: {self.robot_state} | Goal Reached: {self.last_goal_reached} | Mission Completed: {self.completed_course} | Reset: {self.reset_robot_state} | Robot safety: {self.robot_safety}\n")

        
        
    def load_params(self, path, group):
        param_path = os.path.expanduser(path)

        with open(param_path, 'r') as params_list:
            params = yaml.safe_load(params_list)

        # Get the params inside the specified group
        params = params.get(group, {})

        # Declare parameters with values from the YAML file
        for param_name, param_value in params.items():
            # Adjust parameter name to lowercase
            param_name_lower = param_name.lower()
            self.declare_parameter(param_name_lower, param_value)
            self.get_logger().info(f'{param_name_lower}: {param_value}')



    def get_params(self):
        
        self.MANUAL = self.get_parameter('manual').value
        self.AUTONOMOUS = self.get_parameter('autonomous').value
        self.IN_GOAL = self.get_parameter('in_goal').value
        self.MISSION_COMPLETED = self.get_parameter('mission_completed').value
        self.EMERGENCY = self.get_parameter('emergency').value


        # robot mode (MANUAL | AUTONOMOUS)  # robot state (EMERGENCY | IN GOAL | MISSION COMPLETED | MANUAL | AUTONOMOUS)
        self.robot_state = self.EMERGENCY    # Starts in EMERGENCY state 
        self.robot_mode = self.MANUAL     # Starts in MANUAL mode 




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

    rate = node.create_rate(1)

    try:
        while rclpy.ok():
            node.machine_states()
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
