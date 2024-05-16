#!/usr/bin/env python3

import rclpy
import threading
import sys

from typing import List

from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import Parameter, ParameterType
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool, Int16

from autonomous_state_machine import AutonomousStateMachine, AutonomousStates

debug_mode = '--debug' in sys.argv

# --------------------------------------------------------------------------------
#    Constants
# --------------------------------------------------------------------------------



# --------------------------------------------------------------------------------
#    Support structures
# --------------------------------------------------------------------------------

class GenericCallback():


    def callback(self, ref, default_value, callback_1= None, callback_2= None, callback_3= None):

        ref['value'] = default_value
        return lambda msg: [self._set(ref, msg.data), 
                            self._execute_if_not_none(callback_1), 
                            self._execute_if_not_none(callback_2), 
                            self._execute_if_not_none(callback_3)]


    def data_declare(self, default_value = {}):
        """Create dict compatible with generic callback

        Args:
            default_value (dict, optional): default value. Defaults to {}.

        Returns:
            dict: dict with value flag setted to default value
        """
        ref = {}
        ref['value'] = default_value
        return ref


    def _set(self, ref, value):
            
        ref['value'] = value


    def _execute_if_not_none(self, function):
            
        if function is not None: 
            
            function()


    def get(self, ref):

        return ref['value']



# --------------------------------------------------------------------------------
#    Node setup
# --------------------------------------------------------------------------------
class OperationModeNode(Node):

            
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
            durability= QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=10, 
            liveliness=QoSLivelinessPolicy.AUTOMATIC
            
        )


        # handle parameters
        # self.load_ros_param()
        # self.get_ros_params()

        self.generic_callback = GenericCallback() 

        # --------------------------------------------------------------------------------
        #    Class parameters
        # --------------------------------------------------------------------------------

        # set data getted from ROS2 callback
        self.mission_accomplished = GenericCallback.data_declare(False)
        self.goal_reached = GenericCallback.data_declare(False)
        self.following_ghost_waypoint = GenericCallback.data_declare(False)

        self.last_goal_reached = False

        self.state_machine = AutonomousStateMachine()


        # --------------------------------------------------------------------------------
        #    Subscribers
        # --------------------------------------------------------------------------------

        # Mission accomplished
        self.create_subscription(Bool,
                                 '/goal_manager/goal/mission_completed',
                                 self.generic_callback.callback(self.mission_accomplished, False),
                                 qos_profile)

        # Goal reached
        self.create_subscription(Bool,
                                 '/goal_manager/goal/reached',
                                 self.generic_callback.callback(self.goal_reached, False),
                                 qos_profile)
        
        # Ghost waypoint
        self.create_subscription(Bool,
                                 '/goal_manager/goal/sinalization',
                                 self.generic_callback.callback(self.following_ghost_waypoint, True),
                                 qos_profile)



        # --------------------------------------------------------------------------------
        #    Publishers
        # --------------------------------------------------------------------------------
        self.autonomous_state_pub = self.create_publisher(Int16, 'autonomous_state', qos_profile)


        self.add_on_set_parameters_callback(self.parameters_callback)



    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------
    
    # def load_ros_param(self):
    
    #     """Load params from to ROS param server 
    #     """
        
    #     # Declare parameters related to robot states and debug/testing
    #     self.declare_parameters(
    #         namespace='',
    #         parameters=[
    #             ('emergency', 0, ParameterDescriptor(description='Index for EMERGENCY state', type=ParameterType.PARAMETER_INTEGER)),
    #             ('manual', 10, ParameterDescriptor(description='Index for MANUAL state', type=ParameterType.PARAMETER_INTEGER)),
    #             ('autonomous', 20, ParameterDescriptor(description='Index for AUTONOMOUS state', type=ParameterType.PARAMETER_INTEGER)),
    #             ('debug', False, ParameterDescriptor(description='Enable debug prints', type=ParameterType.PARAMETER_BOOL))
    #         ]
    #     )

    #     self.get_logger().info('All parameters set successfully')


    # def get_ros_params(self):
        
    #     """Load params from ros param server to internal class
    #     """
    #     self.MANUAL = self.get_parameter('manual').value
    #     self.AUTONOMOUS = self.get_parameter('autonomous').value
    #     self.EMERGENCY = self.get_parameter('emergency').value
    #     self.DEBUG = self.get_parameter('debug').value


    #     self.robot_mode = self.MANUAL     # Starts in MANUAL mode 





    # def parameters_callback(self, params):
        
    #     for param in params:
    #         self.get_logger().info(f"Parameter '{param.name}' changed to: {param.value}")

    #         if param.name == 'manual':
    #             self.MANUAL = param.value

    #         elif param.name == 'autonomous':
    #             self.AUTONOMOUS = param.value

    #         elif param.name == 'emergency':
    #             self.EMERGENCY = param.value
            
    #         elif param.name == 'debug': 
    #             self.DEBUG = param.name

    #     return SetParametersResult(successful=True)



    # def publish_state(self):

    #     self.autonomous_state_msg = Int16()
    #     data = 0

    #     match self.state_machine.state:

    #         case AutonomousStates.INIT:
    #             data = self.DEBUG

    #         case AutonomousStates.:
    #             data = self.MANUAL

    #         case AutonomousStates.AUTONOMOUS_MODE:
    #             data = self.AUTONOMOUS

    #         case AutonomousStates.EMERGENCY_MODE:
    #             data = self.EMERGENCY

    #     self.operation_mode_msg.data = data
    #     self.operation_state_pub.publish(self.operation_mode_msg)



    # --------------------------------------------------------------------------------
    #    State Machine
    # --------------------------------------------------------------------------------

    def machine_states(self):


        # -------------------------------------
        #    ROS Input
        # -------------------------------------

        no_more_waypoints_ros = self.generic_callback.get(self.mission_accomplished)
        goal_reached_ros = self.generic_callback.get(self.goal_reached)
        following_ghost_waypoint_ros = self.generic_callback.get(self.following_ghost_waypoint)


        # -------------------------------------
        #    Filter
        # -------------------------------------

        # detect change mode rising edge
        goal_reached_rising_edge = (goal_reached_ros >  self.last_goal_reached) # add sensors


        # -------------------------------------
        #    Machine State Input
        # -------------------------------------

        self.state_machine.no_more_waypoints = no_more_waypoints_ros
        self.state_machine.following_ghost_waypoint = following_ghost_waypoint_ros
        self.state_machine.following_ghost_waypoint = following_ghost_waypoint_ros

        self.state_machine.routine()

        # -------------------------------------
        #    Output
        # -------------------------------------

        # self.publish_mode()
        # if debug_mode: 
        #     self.get_logger().info(f"State: {self.state_machine.state}, robot safe: {robot_safe}, change_mode_rising_edge: {change_mode_rising_edge}")


        # -------------------------------------
        #    Handle internal variables
        # -------------------------------------

        self.last_goal_reached = goal_reached_ros



if __name__ == '__main__':
    rclpy.init()

    # Create a custom context for single thread and real-time execution
    states_context = rclpy.Context()
    states_context.init()
    states_context.use_real_time = True

    node = OperationModeNode(
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
