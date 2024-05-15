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

from operation_modes import OperationModesStateMachine, OperationStates

debug_mode = '--debug' in sys.argv

# --------------------------------------------------------------------------------
#    Constants
# --------------------------------------------------------------------------------

ROBOT_UNSAFE_TRIGGER_NANOSECONDS = 2e9
# ROBOT_UNSAFE_TRIGGER_NANOSECONDS = 11e9 # debug


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
        self.load_ros_param()
        self.get_ros_params()

        self.generic_callback = GenericCallback() 

        # --------------------------------------------------------------------------------
        #    Class parameters
        # --------------------------------------------------------------------------------

        # set data getted from ROS2 callback
        self.change_mode = GenericCallback.data_declare(False)
        self.robot_safety = GenericCallback.data_declare(False)

        self.last_change_mode = False

        self.state_machine = OperationModesStateMachine()


        # --------------------------------------------------------------------------------
        #    Subscribers
        # --------------------------------------------------------------------------------

        # Switch mode
        self.create_subscription(Bool,
                                 '/joy/machine_states/switch_mode',
                                 self.generic_callback.callback(self.change_mode, False),
                                 qos_profile)

        # Robot safety
        self.create_subscription(Bool,
                                 '/robot_safety',
                                 self.generic_callback.callback(self.robot_safety, False, self.update_safe_time),
                                 qos_profile)



        # --------------------------------------------------------------------------------
        #    Publishers
        # --------------------------------------------------------------------------------
        self.operation_state_pub = self.create_publisher(Int16, 'robot_state', qos_profile) # suggestion: change name to operation_state


        self.add_on_set_parameters_callback(self.parameters_callback)


        self.last_safe_status_clock = self.get_clock().now()


    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------
    
    def load_ros_param(self):
    
        """Load params from to ROS param server 
        """
        
        # Declare parameters related to robot states and debug/testing
        self.declare_parameters(
            namespace='',
            parameters=[
                ('emergency', 0, ParameterDescriptor(description='Index for EMERGENCY state', type=ParameterType.PARAMETER_INTEGER)),
                ('manual', 10, ParameterDescriptor(description='Index for MANUAL state', type=ParameterType.PARAMETER_INTEGER)),
                ('autonomous', 20, ParameterDescriptor(description='Index for AUTONOMOUS state', type=ParameterType.PARAMETER_INTEGER)),
                ('debug', False, ParameterDescriptor(description='Enable debug prints', type=ParameterType.PARAMETER_BOOL))
            ]
        )

        self.get_logger().info('All parameters set successfully')


    def get_ros_params(self):
        
        """Load params from ros param server to internal class
        """
        self.MANUAL = self.get_parameter('manual').value
        self.AUTONOMOUS = self.get_parameter('autonomous').value
        self.EMERGENCY = self.get_parameter('emergency').value
        self.DEBUG = self.get_parameter('debug').value


        self.robot_mode = self.MANUAL     # Starts in MANUAL mode 





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



    def update_safe_time(self):

        self.last_safe_status_clock = self.get_clock().now()

    def publish_mode(self):

        self.operation_mode_msg = Int16()
        data = 0

        match self.state_machine.state:

            case OperationStates.INIT:
                data = self.DEBUG

            case OperationStates.MANUAL_MODE:
                data = self.MANUAL

            case OperationStates.AUTONOMOUS_MODE:
                data = self.AUTONOMOUS

            case OperationStates.EMERGENCY_MODE:
                data = self.EMERGENCY

        self.operation_mode_msg.data = data
        self.operation_state_pub.publish(self.operation_mode_msg)



    # --------------------------------------------------------------------------------
    #    State Machine
    # --------------------------------------------------------------------------------

    def machine_states(self):


        # -------------------------------------
        #    ROS Input
        # -------------------------------------

        current_time = self.get_clock().now()

        change_mode_ros = self.generic_callback.get(self.change_mode)
        robot_safe_ros = self.generic_callback.get(self.robot_safety)


        # -------------------------------------
        #    Filter
        # -------------------------------------

        # detect change mode rising edge
        change_mode_rising_edge = (change_mode_ros >  self.last_change_mode)

        # detect robot unsafe status
        robot_unsafe_timer_triggered = (current_time - self.last_safe_status_clock).nanoseconds > ROBOT_UNSAFE_TRIGGER_NANOSECONDS 
        robot_safe = (robot_safe_ros and not robot_unsafe_timer_triggered)




        # -------------------------------------
        #    Machine State Input
        # -------------------------------------

        self.state_machine.switch_mode = change_mode_rising_edge
        self.state_machine.robot_safety = robot_safe

        self.state_machine.routine()

        # -------------------------------------
        #    Output
        # -------------------------------------

        self.publish_mode()
        if debug_mode: self.get_logger().info(f"State: {self.state_machine.state}, robot safe: {robot_safe}, change_mode_rising_edge: {change_mode_rising_edge}")


        # -------------------------------------
        #    Handle internal variables
        # -------------------------------------

        self.last_change_mode = change_mode_ros



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
