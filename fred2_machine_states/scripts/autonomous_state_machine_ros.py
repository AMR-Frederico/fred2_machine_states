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
from rcl_interfaces.srv import GetParameters

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


    def callback(self, ref, callback_1= None, callback_2= None, callback_3= None):

        return lambda msg: [self._set(ref, msg.data), 
                            self._execute_if_not_none(callback_1), 
                            self._execute_if_not_none(callback_2), 
                            self._execute_if_not_none(callback_3)]


    def data_declare(self, default_value = None):
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
class AutonomousStateMachineNode(Node):

            
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

        # set data got from ROS2 callback
        self.mission_accomplished = self.generic_callback.data_declare(False)
        self.goal_reached = self.generic_callback.data_declare(False)
        self.signalize_waypoint = self.generic_callback.data_declare(True)
        self.operation_mode = self.generic_callback.data_declare(999)

        self.last_goal_reached = False

        self.state_machine = AutonomousStateMachine()

        self.operation_mode_INIT = None
        self.operation_mode_EMERGENCY = None
        self.operation_mode_MANUAL = None
        self.operation_mode_AUTONOMOUS = None
        self.get_global_parameters()

        # --------------------------------------------------------------------------------
        #    Subscribers
        # --------------------------------------------------------------------------------

        # Operation mode
        self.create_subscription(Bool,
                                 '/goal_manager/goal/sinalization',
                                 self.generic_callback.callback(self.signalize_waypoint),
                                 qos_profile)
        
        # Mission accomplished
        self.create_subscription(Bool,
                                 '/goal_manager/goal/mission_completed',
                                 self.generic_callback.callback(self.mission_accomplished),
                                 qos_profile)

        # Goal reached
        self.create_subscription(Bool,
                                 '/goal_manager/goal/reached',
                                 self.generic_callback.callback(self.goal_reached),
                                 qos_profile)
        
        # Operation mode
        self.create_subscription(Int16,
                                 '/main_robot/operation_mode',
                                 self.generic_callback.callback(self.operation_mode),
                                 qos_profile)

    


        # --------------------------------------------------------------------------------
        #    Publishers
        # --------------------------------------------------------------------------------
        self.autonomous_state_pub = self.create_publisher(Int16, 'autonomous_state', qos_profile)


        # self.add_on_set_parameters_callback(self.parameters_callback)



    # --------------------------------------------------------------------------------
    #    Class Functions
    # --------------------------------------------------------------------------------
   
    def get_global_parameters(self):

        self.client = self.create_client(GetParameters, '/main_robot/operation_modes/get_parameters')
        self.client.wait_for_service()

        request = GetParameters.Request()
        request.names = ['manual', 'autonomous', 'emergency']

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_get_global_param)
        
        

    
    def callback_get_global_param(self, future):


        try:

            result = future.result()

            self.operation_mode_MANUAL = result.values[0].integer_value
            self.operation_mode_AUTONOMOUS = result.values[1].integer_value
            self.operation_mode_EMERGENCY = result.values[2].integer_value


        except Exception as e:

            self.get_logger().warn("Service call failed %r" % (e,))

    def load_ros_param(self):
    
        """Load params from to ROS param server 
        """
        
        # Declare parameters related to robot states and debug/testing
        self.declare_parameters(
            namespace='',
            parameters=[
                ('init', 0, ParameterDescriptor(description='Index for INIT state', type=ParameterType.PARAMETER_INTEGER)),
                ('moving_to_goal', 10, ParameterDescriptor(description='Index for MOVING_TO_GOAL state', type=ParameterType.PARAMETER_INTEGER)),
                ('at_waypoint', 20, ParameterDescriptor(description='Index for AT_WAYPOINT state', type=ParameterType.PARAMETER_INTEGER)),
                ('at_ghost_waypoint', 30, ParameterDescriptor(description='Index for AT_GHOST_WAYPOINT state', type=ParameterType.PARAMETER_INTEGER)),
                ('mission_accomplished', 40, ParameterDescriptor(description='Index for MISSION_ACCOMPLISHED state', type=ParameterType.PARAMETER_INTEGER)),
                ('robot_stuck', 50, ParameterDescriptor(description='Index for ROBOT_STUCK state', type=ParameterType.PARAMETER_INTEGER)),
                ('paused', 60, ParameterDescriptor(description='Index for PAUSED state', type=ParameterType.PARAMETER_INTEGER)),
                ('debug', False, ParameterDescriptor(description='Enable debug prints', type=ParameterType.PARAMETER_BOOL))
            ]
        )

        self.get_logger().info('All parameters set successfully')


    def get_ros_params(self):
        
        """Load params from ros param server to internal class
        """
        self.INIT =                 self.get_parameter('init').value
        self.MOVING_TO_GOAL =       self.get_parameter('moving_to_goal').value
        self.AT_WAYPOINT =          self.get_parameter('at_waypoint').value
        self.AT_GHOST_WAYPOINT =    self.get_parameter('at_ghost_waypoint').value
        self.MISSION_ACCOMPLISHED = self.get_parameter('mission_accomplished').value
        self.ROBOT_STUCK =          self.get_parameter('robot_stuck').value
        self.PAUSED =                self.get_parameter('paused').value
        self.DEBUG =                self.get_parameter('debug').value


        self.robot_mode = self.INIT     # Starts in MANUAL mode 


    def publish_state(self):

        self.state_msg = Int16()
        data = 999

        match self.state_machine.state:

            case AutonomousStates.INIT:
                data = self.INIT

            case AutonomousStates.MOVING_TO_GOAL:
                data = self.MOVING_TO_GOAL

            case AutonomousStates.AT_WAYPOINT:
                data = self.AT_WAYPOINT

            case AutonomousStates.AT_GHOST_WAYPOINT:
                data = self.AT_GHOST_WAYPOINT

            case AutonomousStates.MISSION_ACCOMPLISHED:
                data = self.MISSION_ACCOMPLISHED

            case AutonomousStates.ROBOT_STUCK:
                data = self.ROBOT_STUCK

            case AutonomousStates.PAUSED:
                data = self.PAUSED

        self.state_msg.data = data
        self.autonomous_state_pub.publish(self.state_msg)



    def check_if_at_waypoint(self, goal_reached):
        
        result = goal_reached
        # get sensors and goal reached
        # do something math with parameters
        # decide if in waypoint

        return result

    # --------------------------------------------------------------------------------
    #    State Machine
    # --------------------------------------------------------------------------------

    def machine_states(self):


        # -------------------------------------
        #    ROS Input
        # -------------------------------------

        no_more_waypoints_ros = self.generic_callback.get(self.mission_accomplished)
        goal_reached_ros = self.generic_callback.get(self.goal_reached)
        signalize_waypoint_ros = self.generic_callback.get(self.signalize_waypoint)
        autonomous_mode_ros = self.generic_callback.get(self.operation_mode)


        # -------------------------------------
        #    Filter
        # -------------------------------------

        # detect change mode rising edge
        goal_reached_rising_edge = (goal_reached_ros >  self.last_goal_reached) # add sensors
        at_waypoint = self.check_if_at_waypoint(goal_reached_rising_edge)

        autonomous_mode = (autonomous_mode_ros == self.operation_mode_AUTONOMOUS) 
        paused = not autonomous_mode

        following_ghost_waypoint = not signalize_waypoint_ros

        # -------------------------------------
        #    Machine State Input
        # -------------------------------------

        self.state_machine.no_more_waypoints = no_more_waypoints_ros
        self.state_machine.following_ghost_waypoint = following_ghost_waypoint
        self.state_machine.at_waypoint = at_waypoint
        self.state_machine.paused = paused
        
        self.state_machine.routine()

        # -------------------------------------
        #    Output
        # -------------------------------------

        self.publish_state()

        # -------------------------------------
        #    Handle internal variables
        # -------------------------------------

        self.last_goal_reached = goal_reached_ros

        self.get_logger().info(f"State: {self.state_machine.state}, autonomous_mode_ros: {autonomous_mode_ros}, operation_mode_AUTONOMOUS: {self.operation_mode_AUTONOMOUS}")



if __name__ == '__main__':
    rclpy.init()

    # Create a custom context for single thread and real-time execution
    states_context = rclpy.Context()
    states_context.init()
    states_context.use_real_time = True

    node = AutonomousStateMachineNode(
        node_name='autonomous_state_machine',
        cli_args='--debug',
        context=states_context,
        namespace='main_robot',
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
