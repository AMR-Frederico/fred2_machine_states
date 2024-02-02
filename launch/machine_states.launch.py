import launch_ros
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo


def generate_launch_description():
    
    
    robot_states_node = launch_ros.actions.Node(

        package='fred2_machine_states',
        executable='robot_states.py',
        name='main_robot',
        namespace='machine_states'
    )


    return LaunchDescription([

        TimerAction(period= 1.5, actions= [
            
            LogInfo(msg=' ######################### LAUNCHING MACHINE STATES #################################### '), 
            robot_states_node
        ])
    ])