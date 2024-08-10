import os
import launch_ros
import launch_ros.descriptions


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('fred2_machine_states'),
        'config',
        'params.yaml'
        )
    

    
    operation_modes_node = launch_ros.actions.Node(

        package='fred2_machine_states',
        executable='operation_modes_ros.py',
        name='operation_modes',
        namespace='main_robot', 
        parameters=[config]
    )

    robot_states_node = launch_ros.actions.Node(

        package='fred2_machine_states',
        executable='autonomous_state_machine_ros.py',
        name='autonomous_state_machine',
        namespace='main_robot', 
        parameters=[config]
    )



    return LaunchDescription([

        TimerAction(period= 1.5, actions= [
            
            LogInfo(msg=' ######################### LAUNCHING MACHINE STATES #################################### '), 
            # operation_modes_node,
            robot_states_node

        ])
    ])