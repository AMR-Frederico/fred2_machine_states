# Fred - Machine States 

**note: Package for ROS 2 - C++/Python (CMake) based package**

The fred2_machine_states package in ROS 2 is designed to manage various states of a robot, focusing on modes like MANUAL and AUTONOMOUS. The primary node, goal_mode.py, monitors joystick connectivity, collision alerts, and other safety parameters to determine the robot's state.

---

## Installation

**1. Clone the repository into your ROS2 workspace:**

```bash
cd ros2_ws/src
git clone https://github.com/AMRFrederico/fred2_machine_states.git
```

**2. Build the package:**

```bash
cd ros2_ws
colcon build
```

---

## Usage 

**Launch the package:**

```
ros2 launch fred2_machine_states machine_states_launch.yaml
```

---

## Goal mode node

The goal_mode.py node is the core component of the fred2_machine_states package. It manages the robot's mode (MANUAL or AUTONOMOUS) and state (EMERGENCY, IN GOAL, MISSION COMPLETED, MANUAL, or AUTONOMOUS) based on various inputs.

**Type:** `python` 

**Name:** `main_robot`

**Namespace:** `machine_states`


### Parameters: 

`manual`: index state for manual mode

`autonomous`: index state for autonomous mode

`in_goali`: index state when the robot reaches the goal 

`mission_completed`: index state when the robot finished the course

`emergency`: index state when the robot cannot receive speed commands

#### To check available parameters 
```
ros2 param list 
```

#### To check the parameter description
```
ros2 param describe /machine_states/main_robot <parameter name>

```

#### To check the parameter value
```
ros2 param get /machine_states/main_robot <parameter name>
```

#### To reload the parameters file
```
ros2 param load /machine_states/main_robot <path-to-parameter-config-file>
```


### Subscribers

- `/joy/machine_states/switch_mode`	(*std_msgs/Bool*): Switch between `MANUAL` and `AUTONOMOUS` mode.

- `/robot_safety` (*std_msgs/Bool*): Robot safety status, returns `True` when the robot is safe, and `False` when the robot is blocked by `safe twist node`

- `/goal_manager/goal/mission_completed` (*std_msgs/Bool*): Status of mission completion.

- `/goal_manager/goal/reached` (*std_msgs/Bool*): Status of reaching the goal.

- `/odom/reset`	(*std_msgs/Bool*): Reset odometry command.

</br>




### Publishers:

- `/machine_states/robot_state` (*std_msgs/Int16*): Current robot state

--- 

### Run 

**Default:**

```
ros2 run fred2_machine_states robot_states.py --ros-args --params-file /home/ubuntu/ros2_ws/src/fred2_machine_states/config/params.yaml
```

**Enable debug:**
```
ros2 run fred2_machine_states robot_states.py --ros-args --params-file /home/ubuntu/ros2_ws/src/fred2_machine_states/config/params.yaml --debug
```
---

### Launch

**Default:**
```
ros2 launch fred2_machine_states machine_states.launch.py 
```




---

