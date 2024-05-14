from operation_modes import OperationModesStateMachine, OperationStates


def create_state_machine():

    return OperationModesStateMachine(lambda: print("State Machine started"))



def test_init():

    state_machine = create_state_machine()
    assert state_machine.state == OperationStates.INIT 


def test_started():

    state_machine = create_state_machine()

    switch_mode = False
    robot_safe = True

    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.MANUAL_MODE


def test_manual_to_autonomous():

    state_machine = create_state_machine()
    
    switch_mode = True
    robot_safe = True

    state_machine.machine_states_routine(switch_mode, robot_safe)
    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.AUTONOMOUS_MODE


def test_autonomous_to_manual():

    state_machine = create_state_machine()
    
    switch_mode = True
    robot_safe = True

    state_machine.machine_states_routine(switch_mode, robot_safe)
    state_machine.machine_states_routine(switch_mode, robot_safe)
    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.MANUAL_MODE


def test_init_to_emergency():

    state_machine = create_state_machine()
    
    switch_mode = False
    robot_safe = False

    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.EMERGENCY_MODE


def test_manual_to_emergency():

    state_machine = create_state_machine()
    
    switch_mode = False
    robot_safe = True

    state_machine.machine_states_routine(switch_mode, robot_safe)

    robot_safe = False

    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.EMERGENCY_MODE


def test_autonomous_to_emergency():

    state_machine = create_state_machine()
    
    switch_mode = True
    robot_safe = True

    state_machine.machine_states_routine(switch_mode, robot_safe)
    state_machine.machine_states_routine(switch_mode, robot_safe)
    state_machine.machine_states_routine(switch_mode, robot_safe)

    assert state_machine.state == OperationStates.EMERGENCY_MODE