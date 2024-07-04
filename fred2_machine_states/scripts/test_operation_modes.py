from operation_modes import OperationModesStateMachine, OperationStates


def create_state_machine():

    return OperationModesStateMachine(lambda: print("State Machine started"))


def test_init():

    state_machine = create_state_machine()
    assert state_machine.state == OperationStates.INIT 


def test_started():

    state_machine = create_state_machine()

    state_machine.routine() # init
    state_machine.robot_safety = True
    state_machine.switch_mode = False

    state_machine.routine() # manual

    assert state_machine.state == OperationStates.MANUAL_MODE


def test_manual_to_autonomous():

    state_machine = create_state_machine()
    
    state_machine.robot_safety = True
    state_machine.routine() # Init
    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine() # manual
    
    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine() # autonomous

    assert state_machine.state == OperationStates.AUTONOMOUS_MODE


def test_autonomous_to_manual():

    state_machine = create_state_machine()

    state_machine.robot_safety = True
    state_machine.routine()
    
    state_machine.robot_safety = True
    state_machine.switch_mode = True
    
    state_machine.routine()
    
    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine()
    
    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine()

    assert state_machine.state == OperationStates.MANUAL_MODE


def test_init_to_emergency():

    state_machine = create_state_machine()
    

    state_machine.robot_safety = False
    state_machine.switch_mode = False

    state_machine.routine()

    assert state_machine.state == OperationStates.EMERGENCY_MODE



def test_manual_to_emergency():

    state_machine = create_state_machine()
    
    state_machine.robot_safety = False
    state_machine.switch_mode = True

    state_machine.routine()

    state_machine.robot_safety = False
    state_machine.switch_mode = False

    state_machine.routine()

    assert state_machine.state == OperationStates.EMERGENCY_MODE



def test_autonomous_to_emergency():

    state_machine = create_state_machine()

    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine()
    
    state_machine.robot_safety = True
    state_machine.switch_mode = True

    state_machine.routine()

    state_machine.robot_safety = False
    state_machine.switch_mode = True

    state_machine.routine()

    assert state_machine.state == OperationStates.EMERGENCY_MODE