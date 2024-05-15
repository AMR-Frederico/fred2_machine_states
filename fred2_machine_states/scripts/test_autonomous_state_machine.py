from autonomous_state_machine import AutonomousStateMachine, AutonomousStates


def create_state_machine():

    return AutonomousStateMachine(lambda: print("State Machine started"))


# def test_callback(val):
#     """recept dict if value = False and sets to True

#     Args:
#         val (dict): dict with value == False
#     """
#     val["value"] = True


def test_init():

    state_machine = create_state_machine()
    assert state_machine.state == AutonomousStates.INIT 


def test_started():

    state_machine = create_state_machine()

    state_machine.routine()

    assert state_machine.state == AutonomousStates.MOVING_TO_GOAL


def test_moving_to_at_waypoint():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()

    # moving to goal -> at waypoint
    state_machine.at_waypoint = True
    state_machine.routine()

    assert state_machine.state == AutonomousStates.AT_WAYPOINT

def test_at_waypoint_callback():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()

    # moving to goal -> at waypoint
    state_machine.at_waypoint = True
    callback_works = {'value': False}

    def test_callback():
        """recept dict if value = False and sets to True

        Args:
            val (dict): dict with value == False
        """
        callback_works["value"] = True

    state_machine.at_waypoint_callback = test_callback
    state_machine.routine()


    assert callback_works["value"] == True
