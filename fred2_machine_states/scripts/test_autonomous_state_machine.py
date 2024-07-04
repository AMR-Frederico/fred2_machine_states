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

# --------------------------------------------------------------------------------
#    Moving
# --------------------------------------------------------------------------------

def test_init_to_moving_to_goal():

    state_machine = create_state_machine()

    state_machine.routine()
    state_machine.routine()

    assert state_machine.state == AutonomousStates.MOVING_TO_GOAL



def test_moving_to_goal_callback():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()

    # moving to goal -> at waypoint
    callback_works = {'value': False}

    def test_callback():
        """recept dict if value = False and sets to True

        Args:
            val (dict): dict with value == False
        """
        callback_works["value"] = True

    state_machine.moving_to_goal_callback = test_callback
    state_machine.routine()

# --------------------------------------------------------------------------------
#    Waypoint
# --------------------------------------------------------------------------------
def test_moving_to_at_waypoint():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at waypoint
    state_machine.at_waypoint = True
    state_machine.routine()

    assert state_machine.state == AutonomousStates.AT_WAYPOINT


def test_at_waypoint_callback():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
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

# --------------------------------------------------------------------------------
#    Ghost waypoint
# --------------------------------------------------------------------------------

def test_moving_to_at_ghost_waypoint():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    state_machine.routine()

    assert state_machine.state == AutonomousStates.AT_GHOST_WAYPOINT




def test_at_ghost_waypoint_callback():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at ghost waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    callback_works = {'value': False}

    def test_callback():
        """recept dict if value = False and sets to True

        Args:
            val (dict): dict with value == False
        """
        callback_works["value"] = True

    state_machine.at_ghost_waypoint_callback = test_callback
    state_machine.routine()


    assert callback_works["value"] == True


# --------------------------------------------------------------------------------
#    Mission Accomplished
# --------------------------------------------------------------------------------

def test_at_waypoint_to_mission_accomplished():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at waypoint
    state_machine.at_waypoint = True
    state_machine.no_more_waypoints = True
    state_machine.routine()

    # at waypoint -> mission accomplished
    state_machine.routine()

    assert state_machine.state == AutonomousStates.MISSION_ACCOMPLISHED


def test_at_ghost_waypoint_to_mission_accomplished():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at ghost waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    state_machine.no_more_waypoints = True
    state_machine.routine()

    # at ghost waypoint -> mission accomplished
    state_machine.routine()

    assert state_machine.state == AutonomousStates.MISSION_ACCOMPLISHED

def test_mission_accomplished_to_wait_reset_ack():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at ghost waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    state_machine.no_more_waypoints = True
    state_machine.routine()

    # at ghost waypoint -> mission accomplished
    state_machine.routine()

    # mission accomplished -> paused
    state_machine.odom_reset = True
    state_machine.routine()

    assert state_machine.state == AutonomousStates.WAIT_RESET_ACK


def test_mission_accomplished_to_wait_ack_to_paused():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at ghost waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    state_machine.no_more_waypoints = True
    state_machine.routine()

    # at ghost waypoint -> mission accomplished
    state_machine.routine()

    # mission accomplished -> WAIT
    state_machine.odom_reset = True
    state_machine.routine()

    # wait reset ack -> paused
    state_machine.paused = True
    state_machine.routine()

    assert state_machine.state == AutonomousStates.PAUSED


def test_mission_accomplished_to_wait_ack_to_paused_to_goal():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> at ghost waypoint
    state_machine.at_waypoint = True
    state_machine.following_ghost_waypoint = True
    state_machine.no_more_waypoints = True
    state_machine.routine()

    # at ghost waypoint -> mission accomplished
    state_machine.routine()

    # mission accomplished -> WAIT
    state_machine.odom_reset = True
    state_machine.routine()

    # wait reset ack -> paused
    state_machine.paused = True
    state_machine.routine()

    # paused -> goal
    state_machine.paused = False
    state_machine.routine()

    assert state_machine.state == AutonomousStates.MOVING_TO_GOAL



def test_mission_accomplished_callback():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    
    state_machine.at_waypoint = True
    state_machine.no_more_waypoints = True
    callback_works = {'value': False}

    def test_callback():
        """recept dict if value = False and sets to True

        Args:
            val (dict): dict with value == False
        """
        callback_works["value"] = True

    state_machine.mission_accomplished_callback = test_callback
    
    # moving to goal -> at waypoint
    state_machine.routine()

    # at waypoint -> mission accomplished
    state_machine.routine()

    assert callback_works["value"] == True
    

# --------------------------------------------------------------------------------
#    Stuck
# --------------------------------------------------------------------------------


def test_moving_to_stuck():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> stuck
    state_machine.robot_stuck = True
    state_machine.routine()


    assert state_machine.state == AutonomousStates.ROBOT_STUCK


def test_back_from_stuck():

    state_machine = create_state_machine()

    # init -> moving to goal
    state_machine.routine()
    state_machine.routine()

    # moving to goal -> stuck
    state_machine.robot_stuck = True
    state_machine.routine()
    state_machine.routine()
    state_machine.routine()
    state_machine.routine()

    state_machine.robot_stuck = False
    state_machine.routine()


    assert state_machine.state == AutonomousStates.MOVING_TO_GOAL