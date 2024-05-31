import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from enum import Enum


# --------------------------------------------------------------------------------
#    Problema
# --------------------------------------------------------------------------------
#   Sensor indutivo -> 0, 1
#   Sensor de cor -> RGBA []
#   Odometria -> posicao [x, y, yaw]
#

class Names(Enum):

    # Spaces
    D_WAYPOINT_ODOM = "d_waypoint_odom"
    INDUCTIVE_SIGNAL = "inductive_signal"
    COLOR_SIGNAL = "color_signal"
    AT_WAYPOINT = "at_waypoint"
    
    # Qualities
    AT = "at"
    NEAR = "near"
    FAR = "far"

    METAL = "metal"
    NOT_METAL = "not_metal"

    Y_100 = "y 100%"
    Y_0 = "y 0%"

in_distance_waypoint_odom = ctrl.Antecedent(np.arange(0, 100, 0.1), Names.D_WAYPOINT_ODOM)
in_inductive_signal = ctrl.Antecedent(np.arange(0, 2, 1), Names.INDUCTIVE_SIGNAL)
in_color_signal = ctrl.Antecedent(np.arange(0, 2, 0.01), Names.COLOR_SIGNAL)

out_at_waypoint = ctrl.Consequent(np.arange(0, 100, 1), Names.AT_WAYPOINT)

# automf(Ok, ... , Not Ok)
in_distance_waypoint_odom.automf(number=3, names=[Names.AT, Names.NEAR, Names.FAR])
in_inductive_signal.automf(number=2, names=[Names.METAL, Names.NOT_METAL])
in_color_signal.automf(number=2, names=[Names.Y_100, Names.Y_0])

out_at_waypoint[Names.FAR] = fuzz.trimf(out_at_waypoint.universe, [0, 25, 50])
out_at_waypoint[Names.NEAR] = fuzz.trimf(out_at_waypoint.universe, [25, 50, 75])
out_at_waypoint[Names.AT] = fuzz.trimf(out_at_waypoint.universe, [50, 75, 100])


# in_distance_waypoint_odom.view()
# in_inductive_signal.view()
# in_color_signal.view()

rules = []

rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.AT] and in_inductive_signal[Names.METAL], out_at_waypoint[Names.AT]))
rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.NEAR] and in_inductive_signal[Names.METAL], out_at_waypoint[Names.AT]))
rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.FAR] and in_inductive_signal[Names.METAL], out_at_waypoint[Names.FAR]))
rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.AT] and in_inductive_signal[Names.NOT_METAL], out_at_waypoint[Names.NEAR]))
rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.NEAR] and in_inductive_signal[Names.NOT_METAL], out_at_waypoint[Names.FAR]))
rules.append(ctrl.Rule(in_distance_waypoint_odom[Names.FAR] and in_inductive_signal[Names.NOT_METAL], out_at_waypoint[Names.FAR]))

# out_at_waypoint.view()

# at_waypoint_system = ctrl.ControlSystem(rules)

# at_waypoint = ctrl.ControlSystemSimulation(at_waypoint_system)
r = ctrl.Rule(in_distance_waypoint_odom[Names.AT] and in_inductive_signal[Names.METAL], out_at_waypoint[Names.AT])
r.view()

input("Wait")

def color_to_value(color):

    color_sum = color[0] + color[1] - color[2]
    
    return color_sum        


def bool_to_value(bool_in):

    if bool_in : 

        return 1

    else:

        return 0
    

def check_if_at_waypoint(goal_reached, color_sensor, inductive_sensor):

    # gains
    odom_gain = 1
    color_sensor_gain = 1
    inductive_sensor_gain = 1

    # conversions
    color_read = color_to_value(color_sensor) # value
    inductive_read = bool_to_value(inductive_sensor) # 1 
    odom_read = bool_to_value(goal_reached) # 1 -> seria melhor pegar a posição e fazer a posição - posição do ponto,
                                            # porém problema do futuro

    result = odom_gain * odom_read + color_sensor_gain * color_read + inductive_sensor_gain * inductive_read

    return result