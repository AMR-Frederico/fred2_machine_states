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

class SensorsFuzzySystem():

    def __init__(self):
        
        self._create_fuzzy_system()


    def set_rules(self):
        
        self.rules = []

        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.AT] and self.in_inductive_signal[Names.METAL], self.out_at_waypoint[Names.AT]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR] and self.in_inductive_signal[Names.METAL], self.out_at_waypoint[Names.AT]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.FAR] and self.in_inductive_signal[Names.METAL], self.out_at_waypoint[Names.FAR]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.AT] and self.in_inductive_signal[Names.NOT_METAL], self.out_at_waypoint[Names.NEAR]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR] and self.in_inductive_signal[Names.NOT_METAL], self.out_at_waypoint[Names.FAR]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.FAR] and self.in_inductive_signal[Names.NOT_METAL], self.out_at_waypoint[Names.FAR]))


    def decide(self, inductive_read, color_read, odom_distance):

        at_waypoint_system = ctrl.ControlSystem(self.rules)

        at_waypoint = ctrl.ControlSystemSimulation(at_waypoint_system)
        at_waypoint[Names.INDUCTIVE_SIGNAL] = inductive_read
        at_waypoint[Names.COLOR_SIGNAL] = color_read
        at_waypoint[Names.D_WAYPOINT_ODOM] = odom_distance

        at_waypoint.compute()

        # (debug)
        print(at_waypoint.output[Names.AT_WAYPOINT])
        self.out_at_waypoint.view(sim=at_waypoint)

        return at_waypoint.output[Names.AT_WAYPOINT]


    def _create_fuzzy_system(self):

        self._define_antecedent()
        self._define_consequent()
        self._populate_universe()


    def _define_antecedent(self):

        self.in_distance_waypoint_odom = ctrl.Antecedent(np.arange(0, 100, 0.1), Names.D_WAYPOINT_ODOM)
        self.in_inductive_signal = ctrl.Antecedent(np.arange(0, 2, 1), Names.INDUCTIVE_SIGNAL)
        self.in_color_signal = ctrl.Antecedent(np.arange(0, 2, 0.01), Names.COLOR_SIGNAL)


    def _define_consequent(self):
        
        self.out_at_waypoint = ctrl.Consequent(np.arange(0, 100, 1), Names.AT_WAYPOINT)


    def _populate_universe(self):

        # automf(Ok, ... , Not Ok)
        self.in_distance_waypoint_odom.automf(number=3, names=[Names.AT, Names.NEAR, Names.FAR])
        self.in_inductive_signal.automf(number=2, names=[Names.METAL, Names.NOT_METAL])
        self.in_color_signal.automf(number=2, names=[Names.Y_100, Names.Y_0])

        self.out_at_waypoint[Names.FAR] = fuzz.trimf(self.out_at_waypoint.universe, [0, 0, 50])
        self.out_at_waypoint[Names.NEAR] = fuzz.trimf(self.out_at_waypoint.universe, [0, 50, 100])
        self.out_at_waypoint[Names.AT] = fuzz.trimf(self.out_at_waypoint.universe, [50, 100, 100])





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