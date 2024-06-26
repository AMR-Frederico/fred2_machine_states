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
        
        self.in_distance_waypoint_odom = None
        self.in_inductive_signal = None
        self.in_color_signal = None
        self._create_fuzzy_system()


    def set_rules(self):
        
        self.rules = []
        
        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.AT.value] & self.in_inductive_signal[Names.METAL.value], self.out_at_waypoint[Names.AT.value]))
        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR.value] & self.in_inductive_signal[Names.METAL.value], self.out_at_waypoint[Names.AT.value]))
        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.FAR.value] & self.in_inductive_signal[Names.METAL.value], self.out_at_waypoint[Names.AT.value]))

        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.AT.value] & self.in_inductive_signal[Names.NOT_METAL.value], self.out_at_waypoint[Names.NEAR.value]))
        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR.value] & self.in_inductive_signal[Names.NOT_METAL.value], self.out_at_waypoint[Names.FAR.value]))
        # self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.FAR.value] & self.in_inductive_signal[Names.NOT_METAL.value], self.out_at_waypoint[Names.FAR.value]))

        # self.rules.append(ctrl.Rule(self.in_color_signal[Names.Y_100.value] & (self.in_distance_waypoint_odom[Names.NEAR.value] | self.in_distance_waypoint_odom[Names.AT.value]), self.out_at_waypoint[Names.AT.value]))
        # self.rules.append(ctrl.Rule(self.in_color_signal[Names.Y_100.value] & self.in_inductive_signal[Names.METAL.value], self.out_at_waypoint[Names.AT.value]))

        self.rules.append(ctrl.Rule(self.in_inductive_signal[Names.METAL.value], self.out_at_waypoint[Names.AT.value]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR.value] & self.in_color_signal[Names.Y_100.value], self.out_at_waypoint[Names.AT.value]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.NEAR.value] & self.in_color_signal[Names.Y_0.value], self.out_at_waypoint[Names.NEAR.value]))
        self.rules.append(ctrl.Rule(self.in_distance_waypoint_odom[Names.FAR.value] & self.in_color_signal[Names.Y_100.value], self.out_at_waypoint[Names.NEAR.value]))

    def decide(self, inductive_read, color_read, odom_distance):
        
        at_waypoint_system = ctrl.ControlSystem(self.rules)

        at_waypoint = ctrl.ControlSystemSimulation(at_waypoint_system)

        # for input in at_waypoint.input.sim.ctrl.antecedents:

        #     print(f"Antecedent: {input}")

        at_waypoint.input[Names.D_WAYPOINT_ODOM.value] = odom_distance
        at_waypoint.input[Names.INDUCTIVE_SIGNAL.value] = inductive_read
        at_waypoint.input[Names.COLOR_SIGNAL.value] = color_read

        at_waypoint.compute()

        # (debug)
        # print(at_waypoint.output[Names.AT_WAYPOINT.value])
        self.out_at_waypoint.view(sim=at_waypoint)
        self.in_distance_waypoint_odom.view()
        self.in_color_signal.view()
        self.in_inductive_signal.view()
        input()

        return at_waypoint.output[Names.AT_WAYPOINT.value]


    def _create_fuzzy_system(self):

        self._define_antecedent()
        self._define_consequent()
        self._populate_universe()


    def _define_antecedent(self):

        self.in_distance_waypoint_odom = ctrl.Antecedent(np.arange(0, 101, 1), Names.D_WAYPOINT_ODOM.value)
        self.in_inductive_signal = ctrl.Antecedent(np.arange(0, 1.1, 0.1), Names.INDUCTIVE_SIGNAL.value)
        self.in_color_signal = ctrl.Antecedent(np.arange(0, 101, 1), Names.COLOR_SIGNAL.value)



    def _define_consequent(self):
        
        self.out_at_waypoint = ctrl.Consequent(np.arange(0, 101, 1), Names.AT_WAYPOINT.value)


    def _populate_universe(self):

        # automf(Ok, ... , Not Ok)
        # self.in_distance_waypoint_odom.automf(number=3, names=[Names.AT.value, Names.NEAR.value, Names.FAR.value])
        # self.in_inductive_signal.automf(number=2, names=[Names.METAL.value, Names.NOT_METAL.value])
        # self.in_color_signal.automf(number=2, names=[Names.Y_100.value, Names.Y_0.value])

        self.in_distance_waypoint_odom.automf(number=3, names=[Names.FAR.value, Names.NEAR.value, Names.AT.value])
        self.in_inductive_signal.automf(number=2, names=[Names.NOT_METAL.value, Names.METAL.value])
        self.in_color_signal.automf(number=2, names=[Names.Y_0.value, Names.Y_100.value])

        self.out_at_waypoint[Names.FAR.value] = fuzz.trimf(self.out_at_waypoint.universe, [0, 0, 50])
        self.out_at_waypoint[Names.NEAR.value] = fuzz.trimf(self.out_at_waypoint.universe, [0, 50, 100])
        self.out_at_waypoint[Names.AT.value] = fuzz.trimf(self.out_at_waypoint.universe, [50, 100, 100])





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

if __name__ == "__main__":

    fuzzy_sys = SensorsFuzzySystem()

    fuzzy_sys.set_rules()

    result = fuzzy_sys.decide(1, 1, 60) 
    # result = fuzzy_sys.decide(0, 1, 100) 
    # result = fuzzy_sys.decide(1, 0, 100) 
    # result = fuzzy_sys.decide(1, 0, 0) 
    # result = fuzzy_sys.decide(0, 1, 0) 
    # result = fuzzy_sys.decide(1, 1, 0) 
    # result = fuzzy_sys.decide(0, 0, 0) 
    print(result)