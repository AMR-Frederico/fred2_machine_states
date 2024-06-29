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