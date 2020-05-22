import numpy as np
import math

def trajectory(t,State_d):

    if t < 0.2:
        State_d[0] = 0
    else:
        State_d[0] = 1 - math.exp(-2*t)
        
        
        

    #t_max = 4
    #t = max(0, min(t,t_max))
    #t = t/t_max

    #y = 10*t**3 - 15*t**4 + 6*t**5
    #ydot = (30/t_max)*t**2 - (60/t_max)*t**3 + (30/t_max)*t**4
    #yddot = (60/t_max**2)*t - (180/t_max**2)*t**2 + (120/t_max**2)*t**3

    #State_d[0] = y
    #State_d[2] = ydot
    #State_d[6] = yddot

    #State_d[1] = y*t
    #State_d[3] = ydot*0.25*t
    #State_d[7] = yddot*0.25*t

    return State_d

    
    
