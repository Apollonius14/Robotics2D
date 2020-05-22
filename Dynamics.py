# Takes Actions, Current State, Returns New State

# Remember State [y,z, y_dot, z_dot,Psi, Psi_dot]

import numpy as np
import math

def dynamics(State, Params, Actions, t_step):


    # Take Desired Forces

    U = np.array(Actions).reshape(2,1)
    #print("Forces Before")
    #print (U)
    
    # Relate to individual motors
    
    Motor1 = 0.5 * (U[0] - U[1]/Params["L"])
    Motor2 = 0.5 * (U[0] + U[1]/Params["L"])

    # Calculate Individual motor forces

    Motors = [Motor1,Motor2]
    
    # Limit / Clamp The Motor Forces
    
    ClampedMotors = []
    for motor in Motors:
        if motor > Params["MaxF"]:
            motor = Params["MaxF"]
        if motor < Params["MinF"]:
            motor = Params["MinF"]
        ClampedMotors.append(motor)

    U[0] = ClampedMotors[0]+ClampedMotors[1]
    U[1] = (ClampedMotors[1]-ClampedMotors[0]) * Params["L"]
    
    
    # Get Accelerations in WORLD Fixed Axes. Note the second
    # order taylor series correction to estimate the position

    psi_ddot = U[1]/ Params["I"]
    State[5] += (psi_ddot * t_step)
    State[4] += (State[5] * t_step)+((psi_ddot)*t_step**2)/2

    
    z_ddot = (U[0]*math.cos(State[4])/Params["Mass"]) - 9.81
    State[3] = State[3] + (z_ddot * t_step)
    State[1] = State[1] + (State[3] * t_step)+ ((z_ddot)*t_step**2)/2

    
    y_ddot = -1*(U[0]*math.sin(State[4]))/Params["Mass"]
    State[2] += (y_ddot * t_step)
    State[0] += (State[2] * t_step) + ((y_ddot)*t_step**2)/2

    return State



    
