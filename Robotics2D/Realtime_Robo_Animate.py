# 2D Drone simulator demonstraing proportional derivative control
# User can select pre-programmed trajectories or use an event
# Handler to connect a joystick or keyboard to create real time
# Simulations. Using MATPLOTLIB to visualise drone in 2D and
# Show step response of transfer function

# Author Omar Kadhim May 2020

# Simplified Drone Model looking at a fixed x plane
# Eliminate Motors 2 and 4, just motor F1, F3

# ---Definitions--- 
#
#   State 1x6 is defined by --Position-- [y,z, y_dot, z_dot,Psi, Psi_dot] in *INERTIAL* Axes
#
#    Demanded 1x8 State is as above with subscript d e.g. Position_d = [y_d,....] in *INERTIAL* Axes
#
#    The Demanded State includes y_ddot_d and z_ddot_d as these are needed by the controller
#    to follow a trajectory
#    
#    State is updated in the Dynamics Module after the Actions are Calculated
#                       
#    --Actions-- are calculated from the controller
#    into the vector [U1, U2]
#
#    --Forces-- Required into the Motors are a linear
#    Transformation from [U1, U2] into [F1, F3]
#
#    Remember we are assuming a 2D representation of
#    a drone looking at it normally from the x plane
#
#    ---Gains---
#
#    ---Params---
#    
# ------------------#


# ---Imports--- #
import matplotlib
import numpy as np
import math
import time
import Controller2D as C2D
import Dynamics as DYN
import Trajectory as TR
import Parameters as PRM
import matplotlib.pyplot as plt
from matplotlib import animation


class TwoD_drone:

    def __init__(self):

    # ---Initialise Simulation Plot--- #
        self.t = 0
        self.t_step = 0.022
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0,5,auto = False)
        self.ax.set_ylim(0,1.5, auto = False)
        self.line, = self.ax.plot([],[])
        self.t_vec = []
        self.p_vec = []
        self.p_des_vec=[]

    # ---Set Drone Parameters and Controller Gains--- #
        self.Params, self.Gains  = PRM.get_params()


    #--- Drone Initial State --- #
        self.State = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.State_d = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.PhiC_last = 0

    # ---Define Input Method (Later)--- #

    def terminate(self):
        plt.close("all")

    def get_update(self,i):

        start = time.time()
        
        self.t+= self.t_step

        # Desired Trajectory [From Input or Trajectory] and State [From Dynamics]
        self.State_d = TR.trajectory(self.t,self.State_d)
    
        # Calculate Actions and Forces from Controller
        self.PhiC_last, Actions = C2D.control(self.PhiC_last,self.State,
                          self.State_d,self.Gains,self.Params,self.t_step)


        # Use Dynamics to Simulate New State [Transform Axes, Estimate Derivatives
        self.State = DYN.dynamics(self.State,self.Params,Actions, self.t_step)

        ## Calculation time for above measured at around 0.5 milliseconds on average
        end = time.time()

        print ("Simulation time")
        print (end-start)
        
        # Update Plot
        self.t_vec.append(self.t)
    
        self.p_vec.append(self.State[0])

        self.p_des_vec.append(self.State_d[0])
        
        start = time.time()

        self.line.set_data(self.t_vec,self.p_vec)
        
        #self.fig.canvas.draw()

        self.fig.canvas.blit(self.ax.bbox)

        end = time.time()

        print ("Plotting time")
        print (end-start)

        if self.t > 10:
            self.terminate()
        
        return self.line,


    # the python __call__ method is inbuilt to classes
    # and allows them to be called as functions, in
    # which case they execute the below code using their
    # local variabls and others passed to them via
    # events, callbacks, pipes or queues
    
    def __call__(self):

        self.ani = animation.FuncAnimation(self.fig, self.get_update,interval = 3)        

        plt.show()


def main():

    droney = TwoD_drone()
    droney()
    plt.show()

if __name__ == '__main__':
    main()
    
    
    

