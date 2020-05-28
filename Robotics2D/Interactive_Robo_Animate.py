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
        self.t_step = 0.02
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-1,1,auto = False)
        self.ax.set_ylim(-1,1, auto = False)
        self.line_actual, = self.ax.plot([],[])
        self.line_demand, = self.ax.plot([],[])
        self.t_vec = []
        self.y_vec = []
        self.y_des_vec=[]
        self.z_vec = []
        self.z_des_vec = []

    # ---Set Drone Parameters and Controller Gains--- #
        self.Params, self.Gains  = PRM.get_params()

    # Small representation of drone (coordinates of an upside down capital T...)
        self.base_body = np.array([[0,0],
                                   [0,self.Params["L"]/2],
                                   [-1*self.Params["L"]/2,0],
                                   [self.Params["L"]/2,0]])
        
        
        self.head, = self.ax.plot([],[])
        self.wing, = self.ax.plot([],[],lw=3,solid_capstyle='round')
    

    #--- Drone Initial State --- #
        self.State = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.State_d = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.PhiC_last = 0

    # ---Define Input Method (Later)--- #

    def terminate(self):
        plt.close("all")

    def key_in(self,event):

        if event.key == "right":
            self.State_d[0] += 2*self.t_step**3
            self.State_d[2] += 6*self.t_step**2
            self.State_d[6] += 12*self.t_step
        if event.key == "left":
            self.State_d[0] -= 2*self.t_step**3
            self.State_d[2] -= 6*self.t_step**2
            self.State_d[6] -= 12*self.t_step
        if event.key == "up":
            self.State_d[1] += 1/100
        if event.key == "down":
            self.State_d[1] -= 1/100
        

    def get_update(self,i):

        
        start = time.time()
        
        self.t+= self.t_step

    
        # Calculate Actions and Forces from Controller
        self.PhiC_last, Actions = C2D.control(self.PhiC_last,self.State,
                          self.State_d,self.Gains,self.Params,self.t_step)


        # Use Dynamics to Simulate New State [Transform Axes, Estimate Derivatives
        self.State = DYN.dynamics(self.State,self.Params,Actions, self.t_step)

        ## Calculation time for above measured at around 0.5 milliseconds on average
        
        # Update Plot
        self.t_vec.append(self.t)
    
        self.y_vec.append(self.State[0])

        self.z_vec.append(self.State[1])

        self.y_des_vec.append(self.State_d[0])

        self.z_des_vec.append(self.State_d[1])

        end = time.time()

        theta = self.State[4]

        attitude2D = np.array(((math.cos(theta), -1*math.sin(theta)),
                               (math.sin(theta), math.cos(theta))))

        self.body = attitude2D.dot(self.base_body.T)
        
        self.body = self.body + np.array([[self.State[0]],[self.State[1]]])

        base = self.body[:,0]
        
        top = self.body[:,1]
        
        wingL = self.body[:,2]
        
        wingR = self.body[:,3]
                
        self.head.set_data([base[0],top[0]],[base[1],top[1]])

        self.wing.set_data([wingL[0],wingR[0]],[wingL[1],wingR[1]])

        print ("simulation time")
        print (end-start)

        self.line_actual.set_data(self.y_vec,self.z_vec)

        if self.t > 20:
            self.terminate()
        
        return self.line_actual, self.head, self.wing


    # the python __call__ method is inbuilt to classes
    # and allows them to be called as functions, in
    # which case they execute the below code using their
    # local variabls and others passed to them via
    # events, callbacks, pipes or queues
    
    def __call__(self):

        self.fig.canvas.mpl_connect('key_press_event',self.key_in)
        
        self.ani = animation.FuncAnimation(self.fig, self.get_update,interval = 50)        

        plt.show()


def main():

    droney = TwoD_drone()
    droney()
    plt.show()

if __name__ == '__main__':
    main()
    
    
    

