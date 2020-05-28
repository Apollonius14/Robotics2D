# 2D Drone simulator demonstraing proportional derivative control
# User can select pre-programmed trajectories or use an event
# Handler to connect a joystick or keyboard to create real time
# Simulations. Using MATPLOTLIB to visualise drone in 2D and
# Show step response of transfer function

# Author Omar Kadhim May 2020

# Simplified Drone Model looking at a fixed x plane
# Eliminate Motors 2 and 4, just motor F1, F3

# _________________________________________________________________________
#    Definitions 
#
#    State 1x6 is defined by --Position-- [y,z, y_dot, z_dot,Psi, Psi_dot] in *INERTIAL* Axes
#
#    Demanded 1x8 State is as above with subscript d e.g. Position_d = [y_d,....] in *INERTIAL* Axes
#
#    The Demanded State includes desired accelerations y_ddot_d and z_ddot_d as these are needed
#    by the controller to follow a trajectory
#    
#    State is updated in the Dynamics Module after the Actions are Calculated by the Controller
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
# _________________________________________________________________________

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

    def __init__(self,start,end):

    # ---Initialise Simulation Parameters--- #

        # self.t and self.t_step are simulation time, not real time. I.e.
        # it may take the computer 0.5 seconds to solve the simulation
        # up to 0.1 seconds because of computations in the Controller
        # and Dynamics module. It may also take the plotter 0.5 seconds
        # to plot only 0.1 seconds of simulation because of rendering delays
        # so these parameters are not suitable for real-time visualisation
        
        self.t = 0
        self.t_step = 0.02

        # These are the real times we would like to simulate between and plot
        # Later

        self.start = start
        self.end = end

        # ---Set Drone Parameters and Controller Gains--- #

        self.Params, self.Gains  = PRM.get_params()
    
        # ---Initialise Plot--- #
    
        # Figure Setup    
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-0.2,1.2,auto = False)
        self.ax.set_ylim(-0.5,0.5, auto = False)
        self.traj_line, = self.ax.plot([],[],"m",lw=2)

        # Trajectory Histories
        self.t_vec = []
        self.y_vec = []
        self.y_des_vec=[]
        self.z_vec = []
        self.z_des_vec = []
        self.phi_vec = []

        # Small representation of drone (coordinates of an upside down capital T...)
        self.base_body = np.array([[0,0],
                                   [0,self.Params["L"]/2],
                                   [-1*self.Params["L"]/2,0],
                                   [self.Params["L"]/2,0]])
        
        
        self.head, = self.ax.plot([],[])
        self.wing, = self.ax.plot([],[],lw=3,solid_capstyle='round')

        # Drone Initial State
        self.State = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.State_d = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.PhiC_last = 0

    # ---Define Input Method (Later)--- #

    def terminate(self):
        plt.close("all")

    # ---Solve the Equations of Motion ---#

    def simulate(self):

        self.t = self.start
        
        while self.t < self.end:
        
            self.t+= self.t_step

            # Desired Trajectory [From Input or Trajectory] and State [From Dynamics]
            self.State_d = TR.trajectory(self.t,self.State_d)
    
            # Calculate Actions and Forces from Controller
            self.PhiC_last, Actions = C2D.control(self.PhiC_last,self.State,
                          self.State_d,self.Gains,self.Params,self.t_step)


            # Use Dynamics to Simulate New State [Transform Axes, Estimate Derivatives
            self.State = DYN.dynamics(self.State,self.Params,Actions, self.t_step)

            ## Calculation time for above measured at around 0.5 milliseconds on average
        
            # Record trajectory against simulation time
            self.t_vec.append(self.t)
    
            self.y_vec.append(self.State[0])

            self.z_vec.append(self.State[1])

            self.phi_vec.append(self.State[4])

            self.y_des_vec.append(self.State_d[0])

            self.z_des_vec.append(self.State_d[1])


    # --- Yield the state of the drone from the motion history -- #

    def get_frame(self,i):

        # Actual time that has elapsed in the simulation
        
        realtime = time.time()-self.mark
        
        if realtime > self.end:
            self.terminate()

        # Our simulation was linear in time, interpolate for
        # the index of the time and trajectory to plot until

        index = min(int((realtime/self.end)*len(self.t_vec)),len(self.t_vec))

        # Also now show the drone at the required position
        
        self.traj_line.set_data(self.y_vec[:index],self.z_vec[:index])

        # Also now show the drone at the required position

        theta = self.phi_vec[index]

        attitude2D = np.array(((math.cos(theta), -1*math.sin(theta)),
                               (math.sin(theta), math.cos(theta))))

        self.body = attitude2D.dot(self.base_body.T)
        
        self.body = self.body + np.array([[self.y_vec[index-1]],[self.z_vec[index-1]]])

        base = self.body[:,0]
        
        top = self.body[:,1]
        
        wingL = self.body[:,2]
        
        wingR = self.body[:,3]
                
        self.head.set_data([base[0],top[0]],[base[1],top[1]])

        self.wing.set_data([wingL[0],wingR[0]],[wingL[1],wingR[1]])
        
        return self.traj_line, self.head, self.wing
        

        
    # Now, having solve tfor the equations of motion and the
    # trajectory of the drone, we can replay it in real time
    # at say 30 FPS (33 millisecond intervals)
    
    def replay(self):

        self.mark = time.time()

        self.ax.plot(self.y_des_vec,self.z_des_vec,"r")
        
        self.ani = animation.FuncAnimation(self.fig, self.get_frame, interval = 100)        

        plt.show()


def main():

    # Initialise a Drone Simulation Instance of 20 second simulatino duration
    droney = TwoD_drone(0,20)

    # Solve the equations of motions
    droney.simulate()

    # Replay the simulation in close to real-time
    droney.replay()

if __name__ == '__main__':
    main()
    
    
    

