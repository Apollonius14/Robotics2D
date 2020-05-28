# 2D Controller takes error and returns Actions [U1, U2]

def control(PhiC_last,State, State_d, Gains, Params,t_step):

    # The Drone is not fully actuated in all directions
    # any translation demands must first be turned into
    # rotation commands that can then be used to define
    # Actions
    # Attitude_Err = Error[1]
    # Rotation Command PhiC
    # Need to get last PhiC
    # Remember State [y,z, y_dot, z_dot,Psi, Psi_dot]

    PhiC = (-1/9.81) * (State_d[6]+Gains["kyd"]*(State_d[2]-State[2])+
                        Gains["kyp"]*(State_d[0]-State[0]))

    #print ("Demanded Tilt")
    #print (PhiC)
    
    PhiC_dot = (PhiC - PhiC_last)/t_step
    
    U2 = Gains["kPhid"]*(PhiC_dot-State[5]) + Gains["kPhip"]*(PhiC-State[4])

    U1 = Params["Mass"]*(State_d[7]+9.81+Gains["kzd"]*(State_d[3]-State[3])+Gains["kzp"]*(State_d[1]-State[1]))

    PhiC_last = PhiC
    
    return PhiC_last,[U1,U2]
