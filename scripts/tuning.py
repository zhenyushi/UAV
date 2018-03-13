#!/usr/bin/env python
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
import xml.etree.cElementTree



def roll_pitch_hold():
    
    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max

    time_step = 0.01 
    theta_c = 20
    x_start = 0
    x_end = 10
    y_start = theta_c+0.5*theta_c
    y_end = -1*y_start
    plt.figure(figsize=(20,12))
    num = int((x_end-x_start)/time_step)
    t_signal = np.linspace(x_start, x_end-1, num-100, endpoint=False)
    t = np.linspace(x_start, x_end, num, endpoint=False)
    zero = np.zeros(100)
    command = np.concatenate((zero,theta_c*signal.square(np.pi*t_signal*0.3)),axis=0)

    theta_init = 0
    theta = theta_init*np.ones(num)
    tau_plt = theta

    k_p = 15    
    k_i = 0.2
    k_d = 9

    e_theta_in = 0
    theta_pr = 0

    for i in range(num-1):
        
        e_theta = command[i] - theta[i]

        e_theta_in = e_theta_in + e_theta*time_step

        theta_di = (theta[i]-theta_pr)/time_step
   
        theta_pr = theta[i]

        tau_p = k_p*e_theta + k_i*e_theta_in - k_d*theta_di
    
        if (np.abs(tau_p)>tau_phi_max):
            tau_p = tau_phi_max*(tau_p/np.abs(tau_p))


        theta_ddot = tau_p/j_y

        theta_di = theta_di + theta_ddot*time_step

        theta[i+1] = theta[i] + theta_di*time_step
        

    plt.plot(t, command,color='b')
    plt.plot(t, theta,color='r')
    #plt.plot(t, tau_plt,color='k')
    plt.xlim(x_start, x_end)
    plt.ylim(y_start, y_end)
    plt.grid(True)
    plt.show()




def yaw_hold():

    
    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max

    k_p = 15   
    k_i = 0.3
    k_d = 10

    psi_c = 30



    
    time_step = 0.01 

    x_start = 0
    x_end = 15
    y_start = psi_c+0.5*psi_c
    y_end = -1*y_start
    plt.figure(figsize=(20,12))
    num = int((x_end-x_start)/time_step)
    t_signal = np.linspace(x_start, x_end-1, num-100, endpoint=False)
    t = np.linspace(x_start, x_end, num, endpoint=False)
    zero = np.zeros(100)
    command = np.concatenate((zero,psi_c*signal.square(np.pi*t_signal*0.3)),axis=0)

    psi_init = 0
    psi = psi_init*np.ones(num)


    e_psi_in = 0 #integration
    psi_pr = 0 #previous valuse

    psi_dot = 0

    for i in range(num-1):
        
        e_psi = command[i] - psi[i]

        e_psi_in = e_psi_in + e_psi*time_step

        psi_di = (psi[i]-psi_pr)/time_step
   
        psi_pr = psi[i]

        tau_p = k_p*e_psi + k_i*e_psi_in - k_d*psi_di

        if (np.abs(tau_p)>tau_psi_max):
            tau_p = tau_psi_max*(tau_p/np.abs(tau_p))

        psi_ddot = tau_p/j_z

        psi_dot = psi_di + psi_ddot*time_step

        psi[i+1] = psi[i] + psi_dot*time_step
        

    plt.plot(t, command,color='b')
    plt.plot(t, psi,color='r')
    plt.xlim(x_start, x_end)
    plt.ylim(y_start, y_end)
    plt.grid(True)
    plt.show()



# not done
def altitude_hold():
    
    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max

    yaw_k_p = 10    
    yaw_k_i = 0.3
    yaw_k_d = 10

    phi_k_p = 15    
    phi_k_i = 0.2
    phi_k_d = 9

    pos_k_p = 1    
    pos_k_i = 0
    pos_k_d = 15

    thr_k_p = 3    
    thr_k_i = 0
    thr_k_d = 60

    time_step = 0.01


    x_c = 15
    y_c = 25
    z_c = 30


    x_start = 0
    x_end = 150
    y_end = np.abs(np.max([x_c,y_c,z_c]))*1.5
    y_start = -1*y_end
    plt.figure(figsize=(20,12))
    num = int((x_end-x_start)/time_step)
    t = np.linspace(x_start, x_end, num, endpoint=False)

    x_init = 0
    y_init = 0
    z_init = 0



    phi_init = 0
    theta_init = 0

    x_position = x_init*np.ones(num)
    y_position = y_init*np.ones(num)
    z_position = z_init*np.ones(num)

    x_dot = np.zeros(num)
    throttle = np.zeros(num)

    phi = phi_init*np.ones(num)
    theta = theta_init*np.ones(num)

    e_x_in = 0 # error integration
    e_y_in = 0
    e_z_in = 0
    e_phi_in = 0
    e_theta_in = 0

    x_pr = 0 # previous value
    y_pr = 0
    z_pr = 0
    phi_pr = 0
    theta_pr = 0
 
    for i in range(num-1):
        
        #position loop
        e_x = x_c - x_position[i]
        e_y = y_c - y_position[i]
        e_z = z_c - z_position[i]

        e_x_in = e_x_in + e_x*time_step
        e_y_in = e_y_in + e_y*time_step
        e_z_in = e_z_in + e_z*time_step

        x_di = (x_position[i] - x_pr)/time_step
        y_di = (y_position[i] - y_pr)/time_step
        z_di = (z_position[i] - z_pr)/time_step

        x_pr = x_position[i]
        y_pr = y_position[i]
        z_pr = z_position[i]

        theta_desire = e_x*pos_k_p + e_x_in*pos_k_i - x_di*pos_k_d
        phi_desire = e_y*pos_k_p + e_y_in*pos_k_i - y_di*pos_k_d




        resultant_angle = np.arccos(np.cos(theta_desire)*np.cos(phi_desire))

        #if (np.abs(resultant_angle)>Max_phi_theta*180/np.pi):
        #    theta_desire = theta_desire * (Max_phi_theta*180/np.pi) / np.abs(resultant_angle)
        #    phi_desire = phi_desire * (Max_phi_theta*180/np.pi) / np.abs(resultant_angle)


        if (np.abs(theta_desire)>Max_phi_theta):
            theta_desire = (Max_phi_theta)*(theta_desire/np.abs(theta_desire))

        if (np.abs(phi_desire)>Max_phi_theta):
            phi_desire = (Max_phi_theta)*(phi_desire/np.abs(phi_desire))



        print i,'--theta_desire:',theta_desire*180/np.pi,'  e_x:',e_x,'  e_x_in',e_x_in,'  x_di:',x_di


        #angle loop 

        e_theta = theta_desire - theta[i]
        e_phi = phi_desire - phi[i]

        e_theta_in = e_theta_in + e_theta*time_step
        e_phi_in = e_phi_in + e_phi*time_step

        theta_di = (theta[i] - theta_pr)/time_step
        phi_di = (phi[i] - phi_pr)/time_step

        theta_pr = theta[i]
        phi_pr = phi[i]

        tau_theta = e_theta*phi_k_p + e_theta_in*phi_k_i - theta_di*phi_k_d
        tau_phi = e_phi*phi_k_p + e_phi_in*phi_k_i - phi_di*phi_k_d

        
        if (np.abs(tau_theta)>tau_phi_max):
            tau_theta = tau_phi_max*(tau_theta/np.abs(tau_theta))

        if (np.abs(tau_phi)>tau_phi_max):
            tau_phi = tau_phi_max*(tau_phi/np.abs(tau_phi))

        theta_ddot = tau_theta/j_y
        phi_ddot = tau_phi/j_y

        theta_di = theta_di + theta_ddot*time_step
        phi_di = phi_di + phi_ddot*time_step

        theta[i+1] = theta[i] + theta_di*time_step
        phi[i+1] = phi[i] + phi_di*time_step

        #throttle loop

        throttle_output = (e_z*thr_k_p + e_z_in*thr_k_i - z_di*thr_k_d + m*g)/(np.cos(theta[i])*np.cos(phi[i]))

        #throttle_output = (m*g)/(np.cos(theta[i])*np.cos(phi[i])) ##

        if (abs(throttle_output)>Max_throttle):
                throttle_output = Max_throttle

        throttle[i] = throttle_output
        x_ddot = throttle_output*np.sin(theta[i])*time_step/m
        y_ddot = throttle_output*np.sin(phi[i])*time_step/m
        z_ddot = throttle_output*np.cos(theta[i])*np.cos(phi[i])*time_step/m - g/100

        x_di = x_di + x_ddot*time_step
        y_di = y_di + y_ddot*time_step
        z_di = z_di + z_ddot*time_step

        print i,'--throttle_output:',throttle_output,'  z_di:',z_di,'  z_ddot:',z_ddot,'  z_position:',z_position[i],'\n'

        x_position[i+1] = x_position[i] + x_di*time_step
        y_position[i+1] = y_position[i] + y_di*time_step
        z_position[i+1] = z_position[i] + z_di*time_step


    print('finish calulating')

    #plt.plot(t, theta*180/np.pi,color='r')
    #plt.plot(t, phi,color='r')

    plt.plot(t, x_position,color='b')
    plt.plot(t, y_position,color='r')
    plt.plot(t, z_position,color='k')
    plt.xlim(x_start, x_end)
    plt.ylim(y_start, y_end)

    plt.grid(True)
    plt.show()


if __name__ == '__main__':

    g = 9.8
    e = xml.etree.ElementTree.parse('/home/shizu/myws/src/uav/quadrotor_feature.xml').getroot()
    k1 = float(e.find('factor_SignalToForce').text)
    k2 = float(e.find('factor_SignalToTorque').text)
    l = float(e.find('length_RotorToCenter').text)
    Max_phi_theta = float(e.find('Max_phi_theta').text)
    Max_phi_theta = Max_phi_theta * np.pi/180
    Max_throttle = float(e.find('Max_throttle').text)
    Max_throttle = Max_throttle * g
    

    f = xml.etree.ElementTree.parse('/home/shizu/.gazebo/models/quadrotor_imu/model.sdf').getroot().find('model').find('link').find('inertial')
    m = float(f.find('mass').text)
    j_y = float(f.find('inertia').find('ixx').text)
    j_z = float(f.find('inertia').find('izz').text)
   

    print 'features: ',k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle

    tau_phi_max = l*(Max_throttle/4+m*g/4)

    tau_psi_max = l*((Max_throttle/4)*k2/k1)*2

    print ('max_values: '),tau_phi_max,tau_psi_max

    angle = (14.8*np.pi/180)
    print np.cos(angle)
    print np.arccos(np.cos(angle)**2)*180/np.pi
    #roll_pitch_hold()
    #yaw_hold()
    altitude_hold()




