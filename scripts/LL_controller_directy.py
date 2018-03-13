#!/usr/bin/env python

# license removed for brevity
import rospy
import tf

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import numpy as np
import time
import xml.etree.cElementTree
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench

from numpy.linalg import inv
from gazebo_msgs.srv import GetModelState
import datetime


def get_state_call(model_name):
    try:
        return get_model_state(model_name = model_name)
    except rospy.ServiceException as e:
        print e




def callback_Imu(data):


    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max
    global previous_time, previous_read,e_integration,previous_value,time_sample

    #Low Pass Filter
    current_time = int(round(time.time() * 1000))
    time_sample = current_time - previous_time
    LPF_alpha = np.exp(-1*LPF_e*time_sample/1000)
    previous_time = current_time

    print 'time_sample',time_sample

# call service


    rospy.wait_for_service('/gazebo/get_model_state')
    #print 'before'
    #print datetime.datetime.now()
    model_state = get_state_call(model_name)
    #print datetime.datetime.now()
    #print 'after'
    x = model_state.pose.position.x
    y = model_state.pose.position.y
    z = model_state.pose.position.z

    
    #print '\n',model_state,'\n'
    velocity = model_state.twist
    gyro_model = velocity.angular

    u = velocity.linear.x
    v = velocity.linear.y
    w = velocity.linear.z

    #print '------------------------------------------'
    #print 'position:',model_state.pose.position
    #print '\n'

    # orientation from IMU
    quaternion = (data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    eular = tf.transformations.euler_from_quaternion(quaternion)
    

    # orientation from MODEL_STATE
    quaternion_correct = (model_state.pose.orientation.x,model_state.pose.orientation.y,model_state.pose.orientation.z,model_state.pose.orientation.w)
    eular_correct = tf.transformations.euler_from_quaternion(quaternion_correct)

    #print 'eular',eular
    #print '\n'
    roll = eular_correct[0]
    pitch = eular_correct[1]
    yaw = eular_correct[2]

    

# coordinate rotation

    x_b = x*np.cos(yaw) + y*np.sin(yaw)
    y_b = -1*x*np.sin(yaw) + y*np.cos(yaw)

    u_b = u*np.cos(yaw) + v*np.sin(yaw)
    v_b = -1*u*np.sin(yaw) + v*np.cos(yaw)

    #position = [x, y, z, u, v, w]
    position = [x_b, y_b, z, u_b, v_b, w]


    #data.angular_velocity
    #data.linear_acceleration

    previous_read.angular_velocity = LPS_process(LPF_alpha,previous_read.angular_velocity,data.angular_velocity)
    previous_read.linear_acceleration = LPS_process(LPF_alpha,previous_read.linear_acceleration,data.linear_acceleration)


    e_integration = np.zeros(6) #roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    previous_value = np.zeros(6) #roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    
    eular_c = np.zeros(3) # roll=0, pitch=1, yaw=2

    pose_c = np.zeros(3)  # x=0, y=1, z=2


    #eular_c[2]=np.pi/4


    x_desire = 3
    y_desire = 3
    z_desire = 1


    pose_c[0] = x_desire*np.cos(yaw) + y_desire*np.sin(yaw)
    pose_c[1] = -1*x_desire*np.sin(yaw) + y_desire*np.cos(yaw)
    pose_c[2] = z_desire
    

    print pose_c,'\n',x_b,y_b


    wrenchfactors = [[k1,k1,k1,k1],[0,-1*l*k1,0,l*k1],[l*k1,0,-1*l*k1,0],[-k2,k2,-k2,k2]]



# controller
    
    # position hold
    eular_c[0],eular_c[1],thr_out = position_hold(pose_c,position,eular_correct)

    

    if(0):#1):
        eular_c[0]=-15*np.pi/180
        eular_c[1]=0
        eular_c[2]=np.pi/4


    if(0):#5>z>3):
        eular_c[0]=15*np.pi/180

    #print eular_c[0],eular_c[1],thr_out
    #print '\n'


    # attitude hold

    gyro_zero = Vector3()
    gyro_zero.x = 0
    gyro_zero.y = 0
    gyro_zero.z = 0
    
    print 'eular--------',eular
    print 'eular_correct',eular_correct,'\n'
    print '------position-----''\n',model_state.pose.position,'\n','\n'
    #tau_roll, tau_pitch, tau_yaw = attitude_hold(eular_c,eular,gyro_zero)

    tau_roll, tau_pitch, tau_yaw = attitude_hold(eular_c,eular_correct,gyro_model)

    #tau_roll, tau_pitch, tau_yaw = attitude_hold(eular_c,eular,previous_read.angular_velocity)

    #print roll,tau_roll

    output = np.dot(inv(wrenchfactors),[thr_out, tau_roll, tau_pitch, tau_yaw])


    while(np.amax(output)>1):

        thr_out = thr_out/np.amax(output)
        output = np.dot(inv(wrenchfactors),[thr_out,tau_roll, tau_pitch, tau_yaw])
        #print output




    rotor1 = output[0]
    rotor2 = output[1]
    rotor3 = output[2]
    rotor4 = output[3]

    

    PWM_pub.data = [rotor1,rotor2,rotor3,rotor4,roll,pitch,yaw]


    #PWM_pub.data = [0, 0 ,0 , 0, roll, pitch, yaw]


    #p = np.ones(4)
    #PWM_pub.data = p
    #print PWM_pub.data
    #print("\n")
    #PWM_pub.data = np.random.randint(20, size=4)
    #rospy.loginfo(PWM_pub)
    pub.publish(PWM_pub)


def LPS_process(alpha, previous, current):

    output = Vector3()
    output.x = alpha*previous.x + (1-alpha)*current.x
    output.y = alpha*previous.y + (1-alpha)*current.y
    output.z = alpha*previous.z + (1-alpha)*current.z
    return output



def attitude_hold(eular_c,eular,gyro):

    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max
    global slide_k,yaw_k,e_integration,previous_value,time_sample
    

    #e_integration = np.zeros(6)   roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    #previous_value = np.zeros(6)  roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    #eular_c = np.zeros(3)         roll=0, pitch=1, yaw=2
    #eular[0]=roll, eular[1]=pitch, eular[2]=yaw
    #gyro.y = pitch_dot, gyro.x = roll_dot, gyro.z = yaw_dot


    # phi hold
    e_phi = eular_c[0] - eular[0]

    e_integration[0] = e_integration[0] + e_phi*time_sample

    #phi_di = (eular.x-previous_value[0])/time_sample

    previous_value[0] = eular[0]

    phi_di = gyro.x

    tau_roll = slide_k[0]*e_phi + slide_k[1]*e_integration[0] - slide_k[2]*phi_di
    
    if (np.abs(tau_roll)>tau_phi_max):
        tau_roll = tau_phi_max*(tau_roll/np.abs(tau_roll))


    # theta hold
    e_theta = eular_c[1] - eular[1]

    e_integration[1] = e_integration[1] + e_theta*time_sample

    #theta_di = (eular.y-previous_value[1])/time_sample

    previous_value[1] = eular[1]

    theta_di = gyro.y

    tau_pitch = slide_k[0]*e_theta + slide_k[1]*e_integration[1] - slide_k[2]*theta_di
    
    if (np.abs(tau_pitch)>tau_phi_max):
        tau_pitch = tau_phi_max*(tau_pitch/np.abs(tau_pitch))


    
    # psi hold
    e_yaw = eular_c[2] - eular[2]

    e_integration[2] = e_integration[2] + e_yaw*time_sample

    #yaw_di = (eular.z-previous_value[2])/time_sample

    previous_value[2] = eular[2]

    yaw_di = gyro.z

    tau_yaw = yaw_k[0]*e_yaw + yaw_k[1]*e_integration[2] - yaw_k[2]*yaw_di
    
    if (np.abs(tau_yaw)>tau_phi_max):
        tau_yaw = tau_phi_max*(tau_yaw/np.abs(tau_yaw))


    return  tau_roll, tau_pitch, tau_yaw




def position_hold(pose_c,pose,eular):

    global k1,k2,l,m,j_y,j_z,Max_phi_theta,Max_throttle,tau_phi_max,tau_psi_max
    global pos_k,thr_k,e_integration,previous_value,time_sample

    #e_integration = np.zeros(6)   roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    #previous_value = np.zeros(6)  roll,pitch,yaw = 0,1,2 ; x,y,z = 3,4,5
    #pose_c = np.zeros(3)          x=0, y=1, z=2
    #pose[0]=x, pose[1]=y, pose[2]=z
    #pose[3]=u, pose[4]=v, pose[5]=w


    #position loop
    e_x = pose_c[0] - pose[0]
    e_y = pose_c[1] - pose[1]
    e_z = pose_c[2] - pose[2]

    #print 'e_z',e_z,pose[2],pose[5],thr_k[0]

    e_integration[3] = e_integration[3] + e_x*time_sample
    e_integration[4] = e_integration[4] + e_y*time_sample
    e_integration[5] = e_integration[5] + e_z*time_sample

    #x_di = (pose[0] - previous_value[3])/time_sample
    #y_di = (pose[1] - previous_value[4])/time_sample
    #z_di = (pose[2] - previous_value[5])/time_sample

    x_di = pose[3]
    y_di = pose[4]
    z_di = pose[5]

    previous_value[3] = pose[3]
    previous_value[4] = pose[4]
    previous_value[5] = pose[5]

    theta_desire = (e_x*pos_k[0] + e_integration[3]*pos_k[1] - x_di*pos_k[2])
    phi_desire = -1*(e_y*pos_k[0] + e_integration[4]*pos_k[1] - y_di*pos_k[2])

    print '\n'
    print 'x',e_x,x_di
    print 'y',e_y,y_di
    print '\n'
    
    print theta_desire,phi_desire, '\n'
    # tune later
    #resultant_angle = np.arccos(np.cos(theta_desire)*np.cos(phi_desire))
    #if (np.abs(resultant_angle)>Max_phi_theta*180/np.pi):
    #    theta_desire = theta_desire * (Max_phi_theta*180/np.pi) / np.abs(resultant_angle)
    #    phi_desire = phi_desire * (Max_phi_theta*180/np.pi) / np.abs(resultant_angle)


    if (np.abs(theta_desire)>Max_phi_theta):
        theta_desire = (Max_phi_theta)*(theta_desire/np.abs(theta_desire))

    if (np.abs(phi_desire)>Max_phi_theta):
        phi_desire = (Max_phi_theta)*(phi_desire/np.abs(phi_desire))


    # throttle control
    throttle_output = (e_z*thr_k[0] + e_integration[5]*thr_k[1] - z_di*thr_k[2] + m*g)/(np.cos(eular[0])*np.cos(eular[1]))

    #throttle_output = (m*g)/(np.cos(eular[0])*np.cos(eular[1]))
    
    #throttle_output = m*g
    #print throttle_output,m*g,np.cos(eular[0]),np.cos(eular[1])


    if (throttle_output<0):
        throttle_output = 0

    if (abs(throttle_output)>Max_throttle):
        throttle_output = Max_throttle

    #print throttle_output


    return phi_desire,theta_desire,throttle_output





def controller():

    rospy.Subscriber('/quadrotor_imu/imu', Imu, callback_Imu)
    rospy.spin()






if __name__ == '__main__':


    f = xml.etree.ElementTree.parse('/home/shizu/.gazebo/models/quadrotor_imu/model.sdf').getroot().find('model').find('link').find('inertial')
    m = float(f.find('mass').text)
    j_y = float(f.find('inertia').find('ixx').text)
    j_z = float(f.find('inertia').find('izz').text)

    g = 9.8

    e = xml.etree.ElementTree.parse('/home/shizu/myws/src/uav/quadrotor_feature.xml').getroot()
    k1 = float(e.find('factor_SignalToForce').text)
    k2 = float(e.find('factor_SignalToTorque').text)
    l = float(e.find('length_RotorToCenter').text)
    Max_phi_theta = float(e.find('Max_phi_theta').text)
    Max_phi_theta = Max_phi_theta * np.pi/180
    Max_throttle = float(e.find('Max_throttle').text)
    Max_throttle = Max_throttle*g
    LPF_e = float(e.find('LPF_e').text)

    tau_phi_max = l*(Max_throttle/4+m*g/4)
    tau_psi_max = l*((Max_throttle/4)*k2/k1)*2

    slide_k = np.zeros(3)
    slide_k[0] = float(e.find('slide_k_p').text)
    slide_k[1] = float(e.find('slide_k_i').text)
    slide_k[2] = float(e.find('slide_k_d').text)

    yaw_k = np.zeros(3)
    yaw_k[0] = float(e.find('yaw_k_p').text)
    yaw_k[1] = float(e.find('yaw_k_i').text)
    yaw_k[2] = float(e.find('yaw_k_d').text)

    pos_k = np.zeros(3)
    pos_k[0] = float(e.find('pos_k_p').text)
    pos_k[1] = float(e.find('pos_k_i').text)
    pos_k[2] = float(e.find('pos_k_d').text)

    thr_k = np.zeros(3)
    thr_k[0] = float(e.find('thr_k_p').text)
    thr_k[1] = float(e.find('thr_k_i').text)
    thr_k[2] = float(e.find('thr_k_d').text)


    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('PWM_rotors', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(100) 
    PWM_pub = Float32MultiArray()
    previous_time = int(round(time.time() * 1000))
    previous_read = Imu()


# call service
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model_name = 'quadrotor_imu'

    try:
        controller()
    except rospy.ROSInterruptException:
        pass
