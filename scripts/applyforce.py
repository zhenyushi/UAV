#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench,BodyRequest
import xml.etree.cElementTree


from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import datetime
#noise = np.random.normal(0,1,100)


def clear_wrench_call():
    try:
        clear_body_wrench(body_name = "quadrotor_imu::quadrotor")
    except rospy.ServiceException as e:
        print e


def apply_force_call(wrench,reference_point):
    try:
        apply_body_wrench(body_name = "quadrotor_imu::quadrotor",
                     reference_frame = "quadrotor_imu::quadrotor",
                     reference_point = reference_point,
                     wrench = wrench,
                     duration = rospy.Duration(0.012))
    except rospy.ServiceException as e:
        print e



def callback(data):


    PWM = np.array(data.data)
    #print PWM, data.data

    print PWM[0:4]
    wrenchfactors = [[k1,k1,k1,k1],[0,-1*l*k1,0,l*k1],[l*k1,0,-1*l*k1,0],[-k2,k2,-k2,k2]]
    wrenchoutput  = np.dot(wrenchfactors,PWM[0:4])
    print wrenchoutput

    #rotation_matrix
	   #transf_x=cos(ya)*cos(pi)*ox+cos(ya)*sin(pi)*sin(ro)*oy-sin(ya)*cos(ro)*oy+cos(ya)*sin(pi)*cos(ro)*oz+sin(ya)*sin(ro)*oz+px;
	   #transf_y=sin(ya)*cos(pi)*ox+sin(ya)*sin(pi)*sin(ro)*oy+cos(ya)*cos(ro)*oy+sin(ya)*sin(pi)*cos(ro)*oz-cos(ya)*sin(ro)*oz+py;
	   #transf_z=-1*sin(pi)*ox+cos(pi)*sin(ro)*oy+cos(pi)*cos(ro)*oz+pz;


    wrench.force.x = (np.cos(PWM[6])*np.sin(PWM[5])*np.cos(PWM[4])+np.sin(PWM[6])*np.sin(PWM[4])) * wrenchoutput[0]
    wrench.force.y = (np.sin(PWM[6])*np.sin(PWM[5])*np.cos(PWM[4])-np.cos(PWM[6])*np.sin(PWM[4])) * wrenchoutput[0]
    wrench.force.z = np.cos(PWM[5])*np.cos(PWM[4]) * wrenchoutput[0]

    wrench.torque.x =(np.cos(PWM[6])*np.cos(PWM[5])*wrenchoutput[1] \
        +(np.cos(PWM[6])*np.sin(PWM[5])*np.sin(PWM[4])-np.sin(PWM[6])*np.cos(PWM[4]))*wrenchoutput[2] \
        +(np.cos(PWM[6])*np.sin(PWM[5])*np.cos(PWM[4])+np.sin(PWM[6])*np.sin(PWM[4]))*wrenchoutput[3])
                      
    wrench.torque.y =(np.sin(PWM[6])*np.cos(PWM[5])*wrenchoutput[1] \
        +(np.sin(PWM[6])*np.sin(PWM[5])*np.sin(PWM[4])+np.cos(PWM[6])*np.cos(PWM[4]))*wrenchoutput[2] \
        +(np.sin(PWM[6])*np.sin(PWM[5])*np.cos(PWM[4])-np.cos(PWM[6])*np.sin(PWM[4]))*wrenchoutput[3])

    wrench.torque.z = -1*np.sin(PWM[5])*wrenchoutput[1] \
        +np.cos(PWM[5])*np.sin(PWM[4])*wrenchoutput[2] \
        +np.cos(PWM[5])*np.cos(PWM[4])*wrenchoutput[3]


    #wrench.force.x  = wrenchoutput[0] * np.sin(PWM[5])
    #wrench.force.y  = -1*wrenchoutput[0] * np.sin(PWM[4])
    #wrench.force.z  = wrenchoutput[0] * np.cos(PWM[4]) * np.cos(PWM[5])

    ##wrench.force.z  = wrenchoutput[0]
    #wrench.torque.x = wrenchoutput[1]
    #wrench.torque.y = wrenchoutput[2]
    #wrench.torque.z = wrenchoutput[3]



    

    #print datetime.datetime.now()
    #clear_wrench_call()
    #print datetime.datetime.now()
    apply_force_call(wrench,reference_point)
    print datetime.datetime.now()
    print wrench
    print("\n")



if __name__ == "__main__":

    rospy.init_node('PWM_to_wrench', anonymous=True)

    e = xml.etree.ElementTree.parse('/home/shizu/myws/src/uav/quadrotor_feature.xml').getroot()
    k1 = float(e.find('factor_SignalToForce').text)
    k2 = float(e.find('factor_SignalToTorque').text)
    l = float(e.find('length_RotorToCenter').text)

    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    clear_body_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)

    reference_point = Point(0,0,0)
    reference_frame = 'quadrotor_imu::quadrotor'
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0

    rospy.Subscriber("PWM_rotors", Float32MultiArray, callback)
    rospy.spin()





