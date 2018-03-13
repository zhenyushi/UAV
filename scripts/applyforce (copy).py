#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench,BodyRequest#, ClearBodyWrenches
import xml.etree.cElementTree


from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import numpy as np
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
                     duration = rospy.Duration(-1))
    except rospy.ServiceException as e:
        print e



def callback(data):

    clear_wrench_call()
    PWM = np.array(data.data)
    wrenchfactors = [[k1,k1,k1,k1],[0,-1*l*k1,0,l*k1],[l*k1,0,-1*l*k1,0],[-k2,k2,-k2,k2]]
    wrenchoutput = np.dot(wrenchfactors,PWM)
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = wrenchoutput[0]
    wrench.torque.x = wrenchoutput[1]
    wrench.torque.y = wrenchoutput[2]
    wrench.torque.z = wrenchoutput[3]
    apply_force_call(wrench,reference_point)






if __name__ == "__main__":

    rospy.init_node('PWM_to_wrench', anonymous=True) # initialize node here
    r = rospy.Rate(1)

    e = xml.etree.ElementTree.parse('/home/shizu/myws/src/uav/quadrotor_feature.xml').getroot()
    k1 = float(e.find('factor_SignalToForce').text)
    k2 = float(e.find('factor_SignalToTorque').text)
    l = float(e.find('length_RotorToCenter').text)
    
    print k1,k2,l

    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    clear_body_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
    reference_point = Point(0,0,0)
    #listener()
    
    rospy.Subscriber("PWM_rotors", Int32MultiArray, callback)
    
    rospy.spin()
    while not rospy.is_shutdown():

        wrench1 = Wrench()
        wrench1.force.x = 0
        wrench1.force.y = 0
        wrench1.force.z = 100
        wrench1.torque.x = 0
        wrench1.torque.y = 0
        wrench1.torque.z = 0
        #clear_wrench_call()
        apply_force_call(wrench1,reference_point)
        print("1")
        rospy.spin()
        r.sleep()




