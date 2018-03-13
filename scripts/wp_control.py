#!/usr/bin/env python
import rospy
import rospkg
import time
import numpy as np


from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


#### Global Parameters Initialized ####

xn       = 0
yn       = 0
psi      = 0
psidot = 0
kp = 0.6
xw = 6
yw = 1

def callback(data): #  read data from turtle1/pose

    global xn,yn,psi,psidot
    xn       = data.x
    yn       = data.y
    psi      = data.theta

    psid   = np.arctan2(yw-yn, xw-xn) # psid is the desired heading to move to the waypoint
    psidot = kp*(psid - psi) # turn rate
    
   

if __name__ == '__main__': # this is main function
    
    rospy.init_node('wp_control', anonymous=True) # initialize node here
    r = rospy.Rate(20)
    
    ## setup the publisher
    pub1 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
   

    ## setup the subscriber
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback) #motion model

    while not rospy.is_shutdown():

        sub = rospy.Subscriber("/turtle1/pose", Pose, callback) #motion model
	# compute control commands here
        cmd = Twist()
        cmd.linear.x = 0.4
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = psidot

        print(psidot)

        if (fabs(yw - yn)<0.2 and fabs(xw - xn)):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.loginfo(" Waypoint reached ")
            rospy.signal_shutdown(0)             
                    
        ## publish to the topics
        pub1.publish(cmd)
        

        r.sleep()
    rospy.loginfo("Controller Node Has Shutdown.")
    rospy.signal_shutdown(0)
