#!/usr/bin/env python
import rospy
import rospkg
import time
import numpy as np


from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from gazebo_msgs.msg import ModelState

import xml.etree.cElementTree

#### Global Parameters Initialized ####



    
   

if __name__ == '__main__': # this is main function
    
    rospy.init_node('wp_control', anonymous=True) # initialize node here
    r = rospy.Rate(100)
    print('1')
    ## setup the publisher
    pub1 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
    pub2 = rospy.Publisher('gazebo/set_model_state',ModelState,queue_size=10)

    e = xml.etree.ElementTree.parse('/home/shizu/myws/src/uav/initial_state.xml').getroot()

    for atype in e.findall('country'):
        print(atype.attrib)
        print('2')


    

    while not rospy.is_shutdown():


	# compute control commands here
        cmd = Twist()
        cmd.linear.x = 0.4
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.1

       
        ## publish to the topics
        pub1.publish(cmd)

        garos = ModelState()
        garos.model_name = 'coke_can'
        garos.pose.position.x = 1
        garos.pose.position.y = 0
        garos.pose.position.z = 2
        garos.twist.linear.x=0
        garos.twist.linear.y=0
        garos.twist.linear.z=0
        pub2.publish(garos)        


        r.sleep()
    rospy.loginfo("Controller Node Has Shutdown.")
    rospy.signal_shutdown(0)
