#!/usr/bin/env python
import rospy  
import roslib
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import numpy as np



def get_state_call(model_name):
    try:
        return get_model_state(model_name = model_name)
    except rospy.ServiceException as e:
        print e



def setmodel():

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('set_model_state', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():

        
        modelstate = get_state_call("quadrotor_imu")

        msg = ModelState()  
        msg.pose = modelstate.pose
        msg.twist = modelstate.twist




        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0.1

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1.0

        msg.twist.linear.x = 0
        msg.twist.linear.y = 0
        msg.twist.linear.z = 0

        msg.twist.angular.x = 0
        msg.twist.angular.y = 0
        msg.twist.angular.z = 0
        msg.model_name = "quadrotor_imu"
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':

    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
         setmodel()
    except rospy.ROSInterruptException:
         pass
