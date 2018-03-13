#!/usr/bin/env python
import rospy  
import roslib
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import tf


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


        eular = (0,3.1415926*20/180,0)
        quaternion = tf.transformations.quaternion_from_euler(0,3.1415926*20/180,0)

        msg = ModelState()  


        msg.pose.position.x = 0.0 
        msg.pose.position.y = -0.15 
        msg.pose.position.z = 1.0 
        msg.twist.linear.z = 0


        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]


        msg.model_name = "wideanglecamera"
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':

    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    try:
         setmodel()
    except rospy.ROSInterruptException:
         pass
