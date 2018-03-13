#!/usr/bin/env python
import rospy  
import roslib
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState


def setmodel():

    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('set_model_state', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        msg = ModelState()  
        msg.pose.position.z = 3.0 
        msg.model_name = "quadrotor_imu"
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
         setmodel()
    except rospy.ROSInterruptException:
         pass
