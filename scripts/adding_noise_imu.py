#!/usr/bin/env python

import sys
import rospy
import tf

from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench,BodyRequest
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import numpy as np
import time
#noise = np.random.normal(0,1,100)




def pub_imu(header,angular_velocity,linear_acceleration):
    try:
        imu(header,
                     angular_velocity,
                     reference_point = reference_point,
                     wrench = wrench,
                     duration = rospy.Duration(-1))
    except rospy.ServiceException as e:
        print e

def get_state_call(model_name):
    try:
        return get_model_state(model_name = model_name)
    except rospy.ServiceException as e:
        print e



def callback_Imu(data):

    global previous_time
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print euler
    print '\n'
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


    #rospy.loginfo(data)
    #print("\n")
    #print data.orientation
    #print("-------------------------")
    #print("Euler")
    #print roll, pitch, yaw
    #print("\n")

    rospy.wait_for_service('/gazebo/get_model_state')
    model_state = get_state_call(model_name)

    

    x = model_state.pose.position.x
    y = model_state.pose.position.y
    z = model_state.pose.position.z

    #eular_m = tf.transformations.euler_from_quaternion([model_state.orientation.x,model_state.orientation.y,model_state.orientation.z,model_state.orientation.w,])

    #a = eular_m[0]
    #b = eular_m[1]
    #c = eular_m[2]

    u = model_state.twist.linear.x
    v = model_state.twist.linear.y
    w = model_state.twist.linear.z

    p = model_state.twist.angular.x
    q = model_state.twist.angular.y
    r = model_state.twist.angular.z
    
    imu = data
    g = 9.8
    #imu.orientation = model_state.pose.orientation
    #imu.angular_velocity = model_state.twist.angular

    #imu.linear_acceleration.x = imu.linear_acceleration.x + r*v-q*w - g*np.sin(pitch)
    #imu.linear_acceleration.y = imu.linear_acceleration.y + p*w-r*u + g*np.cos(pitch)*np.sin(roll)
    #imu.linear_acceleration.z = imu.linear_acceleration.z + q*u-p*v + g*np.cos(pitch)*np.cos(roll)
    
    sq = np.sqrt(imu.linear_acceleration.x**2+imu.linear_acceleration.y**2+imu.linear_acceleration.z**2)
    
    imu.header.frame_id = '{0}, {1}, {2}, {3}, {4}, {5}'.format(x, y, z, u, v, w)
    
    
    #rospy.loginfo(imu)

    pub_imu.publish(imu)

    #header = data.header
    #angular_velocity = data.angular_velocity + np.random.normal(0,1,100)
    #linear_acceleration = data.linear_acceleration + np.random.normal(0,1,100)
    #pub_imu(header,angular_velocity_noise,linear_acceleration_noise)




if __name__ == "__main__":

    rospy.init_node('adding_noise_imu', anonymous=True)
    
    pub_imu = rospy.Publisher('/quadrotor_imu/imu_gravity', Imu, queue_size=1)
    imu = Imu()
    model_name = 'quadrotor_imu'
    rospy.Subscriber('/quadrotor_imu/imu', Imu, callback_Imu)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    previous_time = int(round(time.time() * 1000))
    rospy.spin()





