#! /usr/bin/env python2.7

import rospy 
import rospkg 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState 
import os
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Float32MultiArray
import scipy.io as sio
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

boatname = rospy.get_param("boat7/boatname")
def main():
    dir_name = os.path.dirname(__file__)
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 33
    address = '/mat/finalposition{}.mat'.format(boatname)
    matlabposition = sio.loadmat(dir_name +address)
    matlabposition = matlabposition['finalposition']
    state_msg = ModelState() 
    state_msg.model_name = boatname
    

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        yaw = -math.pi/2
        yawf = 0
        for i in range(72):
            if i >=1 and i<= 13:
                yawf = 0
            elif i >=14 and i<=43:
                yawf = yaw
            elif i>=44 and i<=61:
                yawf = yaw*2
            elif i >=62:
                yawf = yaw*3
            rospy.logwarn(yawf)
            quaternion = quaternion_from_euler(0, 0, yawf)
            state_msg.pose.position.x = matlabposition[i,1]
            state_msg.pose.position.y = matlabposition[i,2]
            state_msg.pose.position.z = 0
            state_msg.pose.orientation.x = quaternion[0]
            state_msg.pose.orientation.y = quaternion[1]
            state_msg.pose.orientation.z = quaternion[2]
            state_msg.pose.orientation.w = quaternion[3]
            resp = set_state( state_msg )
            rospy.sleep(1)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('set_pose')
    try:
        rospy.logwarn("Main")
        main()

    except rospy.ROSInterruptException:
        pass