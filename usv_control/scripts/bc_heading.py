#!/usr/bin/env python
#Backstepping controller with heading control and forward thrust input

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

NODE_NAME_THIS = 'bc_heading'

class Controller:
    def __init__(self):
        self.activated = True #determines if the controller should run

#Controller hydrodynamic and physical constants
        self.X_u_dot = -2.25
        self.Y_v_dot = -23.13
        self.mass = 30
        self.N_r = 0
        self.N_r_dot = -2.79
        self.I_z = 4.1
        self.B = 0.41

        self.c = 0.78

#Controller gains
        self.k1 = -3 #Heading controller yaw error gain
        self.k2 = 8 #Heading controller yaw rate gain

#Desired heading
        self.psi_d = 0

        self.tx_d = 0

#Controller feedback variables
        self.u = 0 #surge speed
        self.v = 0 #sway speed
        self.r = 0 #yaw rate
        self.lat = 0 #latiutde
        self.long = 0 #longitude
        self.psi = 0 #yaw

#Controller internal variables
        self.T_x = 0
        self.T_z = 0
        self.epsilon_psi = 0
        self.error_psi = 0

#Controller outputs
        self.T_port = 0 #Thrust in Newtons
        self.T_stbd = 0 #Thrust in Newtons

#Desired values subscribers
        rospy.Subscriber("/guidance/desired_heading", Float64, self.dheading_callback)
        rospy.Subscriber("/guidance/desired_thrust", Float64, self.dthrust_callback)

#IMU data subscribers
        rospy.Subscriber("/vectornav/ins_2d/local_vel", Vector3, self.local_vel_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_pose", Pose2D, self.ins_pose_callback)

#Thruster data publishers
        self.right_thruster_pub = rospy.Publisher("/usv_control/controller/right_thruster", Float64, queue_size=10)
        self.left_thruster_pub = rospy.Publisher("/usv_control/controller/left_thruster", Float64, queue_size=10)

        self.psi_error_pub = rospy.Publisher("/usv_control/bc_h/heading_error", Float64, queue_size=10)

    def dheading_callback(self, d_heading):
        self.psi_d = d_heading.data
        #rospy.logwarn("psi_d %f", self.psi_d)

    def dthrust_callback(self, d_thrust):
        self.tx_d = d_thrust.data

    def local_vel_callback(self, upsilon):
        self.u = upsilon.x
        self.v = upsilon.y
        self.r = upsilon.z

    def ins_pose_callback(self, pose):
        self.lat = pose.x
        self.long = pose.y
        self.psi = pose.theta
	#rospy.logwarn("psi %f", self.psi)

    def control(self, tx_d=0, psi_d=0):
#Nr hydrodynamic variable
        self.N_r = (-0.52)*(math.pow(math.pow((self.u), 2) + math.pow((self.v), 2), 0.5))
        #rospy.logwarn("Nr %f", self.N_r)

        self.error_psi = self.psi - psi_d #Yaw error
        if (math.fabs(self.error_psi) > (math.pi)):
            self.error_psi = (self.error_psi/math.fabs(self.error_psi))*(math.fabs(self.error_psi)-2*math.pi)
        if (math.fabs(self.error_psi)) < 0.015:
            self.error_psi = 0
        self.degree_error = math.degrees(self.error_psi)
        #rospy.logwarn("psi error %f", self.degree_error)

        self.epsilon_psi = (self.k1)*(self.error_psi) - (self.k2)*(self.r)

        self.T_z = (self.I_z - self.N_r_dot)*(self.epsilon_psi) - (self.Y_v_dot - self.X_u_dot)*(self.u)*(self.v) - (self.N_r)*(self.r)
        if math.fabs(self.error_psi) > 0.03:
            self.T_z = self.T_z * .5
        if math.fabs(self.error_psi) > 0.1:
            self.T_z = self.T_z * .7
        if math.fabs(self.error_psi) > 0.2:
            self.T_z = self.T_z * .7
        if math.fabs(self.error_psi) > 0.3:
            self.T_z = self.T_z * .8

        self.T_x = tx_d
        if math.fabs(self.error_psi) > 0.3:
            self.T_x = self.T_x * .8
        if math.fabs(self.error_psi) > 0.6:
            self.T_x = self.T_x * .7
        if math.fabs(self.error_psi) > 1:
            self.T_x = self.T_x * .7
        if math.fabs(self.error_psi) > 2:
            self.T_x = self.T_x * .5

        self.T_port = (self.T_x/(2*self.c)) + (self.T_z/(self.B*self.c))
        self.T_stbd = (self.T_x/2) - (self.T_z/self.B)
        if self.T_port > 36.5:
            self.T_port = 20
        elif self.T_port < -30:
            self.T_port = -20
        if self.T_stbd > 36.5:
            self.T_stbd = 20
        elif self.T_stbd < -30:
            self.T_stbd = -20

        if self.T_x == 0:
            self.T_stbd = 0
            self.T_port = 0

#Controller outputs
        self.right_thruster_pub.publish(self.T_stbd)
        self.left_thruster_pub.publish(self.T_port)

        self.psi_error_pub.publish(self.degree_error)

    def run(self, tx_d=0, psi_d=0):
        self.control(tx_d, psi_d)

def main():
    rospy.init_node(NODE_NAME_THIS, anonymous=False, disable_signals=False)
    rate = rospy.Rate(100) # 100hz
    rospy.loginfo("Test node running")
    C = Controller()
    while not rospy.is_shutdown() and C.activated:
        C.run(C.tx_d, C.psi_d)
        rate.sleep()
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
