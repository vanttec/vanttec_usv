#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
----------------------------------------------------------
    @file: usv_master.py
    @date: Tue Jan 28, 2020
	@author: Roberto Mendivil Castro
    @e-mail: robertomc97@gmail.com
    @co-author: Sebastian Martinez Perez
    @e-mail: sebas.martp@gmail.com
	@brief: Master script that controls the general mission status
    Open source
----------------------------------------------------------
'''

import subprocess

import subprocess

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

#TO DO 
#Cambiar tipo de dato STATUS
#AUTONAV Y SPEED CHALLENGE
#3 CURSOS Y 6 RETOS 

class USVMaster:
    def __init__(self):

        self.xbee_message= ""
        self.mission_status = 0

        self.kill_switch = False

        #ROS Subscribers
        rospy.Subscriber("/usv_comms/boat_transceiver/course_config", String, self.mission_callback) 
        rospy.Subscriber("/mission_status", Int32, self.mission_status_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/stop_mission", Empty, self.stop_mission_callback)
        rospy.Subscriber("/usv_comms/boat_transceiver/start_mission", Empty , self.start_mission_callback)

        ##Ros Publishers
        self.dataframe_pub = rospy.Publisher('/vanttec_usv/usv_master/usv_data', String, queue_size=10)
        self.boat_current_mission_pub = rospy.Publisher('/usv_master/usv_master_status', String, queue_size=10)
    
    #Callbacks
    def mission_callback(self,xbee_message):
        self.xbee_message = xbee_message.data
        rospy.loginfo(self.xbee_message)
    
    def mission_status_callback(self,mission_status):
        self.mission_status = mission_status.data

    def start_mission_callback(self,start_mission):
        self.kill_switch = False

    def stop_mission_callback(self,stop_mission):
        self.kill_switch = True
        #subprocess.Popen("roskill arduino-br ardumotors.py", shell = True)


def main():
    rospy.init_node('usv_master', anonymous=True)
    rate = rospy.Rate(100)
    usvMaster = USVMaster()

    #CUrrentMission Node 
    current_node=None 
    
    while not rospy.is_shutdown():    
      
        if (usvMaster.xbee_message == "CA") or (usvMaster.xbee_message == "ca"):
            usvMaster.boat_current_mission_pub.publish("Course Alpha")

        elif (usvMaster.xbee_message == "CB") or (usvMaster.xbee_message == "cb"):
            usvMaster.boat_current_mission_pub.publish("Course Beta")
            
        elif (usvMaster.xbee_message == "CC") or  (usvMaster.xbee_message == "cc"):
            usvMaster.boat_current_mission_pub.publish("Course Charlie")
            
        elif (usvMaster.xbee_message == "AN") or  (usvMaster.xbee_message == "an"):
            current_node = subprocess.Popen("rosrun rb_missions auto_nav_position.py", shell = True)
            usvMaster.xbee_message= ""
            while (usvMaster.mission_status != 1) and  (usvMaster.kill_switch!=True) and (not rospy.is_shutdown()):
                 usvMaster.boat_current_mission_pub.publish("Atonomous Navigation")
                 rate.sleep()
            current_node.kill()
                         
        elif (usvMaster.xbee_message == "SP") or (usvMaster.xbee_message == "sp"):
            current_node = subprocess.Popen("rosrun rb_missions speed_ch.py", shell = True)
            usvMaster.xbee_message= ""
            while (usvMaster.mission_status != 1) and (usvMaster.kill_switch!=True) and (not rospy.is_shutdown()):
                 usvMaster.boat_current_mission_pub.publish("Speed Challenge")
                 rate.sleep()
            current_node.kill()
        
        usvMaster.xbee_message= ""
        rate.sleep()

    if current_node is not None:
        pass
        #current_node.kill()# subprocess.Popen("rosnode kill usv_master", shell = True)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass
