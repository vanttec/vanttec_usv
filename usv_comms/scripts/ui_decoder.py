#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
#****************************************************************************************#
# Message
MESSAGE = "course_task2#state2#object_detected_list2#gps_pose2#target2"
# Frecuencia en Hz a la que se va a correr el nodo.
ROS_RATE = 100
#****************************************************************************************#

class Ui_decoder:
    def __init__(self, message, _ros_rate):
        # String for separating string
        self.data = message.split("#")

        rospy.loginfo('[STATION] Ui_decoder initialized.')

        # ROS Configuration
        self.ros_rate = rospy.Rate(_ros_rate)

        # ROS Publisher
        self.mission_tasks = rospy.Publisher('/usv_comms/ui_decoder/mission_tasks', String, queue_size=10)
        self.mission_state = rospy.Publisher('/usv_comms/ui_decoder/mission_state', String, queue_size=10)
        self.object_detected_list = rospy.Publisher('/usv_comms/ui_decoder/object_detected_list', String, queue_size=10)
        self.gps_pose = rospy.Publisher('/usv_comms/ui_decoder/gps_pose', String, queue_size=10)
        self.target = rospy.Publisher('/usv_comms/ui_decoder/target', String, queue_size=10)

    def publish_messages(self):
        self.mission_tasks.publish(self.data[0])
        self.mission_state.publish(self.data[1])
        self.object_detected_list.publish(self.data[2])
        self.gps_pose.publish(self.data[3])
        self.target.publish(self.data[4])

def main():
    rospy.loginfo(" +--------------------------------------+")
    rospy.loginfo(" |                Ui_decoder               |")
    rospy.loginfo(" +--------------------------------------+\n")

    rospy.init_node('ui_decoder', anonymous=True)

    Decoder= Ui_decoder(MESSAGE,ROS_RATE)
    while not rospy.is_shutdown():
        Decoder.publish_messages()
        Decoder.ros_rate.sleep()
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass