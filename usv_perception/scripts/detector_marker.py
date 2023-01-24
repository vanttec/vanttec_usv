#!/usr/bin/env python

#The code on top tells the compiler to use the main python

from genpy import Time
import rospy
import math
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2




#from sensor_msgs.msg import Image
import numpy as np
from usv_perception.msg     import obj_detected
from usv_perception.msg     import obj_detected_list
from usv_perception.msg     import obj_registered
from usv_perception.msg     import obj_registered_list
from geometry_msgs.msg      import Pose2D #Message to function Pose2D
from geometry_msgs.msg      import Vector3 #Message to function Pose2D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


import matplotlib.pyplot as plt

#This script creates markers in rviz that directly represent the obstacles detected 

class marker_detector_node:
    def __init__(self):

        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.callback_yolo_det)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)
    
        self.objects_detected_lidar = obj_detected_list()
        #self.obj_detected_yolo = obj_detected_list()
        self.NED_pose = Pose2D()
        self.detection_markers = MarkerArray
        self.detection_markers_pub = rospy.Publisher('/usv_perception/detection_markers', MarkerArray, queue_size=10)

    def objs_callback(self,data):
        self.objects_detected_lidar = data
        #rospy.loginfo("Detecting currently: " + str(len(data.objects)) + " objects")

        
    def callback_NED_pose(self,data):
        self.NED_pose = data

    ## First function to change the object_registered list coordinates from BOAT frame to GPS frame - VN
            #rospy.loginfo("New coord: " + str(i.X) + " X and: " + str(i.Y) + " Y" ) 

    def makeMarkers(self):
        
        tempMarkerArray = MarkerArray()

        for registered_object in self.objects_detected_lidar.objects:
  
                tempMarker = Marker()
                tempObjReg = obj_registered()   
                tempObjReg.X = registered_object.X
                tempObjReg.Y = registered_object.Y
                tempObjReg.id = registered_object.id
                tempObjReg.color = registered_object.color

                # #This sections transforms the NED coordinates to body to visualize in RViz
                # rospy.loginfo("MarkNED coords " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")
                # n = np.array([tempObjReg.X - self.NED_pose.x, tempObjReg.Y - self.NED_pose.y])
                # J = self.rotation_matrix()
                # J = np.linalg.inv(J)
                # b = J.dot(n)
                # tempObjReg.X = b[0]
                # tempObjReg.Y = b[1]
                # rospy.loginfo("MarkBody coords are " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")

                tempMarker.header.frame_id = "vtec_s3/velodyne"
                tempMarker.header.stamp = rospy.Time()
                tempMarker.type = tempMarker.SPHERE
                tempMarker.id = tempObjReg.id
                tempMarker.pose.position.z = 0
                tempMarker.pose.position.x = tempObjReg.X
                tempMarker.pose.position.y = tempObjReg.Y
                tempMarker.color.a = 1
                tempMarker.color.r = 1
                tempMarker.color.g = 0
                tempMarker.color.b = 1
                tempMarker.scale.x = 0.3
                tempMarker.scale.y = 0.3
                tempMarker.scale.z = 0.3

                tempMarkerArray.markers.append(tempMarker)
                
        self.register_markers = tempMarkerArray
            


    def registerObjects(self):
        self.makeMarkers()
        self.detection_markers_pub.publish((self.register_markers))
        



def main():
    rospy.init_node("marker_detector_node", anonymous=False)
    rate = rospy.Rate(20)
    markDetNode = marker_detector_node()
    while not rospy.is_shutdown() :
        markDetNode.registerObjects()
        rate.sleep()
    rospy.spin()

        
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass