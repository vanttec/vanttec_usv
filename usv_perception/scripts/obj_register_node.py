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
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from usv_perception.msg import obj_registered
from usv_perception.msg import obj_registered_list
from geometry_msgs.msg import Pose2D #Message to function Pose2D
from geometry_msgs.msg import Vector3 #Message to function Pose2D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


import matplotlib.pyplot as plt


class register_node:
    def __init__(self):

        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.callback_yolo_det)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)
        rospy.Subscriber("/vectornav/ins_2d/ECEF_pose", Vector3, self.callback_ECEF_pose)
        rospy.Subscriber("/vectornav/ins_2d/ecef_ref", Vector3, self.callback_ECEF_ref)

        self.objects_detected_lidar = obj_detected_list()
        #self.obj_detected_yolo = obj_detected_list()
        self.obj_trackId = {}
        self.NED_pose = Pose2D()
        self.ECEF_pose = Vector3()
        self.ECEF_ref = Vector3()
        self.register_amount = 0
        self.objects_registered = obj_registered_list()
        self.register_markers = MarkerArray()
        self.objects_to_filter = obj_detected_list()
        self.register_node_pub = rospy.Publisher('/usv_perception/objects_registered', obj_registered_list, queue_size=10)
        self.register_markers_pub = rospy.Publisher('/usv_perception/register_markers', MarkerArray, queue_size=10)

    def objs_callback(self,data):
        self.objects_detected_lidar = data
        #rospy.loginfo("Detecting currently: " + str(len(data.objects)) + " objects")

        
    def callback_NED_pose(self,data):
        self.NED_pose = data

    def callback_ECEF_pose(self,data):
        self.ECEF_pose = data

    def callback_ECEF_ref(self,data):
        self.ECEF_ref = data
    ## First function to change the object_registered list coordinates from BOAT frame to GPS frame - VN


    def coordBodyToNED(self):
        #Obstacle coordinates to NED
        rospy.loginfo("Transforming from body to NED")
        self.objects_to_filter = self.objects_detected_lidar
        for i in self.objects_to_filter.objects:      
            #rospy.loginfo("Body coords are " + str(i.X) + " X and " + str(i.Y) + " Y")
            p = np.array([i.X, -i.Y])
            J = self.rotation_matrix()
            n = J.dot(p)
            i.X = n[0] + self.NED_pose.x #Offset en X por el LIDAR
            i.Y = n[1] + self.NED_pose.y
            rospy.loginfo("NED coords are " + str(i.X) + " X and " + str(i.Y) + " Y")
    

    
            
            #rospy.loginfo("New coord: " + str(i.X) + " X and: " + str(i.Y) + " Y" )
    
    def rotation_matrix(self):
        '''
        @name: rotation_matrix
        @brief: Transformation matrix template.
        @param: angle: angle of rotation
        @return: J: transformation matrix
        '''
        J = np.array([[math.cos(self.NED_pose.theta), -1*math.sin(self.NED_pose.theta)],
                      [math.sin(self.NED_pose.theta), math.cos(self.NED_pose.theta)]])
        return (J)


        #Da los obstacles con las coordenadas de NED


        
    ## Creation of object with properties for passing each object registered

        
            


    ## Second function to filter objects X, Y to now objects registered, iterating the length of objects registere 

    def filterDetectedObjects(self):
        for detected_object in self.objects_to_filter.objects:
            rospy.loginfo("Filtering an object")
            detected_object.R = 1 ## Copy value to determine if structure if positions are already registered
            if(len(self.objects_registered.objects_reg) == 0):
                rospy.loginfo("No objects registered, registering first object!")
                detected_object.R = 0
                newReg = obj_registered()
                newReg.X = detected_object.X
                newReg.Y = detected_object.Y
                newReg.color = detected_object.color
                newReg.clase = detected_object.clase
                newReg.id = self.nextID()
                ##string color
                ##string clase
                ##float32 X
                ##float32 Y
                ##float32 R
                ##int64 id
                self.objects_registered.objects_reg.append(newReg)
                self.objects_registered.len += 1
                rospy.loginfo("Registered object!")
                
            
            for reg_object in self.objects_registered.objects_reg:

                sharesX = (detected_object.X > (reg_object.X - 1)) & (detected_object.X < (reg_object.X + 1))
                sharesY = (detected_object.Y > (reg_object.Y - 1)) & (detected_object.Y < (reg_object.Y + 1))
                
                if (sharesX & sharesY):
                    detected_object.R = 0
                    reg_object.numDetections += 1
                    


                
            

    ## Third function to pass through the object if filter boolean is false
    def makeRegister(self):
        for detected_object in self.objects_to_filter.objects:
            
        
            if (detected_object.R != 0):
                rospy.loginfo("Registered object!")
                newReg = obj_registered()
                newReg.X = detected_object.X
                newReg.Y = detected_object.Y
                rospy.loginfo("REG NED Coords " + str(newReg.X) + " X and " + str(newReg.Y) + " Y")
                newReg.color = detected_object.color
                newReg.clase = detected_object.clase
                newReg.id = self.nextID()
                ##string color
                ##string clase
                ##float32 X
                ##float32 Y
                ##float32 R
                ##int64 id
                self.objects_registered.objects_reg.append(newReg)
                self.objects_registered.len += 1
                



    def nextID(self):

        return (self.objects_registered.len + 1)
            

    def makeMarkers(self):
        
        tempMarkerArray = MarkerArray()

        for registered_object in self.objects_registered.objects_reg:

            if (registered_object.numDetections > 10):
            
                tempMarker = Marker()
                tempObjReg = obj_registered()
                
                
                tempObjReg.X = registered_object.X
                tempObjReg.Y = registered_object.Y
                tempObjReg.id = registered_object.id
                tempObjReg.color = registered_object.color

                #This sections transforms the NED coordinates to body to visualize in RViz
                rospy.loginfo("MarkNED coords " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")
                n = np.array([tempObjReg.X - self.NED_pose.x, tempObjReg.Y - self.NED_pose.y])
                J = self.rotation_matrix()
                J = np.linalg.inv(J)
                b = J.dot(n)
                tempObjReg.X = b[0]
                tempObjReg.Y = b[1]
                rospy.loginfo("MarkBody coords are " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")

                tempMarker.header.frame_id = "vtec_s3/velodyne"
                tempMarker.header.stamp = rospy.Time()
                tempMarker.type = tempMarker.SPHERE
                tempMarker.id = tempObjReg.id
                tempMarker.pose.position.z = 0
                tempMarker.pose.position.x = tempObjReg.X
                tempMarker.pose.position.y = -tempObjReg.Y
                tempMarker.color.a = 1
                tempMarker.color.r = 0
                tempMarker.color.g = 1
                tempMarker.color.b = 0
                tempMarker.scale.x = 0.3
                tempMarker.scale.y = 0.3
                tempMarker.scale.z = 0.3

                tempMarkerArray.markers.append(tempMarker)
                
        self.register_markers = tempMarkerArray
            


    def registerObjects(self):
        self.coordBodyToNED()
        self.filterDetectedObjects()
        self.makeRegister()
        self.makeMarkers()
        self.register_node_pub.publish((self.objects_registered))
        self.register_markers_pub.publish((self.register_markers))
        



def main():
    rospy.init_node("obj_registry_node", anonymous=False)
    rate = rospy.Rate(5)
    registryNode = register_node()
    while not rospy.is_shutdown() :
        registryNode.registerObjects()
        rate.sleep()
    rospy.spin()

        
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass