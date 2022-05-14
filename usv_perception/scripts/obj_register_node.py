#!/usr/bin/env python
#The code on top tells the compiler to use the main python

import rospy
import math
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from usv_perception.msg import obj_registered
from usv_perception.msg import obj_registered_list
from geometry_msgs.msg import Pose2D #Message to function Pose2D
from geometry_msgs.msg import Vector3 #Message to function Pose2D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class register_node:
    def __init__(self):
        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_detection)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)
        rospy.Subscriber("/vectornav/ins_2d/ECEF_pose", Vector3, self.callback_ECEF_pose)
        rospy.Subscriber("/vectornav/ins_2d/ecef_ref", Vector3, self.callback_ECEF_ref)

        self.register_node_pub = rospy.Publisher('/usv_perception/objects_registered', obj_registered_list, queue_size=10)
        self.register_markers_pub = rospy.Publisher('/usv_perception/register_markers', MarkerArray, queue_size=10)

        # Pose in different frames
        self.NED_pose = Pose2D()
        self.ECEF_pose = Vector3()
        self.ECEF_ref = Vector3()

        # Sensor offsets to body frame
        self.lidar_offset_x = 0.25
        self.zed_offset_x = 0.52

        self.lidar_detected_objects = obj_detected_list()
        self.zed_detected_objects = BoundingBoxes()
        self.zed_objects_ned = BoundingBoxes()
        self.lidar_registered_objects = obj_registered_list()
        self.register_markers = MarkerArray()
        self.lidar_objects_to_filter = obj_detected_list()

    def objs_callback(self,data):
        self.lidar_detected_objects = data
        #rospy.loginfo("Detecting currently: " + str(len(data.objects)) + " objects")

    def yolo_detection(self, data):
        for obj in data.bounding_boxes:
            if np.isfinite(obj.x) and np.isfinite(obj.y):
                zed_obj = BoundingBox()
                zed_obj.x = obj.x
                zed_obj.y = obj.y
                self.zed_detected_objects.bounding_boxes.append(zed_obj)

    def callback_NED_pose(self,data):
        self.NED_pose = data

    def callback_ECEF_pose(self,data):
        self.ECEF_pose = data

    def callback_ECEF_ref(self,data):
        self.ECEF_ref = data

    
    def rotation_matrix(self, angle):
        '''
        @name: rotation_matrix
        @brief: Transformation matrix template.
        @param: angle: angle of rotation
        @return: J: transformation matrix
        '''
        J = np.array([[math.cos(angle), -1*math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]])
        return (J)

    ## First function to change the object_registered list coordinates from BOAT frame to GPS frame - VN
    def lidar_objects_body2NED(self):
        # rospy.loginfo("Transforming LiDAR readings from body to NED")
        self.lidar_objects_to_filter = self.lidar_detected_objects
        for obj in self.lidar_objects_to_filter.objects:      
            #rospy.loginfo("Body coords are " + str(obj.X) + " X and " + str(obj.Y) + " Y")
            p = np.array([obj.X + self.lidar_offset_x, -obj.Y])
            J = self.rotation_matrix(self.NED_pose.theta)
            n = J.dot(p)
            obj.X = n[0] + self.NED_pose.x
            obj.Y = n[1] + self.NED_pose.y
            # rospy.loginfo("LiDAR NED coords are " + str(obj.X) + " X and " + str(obj.Y) + " Y")
    
    def zed_objects_body2NED(self):
        # rospy.loginfo("Transforming ZED readings from body to NED")
        self.zed_objects_ned = self.zed_detected_objects
        for obj in self.zed_objects_ned.bounding_boxes:    
            p = np.array([obj.x + self.zed_offset_x, obj.y])
            J = self.rotation_matrix(self.NED_pose.theta)
            n = J.dot(p)
            obj.x = n[0] + self.NED_pose.x
            obj.x = n[1] + self.NED_pose.y
            # rospy.loginfo("ZED NED coords are " + str(obj.x) + " X and " + str(obj.y) + " Y")


    ## Second function to filter objects X, Y to now objects registered, iterating the length of objects registere 
    def filter_detected_objects(self):
        for lidar_object_to_filter in self.lidar_objects_to_filter.objects:
            rospy.loginfo("Filtering an object")
            lidar_object_to_filter.R = 1 ## Copy value to determine if structure if positions are already registered
            if(len(self.lidar_registered_objects.objects_reg) == 0):
                lidar_object_to_filter.R = 0
                new_reg = obj_registered()
                new_reg.X = lidar_object_to_filter.X
                new_reg.Y = lidar_object_to_filter.Y
                new_reg.color = lidar_object_to_filter.color
                new_reg.clase = lidar_object_to_filter.clase
                new_reg.id = self.lidar_registered_objects.len + 1
                self.lidar_registered_objects.objects_reg.append(new_reg)
                self.lidar_registered_objects.len += 1
                rospy.loginfo("No objects saved, first object registered!")
            
            for reg_object in self.lidar_registered_objects.objects_reg:
                sharesX = (lidar_object_to_filter.X > (reg_object.X - 1)) & (lidar_object_to_filter.X < (reg_object.X + 1))
                sharesY = (lidar_object_to_filter.Y > (reg_object.Y - 1)) & (lidar_object_to_filter.Y < (reg_object.Y + 1))
                
                if (sharesX & sharesY): 
                    lidar_object_to_filter.R = 0 #R=0 detected object registered / R=1 detected object not registered
                    reg_object.numDetections += 1
                    if (reg_object.numDetections > 15):
                        reg_object.R = 2 #R=2 means reg object is persistent       
            
    ## Third function to pass through the object if filter boolean is false
    def make_register(self):
        for lidar_object_to_filter in self.lidar_objects_to_filter.objects:
            if (lidar_object_to_filter.R != 0):
                rospy.loginfo("Object registered!")
                new_reg = obj_registered()
                new_reg.X = lidar_object_to_filter.X
                new_reg.Y = lidar_object_to_filter.Y
                rospy.loginfo("REG NED Coords " + str(new_reg.X) + " X and " + str(new_reg.Y) + " Y")
                new_reg.color = lidar_object_to_filter.color
                new_reg.clase = lidar_object_to_filter.clase
                new_reg.R = 3 # R=3, means that reg object is registered
                new_reg.id = self.lidar_registered_objects.len + 1
                self.lidar_registered_objects.objects_reg.append(new_reg)
                self.lidar_registered_objects.len += 1


    def verify_with_ZED(self):
        for zed_detected_object in self.zed_objects_ned.bounding_boxes:
            for reg_object in self.lidar_registered_objects.objects_reg:
                sharesX = (zed_detected_object.x > (reg_object.X - 1)) & (zed_detected_object.x < (reg_object.X + 1))
                sharesY = (zed_detected_object.y > (reg_object.Y - 1)) & (zed_detected_object.y < (reg_object.Y + 1))
                if (sharesX & sharesY): 
                    reg_object.R = 4 #R=4 means reg object is verified with ZED

    def nextID(self):
        return (self.lidar_registered_objects.len + 1)

    def make_markers(self):
        tempMarkerArray = MarkerArray()
        for registered_object in self.lidar_registered_objects.objects_reg:
            if (registered_object.R == 3): #If registered object is considered persistent
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
                
                if (registered_object.R == 3):
                    tempMarker.color.r = 0
                    tempMarker.color.g = 1
                    tempMarker.color.b = 0
                else:
                    if (registered_object.R == 4):
                        tempMarker.color.r = 1
                        tempMarker.color.g = 0
                        tempMarker.color.b = 0
                tempMarker.scale.x = 0.3
                tempMarker.scale.y = 0.3
                tempMarker.scale.z = 0.3

                tempMarkerArray.markers.append(tempMarker)
        self.register_markers = tempMarkerArray

    def registerObjects(self):
        self.lidar_objects_body2NED()
        self.zed_objects_body2NED()
        self.filter_detected_objects()
        self.make_register()
        self.verify_with_ZED()
        self.make_markers()
        self.register_node_pub.publish((self.lidar_registered_objects))
        self.register_markers_pub.publish((self.register_markers))
        
def main():
    rospy.init_node("obj_registry_node", anonymous=False)
    rate = rospy.Rate(5)
    registryNode = register_node()
    while not rospy.is_shutdown() :
        registryNode.registerObjects()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass