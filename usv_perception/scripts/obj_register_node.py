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
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.zed_yolo_detection_callback)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)

        self.register_node_pub = rospy.Publisher('/usv_perception/objects_registered', obj_registered_list, queue_size=10)
        self.register_markers_pub = rospy.Publisher('/usv_perception/register_markers', MarkerArray, queue_size=10)

        # Pose in different frames
        self.NED_pose = Pose2D()
        self.ECEF_pose = Vector3()
        self.ECEF_ref = Vector3()

        # Sensor offsets to body frame
        self.lidar_offset_x = rospy.get_param("~lidar_x_offset",0)
        self.zed_offset_x = rospy.get_param("~zed_x_offset",0)

        self.lidar_detected_objects = obj_detected_list()
        self.lidar_objects_ned = obj_detected_list()
        self.zed_detected_objects = obj_detected_list()
        self.zed_objects_ned = obj_detected_list()

        self.registered_objects = obj_registered_list()
        self.register_markers = MarkerArray()

    def objs_callback(self,data):
        self.lidar_detected_objects = data
        #rospy.loginfo("Detecting currently: " + str(len(data.objects)) + " objects")

    def zed_yolo_detection_callback(self, data):
        obj_list = obj_detected_list()
        for objt in data.bounding_boxes:
            if np.isfinite(objt.x) and np.isfinite(objt.y):
                zed_obj = obj_detected()
                zed_obj.X = objt.x
                zed_obj.Y = objt.y
                obj_list.objects.append(zed_obj)
                obj_list.len = len(data.bounding_boxes)
        self.zed_detected_objects = obj_list
        # rospy.loginfo(self.zed_detected_objects)

    def callback_NED_pose(self,data):
        self.NED_pose = data

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
        self.lidar_objects_ned = self.lidar_detected_objects
        # rospy.loginfo(self.lidar_objects_ned)
        for obj in self.lidar_objects_ned.objects:      
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
        for obj in self.zed_objects_ned.objects:
            p = np.array([obj.X + self.zed_offset_x, obj.Y])
            J = self.rotation_matrix(self.NED_pose.theta)
            n = J.dot(p)
            # rospy.loginfo("ZED coords are " + str(obj.X) + " X and " + str(obj.Y) + " Y")
            obj.X = n[0] + self.NED_pose.x
            obj.Y = n[1] + self.NED_pose.y
            # rospy.loginfo("ZED NED coords are " + str(obj.X) + " X and " + str(obj.Y) + " Y")

    ## Second function to filter objects X, Y to now objects registered, iterating the length of objects registere 
    def filter_detected_objects(self):
        for lidar_object_to_filter in self.lidar_objects_ned.objects:
            lidar_object_to_filter.R = 0 # Object saved
            if(len(self.registered_objects.objects_reg) == 0):
                lidar_object_to_filter.R = 0
                new_reg = obj_registered()
                new_reg.X = lidar_object_to_filter.X
                new_reg.Y = lidar_object_to_filter.Y
                new_reg.color = lidar_object_to_filter.color
                new_reg.clase = lidar_object_to_filter.clase
                new_reg.id = self.registered_objects.len + 1
                self.registered_objects.objects_reg.append(new_reg)
                self.registered_objects.len += 1
                # rospy.loginfo("No objects saved, first object registered!")

            for reg_object in self.registered_objects.objects_reg:
                distance = math.sqrt(math.pow(reg_object.X - lidar_object_to_filter.X, 2)+math.pow(reg_object.Y - lidar_object_to_filter.Y, 2))

                if (distance < 1):
                    lidar_object_to_filter.R = 1 # Object already seen and saved
                    reg_object.numDetections += 1
                    if (reg_object.numDetections > 15):
                        reg_object.R = 2 # Object is persistent (appears consistently as obstacle)
                        # rospy.loginfo("Object persistent")
            
    ## Third function to pass through the object if filter boolean is false
    def make_register(self):
        for registered_object in self.registered_objects.objects_reg:
            if (registered_object.R == 2):
                # rospy.loginfo("Object registered!")
                new_reg = obj_registered()
                new_reg.X = registered_object.X
                new_reg.Y = registered_object.Y
                # rospy.loginfo("REG NED Coords " + str(new_reg.X) + " X and " + str(new_reg.Y) + " Y")
                new_reg.color = registered_object.color
                new_reg.clase = registered_object.clase
                new_reg.R = 3 # Object is finally registered
                new_reg.id = self.registered_objects.len + 1
                self.registered_objects.objects_reg.append(new_reg)
                self.registered_objects.len += 1

    def verify_with_ZED(self):
        for zed_detected_object in self.zed_objects_ned.objects:
            for reg_object in self.registered_objects.objects_reg:
                distance = math.sqrt(math.pow(reg_object.X - zed_detected_object.X, 2)+math.pow(reg_object.Y - zed_detected_object.Y, 2))
                if (distance < 1):
                    reg_object.R = 4 # Registered object is verified with ZED-Yolo detection
                    rospy.loginfo("Object verified!")

    def make_markers(self):
        tempMarkerArray = MarkerArray()
        for registered_object in self.registered_objects.objects_reg:
            if (registered_object.R > 2): #If registered object is registered or verified
                tempMarker = Marker()
                tempObjReg = obj_registered()
                tempObjReg.X = registered_object.X
                tempObjReg.Y = registered_object.Y
                #This sections transforms the NED coordinates to body to visualize in RViz
                # rospy.loginfo("MarkNED coords " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")
                n = np.array([tempObjReg.X - self.NED_pose.x, tempObjReg.Y - self.NED_pose.y])
                J = self.rotation_matrix(self.NED_pose.theta)
                J = np.linalg.inv(J)
                b = J.dot(n)
                tempObjReg.X = b[0]
                tempObjReg.Y = b[1]
                # rospy.loginfo("MarkBody coords are " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")

                tempMarker.header.frame_id = "velodyne"
                tempMarker.header.stamp = rospy.Time()
                tempMarker.type = tempMarker.SPHERE
                tempMarker.id = registered_object.id
                tempMarker.pose.position.z = 0
                tempMarker.pose.position.x = tempObjReg.X
                tempMarker.pose.position.y = -tempObjReg.Y 
                tempMarker.pose.orientation.x = 0
                tempMarker.pose.orientation.y = 0
                tempMarker.pose.orientation.z = 0
                tempMarker.pose.orientation.w = 1             
                tempMarker.color.a = 1
                
                if (registered_object.R == 3):
                    tempMarker.color.r = 0
                    tempMarker.color.g = 0
                    tempMarker.color.b = 1
                else:
                    if (registered_object.R == 4):
                        tempMarker.color.r = 0
                        tempMarker.color.g = 1
                        tempMarker.color.b = 1
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
        self.register_node_pub.publish((self.registered_objects))
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