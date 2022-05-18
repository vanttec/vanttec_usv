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
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.zed_yolo_detection)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)

        self.register_node_pub = rospy.Publisher('/usv_perception/objects_registered', obj_detected_list, queue_size=10)
        self.register_markers_pub = rospy.Publisher('/usv_perception/register_markers', MarkerArray, queue_size=10)

        # Pose in different frames
        self.NED_pose = Pose2D()
        self.ECEF_pose = Vector3()
        self.ECEF_ref = Vector3()

        # Sensor offsets to body frame
        self.lidar_offset_x = rospy.get_param("~lidar_x_offset", 0)
        self.zed_offset_x = rospy.get_param("~zed_x_offset", 0)

        self.lidar_detected_objects = obj_detected_list()
        self.zed_detected_objects = obj_detected_list()
        self.zed_objects_ned = obj_detected_list()
        self.lidar_objects_ned = obj_detected_list()
        self.registered_objects = obj_detected_list()

        self.lidar_registered_objects = obj_registered_list()
        self.register_markers = MarkerArray()

    def objs_callback(self,data):
        self.lidar_detected_objects = data

    def zed_yolo_detection(self, data):
        obj_list = obj_detected_list()
        for objt in data.bounding_boxes:
            if np.isfinite(objt.x) and np.isfinite(objt.y):
                zed_obj = obj_detected()
                zed_obj.X = objt.x
                zed_obj.Y = -objt.y
                obj_list.objects.append(zed_obj)
            obj_list.len = len(data.bounding_boxes)
        self.zed_detected_objects = obj_list

    def callback_NED_pose(self,data):
        self.NED_pose = data

    def rotation_matrix(self, angle, x, offset_x, y):
        J = np.array([[math.cos(angle), -1*math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]])
        p = np.array([x + offset_x, y])
        n = J.dot(p)
        return (n)

    def lidar_objects_body2NED(self):
        self.lidar_objects_ned.objects = list(self.lidar_detected_objects.objects)
        self.lidar_objects_ned.len = len(self.lidar_detected_objects.objects)
        for obj in self.lidar_objects_ned.objects:
            n = self.rotation_matrix(self.NED_pose.theta, obj.X, self.lidar_offset_x,-obj.Y)
            obj.X = n[0] + self.NED_pose.x
            obj.Y = n[1] + self.NED_pose.y
    
    def zed_objects_body2NED(self):
        self.zed_objects_ned.objects = list(self.zed_detected_objects.objects)
        self.zed_objects_ned.len = len(self.zed_objects_ned.objects)
        for obj in self.zed_objects_ned.objects:    
            n = self.rotation_matrix(self.NED_pose.theta, obj.X, self.zed_offset_x,obj.Y)
            obj.x = n[0] + self.NED_pose.x
            obj.x = n[1] + self.NED_pose.y

    def register_objects(self):
        rospy.loginfo("register")
        objects_to_register = obj_detected_list()
        if(self.registered_objects.len == 0):
            self.registered_objects.objects = list(self.lidar_objects_ned.objects)
            self.registered_objects.len = len(self.lidar_objects_ned.objects)
            # rospy.loginfo("First objects registered")

        # rospy.loginfo(len(self.registered_objects.objects))
        rospy.loginfo(self.registered_objects.len)
        # rospy.loginfo(len(self.lidar_objects_ned.objects))
        # rospy.loginfo(self.lidar_objects_ned.len)
        # rospy.loginfo(len(objects_to_register.objects))
        # rospy.loginfo(objects_to_register.len)

        for obj_registered in self.registered_objects.objects:
            for lidar_obj in self.lidar_objects_ned.objects:
                distance = math.sqrt(math.pow(obj_registered.X - lidar_obj.X, 2)+math.pow(obj_registered.Y - lidar_obj.Y, 2))
                if distance < 0.3:
                    # rospy.loginfo(obj_registered)
                    # rospy.loginfo(lidar_obj)
                    # rospy.loginfo("distance: " + str(distance))
                    objects_to_register.objects.append(lidar_obj)
                    objects_to_register.len += 1
        
        self.registered_objects.objects = []
        self.registered_objects.len = 0
        self.registered_objects.objects = objects_to_register.objects
        self.registered_objects.len = objects_to_register.len

        for obj_registered in self.registered_objects.objects:
            for zed_obj in self.zed_objects_ned.objects:
                distance = math.sqrt(math.pow(obj_registered.X - zed_obj.X, 2)+math.pow(obj_registered.Y - zed_obj.Y, 2))
                if distance > 0.3:
                    # rospy.loginfo(obj_registered)
                    self.registered_objects.objects.remove(obj_registered)
                    self.registered_objects.len -=1
        
        # rospy.loginfo("Objects registered")

    def make_markers(self):
        # rospy.loginfo(len(self.registered_objects.objects))
        tempMarkerArray = MarkerArray()
        for registered_object in self.registered_objects.objects:
            # rospy.loginfo("Marker created")
            tempMarker = Marker()
            tempObjReg = obj_registered()
            tempObjReg.X = registered_object.X
            tempObjReg.Y = registered_object.Y

            tempMarker.header.frame_id = "world"
            tempMarker.header.stamp = rospy.Time()
            tempMarker.type = tempMarker.SPHERE
            tempMarker.id = registered_object.id
            tempMarker.pose.position.z = 0
            tempMarker.pose.position.x = tempObjReg.X
            tempMarker.pose.position.y = tempObjReg.Y 
            tempMarker.pose.orientation.x = 0
            tempMarker.pose.orientation.y = 0
            tempMarker.pose.orientation.z = 0
            tempMarker.pose.orientation.w = 1             
            tempMarker.color.a = 1
            tempMarker.color.r = 1
            tempMarker.color.g = 0
            tempMarker.color.b = 0
            tempMarker.scale.x = 0.3
            tempMarker.scale.y = 0.3
            tempMarker.scale.z = 0.3

            tempMarkerArray.markers.append(tempMarker)
        rospy.loginfo(self.registered_objects.len)
        self.register_markers = tempMarkerArray

    def registerObjects(self):
        self.lidar_objects_body2NED()
        self.zed_objects_body2NED()
        self.register_objects()
        self.make_markers()
        self.register_node_pub.publish(self.registered_objects)
        self.register_markers_pub.publish(self.register_markers)
        
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