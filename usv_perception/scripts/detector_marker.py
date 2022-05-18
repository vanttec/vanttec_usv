#!/usr/bin/env python

import rospy
import numpy as np
from genpy import Time
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from usv_perception.msg     import obj_detected, obj_detected_list
from usv_perception.msg     import obj_registered
from geometry_msgs.msg      import Pose2D #Message to function Pose2D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#This script creates markers in rviz that directly represent the obstacles detected

class marker_detector_node:
    def __init__(self):
        self.objects_detected_lidar = obj_detected_list()
        self.objects_detected_yolo = obj_detected_list()
        self.detection_markers = MarkerArray()
        self.NED_pose = Pose2D()
        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.lidar_detection)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_detection)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)

        self.detection_markers_pub = rospy.Publisher('/usv_perception/detection_markers', MarkerArray, queue_size=10)

    def lidar_detection(self, data):
        self.objects_detected_lidar = data
        # rospy.loginfo(self.objects_detected_lidar.objects)

    def yolo_detection(self, data):
        obj_list = obj_detected_list()
        # rospy.loginfo(len(data.bounding_boxes))
        for objt in data.bounding_boxes:
            if np.isfinite(objt.x) and np.isfinite(objt.y):
                obj = obj_detected()
                obj.X = objt.x
                obj.Y = objt.y
                obj_list.objects.append(obj)
            # obj_list.len = len(data.bounding_boxes)
        self.objects_detected_yolo = obj_list
        # rospy.loginfo(obj_list)

    def callback_NED_pose(self, data):
        self.NED_pose = data

    ## First function to change the object_registered list coordinates from BOAT frame to GPS frame - VN
            #rospy.loginfo("New coord: " + str(i.X) + " X and: " + str(i.Y) + " Y" )

    def make_markers(self):
        temp_marker_array = MarkerArray()

        for registered_object in self.objects_detected_lidar.objects:
            tempMarker = Marker()
            tempMarker.header.frame_id = "velodyne"
            tempMarker.header.stamp = rospy.Time()
            tempMarker.type = tempMarker.SPHERE
            tempMarker.id = registered_object.id
            tempMarker.pose.position.z = 0
            tempMarker.pose.position.x = registered_object.X
            tempMarker.pose.position.y = registered_object.Y
            tempMarker.pose.orientation.x = 0
            tempMarker.pose.orientation.y = 0
            tempMarker.pose.orientation.z = 0
            tempMarker.pose.orientation.w = 1
            tempMarker.color.a = 1
            tempMarker.color.r = 1
            tempMarker.color.g = 0
            tempMarker.color.b = 1
            tempMarker.scale.x = 0.3
            tempMarker.scale.y = 0.3
            tempMarker.scale.z = 0.3
            temp_marker_array.markers.append(tempMarker)

        # rospy.loginfo(len(self.objects_detected_yolo.objects))
        for registered_object in self.objects_detected_yolo.objects:            
            tempMarker = Marker()
            tempMarker.header.frame_id = "velodyne"
            tempMarker.header.stamp = rospy.Time()
            tempMarker.type = tempMarker.SPHERE
            id_start = len(temp_marker_array.markers)
            tempMarker.id = id_start
            tempMarker.pose.position.z = 0
            tempMarker.pose.position.x = registered_object.X #-0.3
            tempMarker.pose.position.y = -registered_object.Y
            tempMarker.pose.orientation.x = 0
            tempMarker.pose.orientation.y = 0
            tempMarker.pose.orientation.z = 0
            tempMarker.pose.orientation.w = 1
            tempMarker.color.a = 1
            tempMarker.color.r = 0
            tempMarker.color.g = 1
            tempMarker.color.b = 0
            tempMarker.scale.x = 0.3
            tempMarker.scale.y = 0.3
            tempMarker.scale.z = 0.3
            temp_marker_array.markers.append(tempMarker)

        self.detection_markers = temp_marker_array

def main():
    rospy.init_node("marker_detector_node", anonymous=False)
    rate = rospy.Rate(20)
    markDetNode = marker_detector_node()
    while not rospy.is_shutdown() :
        markDetNode.make_markers()
        markDetNode.detection_markers_pub.publish(markDetNode.detection_markers)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass