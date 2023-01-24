#!/usr/bin/env python


import rospy
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
#from sensor_msgs.msg import Image
import numpy as np
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt




class detection_fusion:
    def __init__(self):

        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.callback_lidar_det)
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.callback_yolo_det)
        
        self.detector_fusion_pub = rospy.Publisher('/usv_perception/sensor_fusion/objects_detected', obj_detected_list, queue_size=10)

        self.obj_detected_lidar = obj_detected_list()
        self.obj_detected_yolo = obj_detected_list()

        self.obj_trackId = {}


    def objDetToDict(self, obj_det_list):

        obj_dict = {}
        for obj in obj_det_list.objects:
            if(obj.id != -1):
                obj_dict[obj.id] = obj

        return obj_dict
            
    def matchDetections(self, obj_det_lidar, obj_det_yolo):

        lidar_points = np.array([[obj.X, obj.Y] for obj in obj_det_lidar.objects])
        tree = KDTree(lidar_points, leaf_size=1) 

        obj_detected_sf = obj_det_lidar

        for obj in obj_det_yolo.objects:

            query_point = np.array([obj.X + 0.30, obj.Y * -1])
            dist, ind = tree.query(query_point.reshape(1, -1), k=1)  

            new_obj = obj_detected()

            if dist < 0.5:
                
                obj_detected_sf.objects[ind[0][0]].color = obj.color

            else:
                new_obj = obj
                obj_detected_sf.objects.append(new_obj)

        return obj_detected_sf

    def plotDetections(self, obj_dets):

        obj_list = []
        
        plt.clf()

        plt.xlim(-5, 5)
        plt.ylim(-5, 5)

        for obj in obj_dets.objects:
            color = obj.color[0] if obj.color != "" else "b"
            plt.scatter(obj.Y, obj.X, color = color)
        plt.pause(0.001)



    def callback_yolo_det(self,msg):

        self.obj_detected_yolo = msg

        obj_det_lidar = self.obj_detected_lidar
        obj_det_yolo =self.obj_detected_yolo

        if(bool(self.obj_trackId)):
            for obj in obj_det_lidar.objects:
                if(self.obj_trackId.has_key(obj.id)):
                    obj.color = self.obj_trackId[obj.id].color

        obj_detected_sf = self.matchDetections(obj_det_lidar, obj_det_yolo)

        self.obj_trackId = self.objDetToDict(obj_detected_sf)

        obj_detected_sf.len = len(obj_detected_sf.objects)

        self.plotDetections(obj_detected_sf)

        self.detector_fusion_pub.publish(obj_detected_sf)


    def callback_lidar_det(self,msg):
        self.obj_detected_lidar = msg
        
        
        
        
  
if __name__ == '__main__':
    try:
        rospy.init_node('detector_fusion')
        rate = rospy.Rate(5)
        DF = detection_fusion()
        rospy.spin()


    except rospy.ROSInterruptException:
        pass