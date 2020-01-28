#!/usr/bin/env python

from include.detector_lib import Detector
from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

from usv_perception.srv import color_id
#from srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list

import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np
import time
import rospy
import cv2
import math

import os

class Color():
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED  = '\033[91m'
    DONE  = '\033[0m'


class Detection_Node:
    def __init__(self):

        self.bridge = CvBridge()
        self.image = np.zeros((560,1000,3),np.uint8)
        self.depth = np.zeros((560,1000,3),np.uint8)
        self.points_list = [[0,0,0]]*921600


        rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback_zed_img)
        rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_zed_cp)

        self.detector_pub = rospy.Publisher('/usv_perception/yolo_zed/objects_detected', obj_detected_list, queue_size=10)


    def callback_zed_img(self,img):
        """ ZED rect_image callback"""
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")


    def callback_zed_cp(self,ros_cloud):
        self.points_list = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = ("x", "y", "z")))
        #skip_nans=True

    def send_message(self, color, msg):
        """ Publish message to ros node. """

        msg = color + msg + Color.DONE
        rospy.loginfo(msg)

    def calculate_color(self,img,x,y,w,h):
        """ Calculates distance using get_distance service """

        img = self.bridge.cv2_to_imgmsg(img, encoding = "bgr8")
        rospy.wait_for_service("/get_color")

        service = rospy.ServiceProxy("/get_color", color_id)
        color = service(img,x,y,w,h)

        return color


    def detect(self):
        """ Performs object detection and publishes coordinates. """

        # Initialize detector
        self.send_message(Color.GREEN, "[INFO] Initializing TinyYOLOv3 detector.")
        dirname = os.path.dirname(__file__)
        tiny3_file = dirname + "/yolo-config/tiny3.cfg"
        weights_file = dirname + "/yolo-config/tiny3_68000.weights"
        names_file = dirname + "/yolo-config/obj.names"

        det = Detector(tiny3_file,
                       weights_file,
                       names_file)

        (H, W) = (None, None)

        # Load model
        self.send_message(Color.GREEN, "[INFO] Loading network model.")
        net = det.load_model()
        print(net)

        # Initilialize Video Stream
        self.send_message(Color.GREEN, "[INFO] Starting video stream.")

        counter = 0
        dets = 0
        nondets = 0
        detect = True
        fps = FPS().start()
        boxes, confidences, indices, cls_ids, colors, ids, distances = [], [], [], [], [], [], []
        zed_cam_size = 1280

        ret = True
        while not rospy.is_shutdown():
            # Grab next frame

            #ret, frame = video.read()
            frame = self.image
            #cap = cv2.VideoCapture("/home/nvidia/opencv_install/pajarito/bird.jpg")
            #hasFrame, frame = cap.read()
            color = ""
            diststring = ""

            ##AQUI SE MODIFICA EL VIDEO

            #frame = add_brightness(frame)
            #frame = add_darkness(frame)

            if cv2.waitKey(1) & 0xFF == ord ('q'):
                self.send_message(Color.RED, "[DONE] Quitting program.")
                break

            frame = imutils.resize(frame, width=1000)

            (H, W) = frame.shape[:2]
            if det.get_w() is None or det.get_h() is None:
                det.set_h(H)
                det.set_w(W)

            # Perform detection

            detect = True
            dets += 1
            # Get bounding boxes, condifences, indices and class IDs
            boxes, confidences, indices, cls_ids = det.get_detections(net, frame)
            # Publish detections



            # If there were any previous detections, draw them
            colors = []
            distances = []
            obj_list = obj_detected_list()
            len_list = 0

            for ix in indices:
                i = ix[0]

                box = boxes[i]
                x, y, w, h = box
                x, y, w, h = int(x), int(y), int(w), int(h)

                if detect == True:
                    color = self.calculate_color(frame,x,y,h,w)
                    print(zed_cam_size)

                    p1= int((x+w/2)*zed_cam_size/1000) #1.28 hd
                    p2= int((y+h/2)*zed_cam_size/1000)

                    #1280 si es HD , 672

                    ind = p1+p2*zed_cam_size

                    d_list = self.points_list[ind-15:ind+15]



                    d_list2_Y = []
                    for j in d_list:
                        if str(j[0]) != 'nan' and str(j[0]) != 'inf':
                            d_list2_Y.append(j[0])

                    d_list2_X = []
                    for j in d_list:
                        if str(j[1]) != 'nan' and str(j[1]) != 'inf':
                            d_list2_X.append(j[1])

                    d_list = d_list2_Y
                    d_list_x = d_list2_X


                    if len(d_list_x) != 0:
                        dist_x = np.mean(d_list_x)
                    else:
                        dist_x = 'nan'

                    if len(d_list) != 0:
                        dist = np.mean(d_list)
                    else:
                        dist = 'nan'


                    if (dist < .30):
                        diststring = "OUT OF RANGE"
                    else:
                        diststring = str(dist) + " m"

                    color = str(color.color)
                    colors.append(color)
                    distances.append(dist)



                    if str(dist) != 'nan' and str(dist_x) != 'nan':
                        obj = obj_detected()
                        #print(p1,p2)
                        obj.x = x
                        obj.y = y
                        obj.h = h
                        obj.w = w
                        obj.X = dist
                        obj.Y = dist_x
                        obj.color = color
                        obj.clase = 'bouy' if cls_ids[i] == 0 else 'marker'
                        len_list += 1
                        obj_list.objects.append(obj)


                    det.draw_prediction(frame, cls_ids[i], confidences[i], color,diststring, x, y, x+w, y+h)

            det_str = "Det: {}, BBoxes {}, Colors {}, Distance {}".format(dets, boxes, colors, distances)
            self.send_message(Color.BLUE, det_str)
            fps.update()
            obj_list.len = len_list
            self.detector_pub.publish(obj_list)
            cv2.line(frame, (500,560), (500,0), (255,0,0))
            fps.stop()

            info = [
                ("Detects: ", dets),
                ("No detects: ", nondets),
                ("FPS", "{:.2F}".format(fps.fps())),
            ]
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, det.get_h() - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Show current frame
            #cv2.imshow("Frame", frame)
            #print(self.depth)

            #cv2.waitKey(3)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('yolo_zed')

        rate = rospy.Rate(20) # 20Hz
        D = Detection_Node()
        D.detect()
    except rospy.ROSInterruptException:
        pass
