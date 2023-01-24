#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from usv_perception.srv import color_id
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
red=['red' for i in range(2)]
orange=['orange' for i in range(2)]
yellow=['yellow' for i in range(3)]
green=['green' for i in range(11)]
blue=['blue' for i in range(8)]
pink=['-' for i in range(8)]
colors = red + orange + yellow + green + blue + pink + red




def callback_color(img):
    global bridge

    image = img.imagen
    x = img.x
    y = img.y
    h = img.h
    w = img.w
    image = bridge.imgmsg_to_cv2(image, "bgr8")

    image = image[y:y+h,x:x+w]
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    # split each channel from hsv tensor
    (h, s, v) = cv2.split(image)


    ret = ""
    
    
    hist = cv2.calcHist([hsv], [0], None, [36], [0, 180])
    

    hist_list = [hist[i][0] for i in range(len(hist))]

    max_index = hist_list.index(max(hist_list))

    ret = colors[max_index]

    return ret


if __name__ == '__main__':

    rospy.init_node('color_srv')
    rospy.loginfo("Node created!")

    service = rospy.Service("/get_color", color_id, callback_color)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
