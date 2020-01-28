#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from usv_perception.srv import color_id
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

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
    if np.median(s) < 50:

        predicted_color = "white"

    else:
        red=['red' for i in range(2)]
        orange=['orange' for i in range(2)]
        yellow=['yellow' for i in range(3)]
        green=['green' for i in range(11)]
        blue=['blue' for i in range(8)]
        pink=['-' for i in range(8)]

        hist = cv2.calcHist([hsv], [0], None, [36], [0, 180])
        colors = red + orange + yellow + green + blue + pink + red

        hist_list = [hist[i][0] for i in range(len(hist))]
        hist_dic = dict(zip(hist_list, colors))

        # uncomment to see the frequencies of each color region
        # print(hist_dic)

        # predicted_color contains the region of maximum frequency
        ret = str(hist_dic[max(hist_dic)])

    return ret


if __name__ == '__main__':

    rospy.init_node('color_srv')
    rospy.loginfo("Node created!")

    service = rospy.Service("/get_color", color_id, callback_color)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
