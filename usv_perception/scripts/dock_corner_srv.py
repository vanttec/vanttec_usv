#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math 
from usv_perception.srv import dock_corners,dock_cornersResponse
from geometry_msgs.msg import Point32, PoseArray, Pose
from scipy.spatial import ConvexHull, convex_hull_plot_2d, distance
import matplotlib.pyplot as plt

from cv_bridge import CvBridge, CvBridgeError





c = 0

pub = rospy.Publisher('/usv_perception/lidar_detector/dock', PoseArray, queue_size=10)


def callback(req):
    global c

    points = np.round(np.array([[point.x, point.y] for point in req.pointCoordinates]), 2) * 100 + 250


    #Fing hull shape and fill it
    hull = ConvexHull(points)

    image=np.zeros((500,500,3),np.uint8)
    image2=np.zeros((500,500,3),np.uint8)

    pts = np.array(points[hull.vertices,:], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.fillPoly(image,[pts],(255,255,255))


    #Obtain corners from shape
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,9,0.04)
    dst = cv2.dilate(dst,None)


    image2[dst>0.01*dst.max()]=[255,255,255]
    gray2 = cv2.cvtColor(image2,cv2.COLOR_BGR2GRAY)
    gray2 = cv2.dilate(gray2,None, iterations = 5)

    #find the centroid of the multiple corner detections
    #cv2.imwrite("/home/fitocuan/Pictures/" + str(c) + ".jpg", gray2)
    contours, hierarchy = cv2.findContours(gray2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    res = dock_cornersResponse()

    poseArray = PoseArray()

    cornersArray = []

    for i in contours:
        M = cv2.moments(i)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])


        cX = (cX - 250 )/100.0
        cY = (cY - 250 )/100.0

        #print(cX, cY)

        


        cornersArray.append((cX + req.centerPoint.x, cX + req.centerPoint.y))


        #poseArray.poses.append(pose)

        point = Point32(cX+ req.centerPoint.x ,cY + req.centerPoint.y,0)
        res.dockCoordinates.append(point)
    
    c+=1


    for point in cornersArray:
        for point2 in cornersArray:


            #print(distance.euclidean(point, point2))

            if(distance.euclidean(point, point2) > 4):


                pose = Pose()
                pose2 = Pose()
                pose.position.x = point[0] + req.centerPoint.x
                pose.position.y = point[1] + req.centerPoint.y

                pose2.position.x = point2[0] + req.centerPoint.x
                pose2.position.y = point2[1] + req.centerPoint.y

                poseArray.poses.append(pose)
                poseArray.poses.append(pose2)


                break
        else:
            continue
        break
        


    

    pub.publish(poseArray)



    return res


if __name__ == '__main__':

    rospy.init_node('dock_corners_server')
    rospy.loginfo("Node created!")

    service = rospy.Service("/get_dock_corners", dock_corners, callback)

    

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
