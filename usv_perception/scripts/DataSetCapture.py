#!/usr/bin/env python

'''
----------------------------------------------------------
    @file: DataSetCapture.py
    @date: Ago 2021
    @date_modif: Sun Ago 15, 2021
    @date_modif: Sun Ago 15, 2021
    @author: Raul David Dominguez Sanchez
    @e-mail: rauldavidds@hotmail.com
    @brief: Node to capture sinchronoulsy Point clouds 
    and Images
    @Note: Modify path,cntr and path2 accordingly
----------------------------------------------------------
'''

import rospy
import pypcd
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import numpy as np
import message_filters

bridge = CvBridge()
#counter of the dataset
cntr=124
#dataset path
path='/home/rauldds/catkin_ws/src/vttc/dataset/'
#path for labeling
path2='/home/rauldds/Desktop/labelCloud/pointclouds/' 

rospy.init_node("Dataset_Capture",anonymous=True)

def imgname(x):
    '''
        @name: imgname
        @brief: function to define the name of a image/point cloud file
        @param: x: current counter value
        @return: string with the name of the image/point cloud file
    '''

    return{
        '1': '00000'+str(cntr),
        '2': '0000'+str(cntr),
        '3': '000'+str(cntr),
        '4': '00'+str(cntr),
        '5': '0'+str(cntr),
        '6': str(cntr)
    }[x]

def CB(img, PC2):
    '''
        @name: CB
        @brief: function to store sinchronous point clouds and images
        in "bin" and "png" format files
        @param: img: Image message
                PC2: PointCLoud2 message
        @return: --
    '''

    global cntr
    cntr=cntr+1
    print(cntr)
    name = imgname(str(len(str(cntr))))
    pc = pypcd.PointCloud.from_msg(PC2)
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    intensity = pc.pc_data['intensity']
    arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
    arr[::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity
    fp=path+'velodyne/'+name+'.bin'
    fp2=path2+name+'.bin'
    arr.astype('float32').tofile(fp)
    arr.astype('float32').tofile(fp2)
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a png 
        fp=path+'image_2/'+name+'.png'
        cv2.imwrite(fp, cv2_img)
        rospy.loginfo("success")

img_sub = message_filters.Subscriber('/r200/camera/color/image_raw', Image)
pc2_sub = message_filters.Subscriber('velodyne_points', PointCloud2)

ts = message_filters.TimeSynchronizer([img_sub, pc2_sub], 10)
print("hola")
ts.registerCallback(CB)
rospy.spin()
