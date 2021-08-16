#!/usr/bin/env python
import os
from kitti_object import *
import cv2
import rospy
import pypcd
from sensor_msgs.msg import PointCloud2
import numpy as np
import matplotlib.pyplot as plt
from FP_IMP import test_from_rgb_detection, get_session_and_ops
from visualization_msgs.msg import Marker

sess, ops = get_session_and_ops(32, 1024)
mrkr_pub=rospy.Publisher("marker_publisher",Marker,queue_size=10)

def in_hull(p, hull):
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    return hull.find_simplex(p)>=0

def extract_pc_in_box3d(pc, box3d):
    ''' pc: (N,3), box3d: (8,3) '''
    box3d_roi_inds = in_hull(pc[:,0:3], box3d)
    return pc[box3d_roi_inds,:], box3d_roi_inds

def frustums_gen(pointcloud,arr_c3,arr_c4):
    dataset = kitti_object(os.path.join('/home/rauldds/Desktop/VantTec/frustum-pointnets', 'dataset/KITTI/object'))
    data_idx = 9
    pc_batch =[]
    oh_batch = []
    result=[]

    # Load data from dataset
    # cambiar objects por nodo jorge
    objects = dataset.get_label_objects(data_idx)
    #objects[0].print_object() 
    pc_velo = pointcloud 
    # crear script para obtener calibracion
    calib = dataset.get_calibration(data_idx)
    
    # Only display those points that fall into 2d box
    #print(' -------- LiDAR points in a frustum from a 2D box --------')
    xmin,ymin,xmax,ymax = \
        objects[0].xmin, objects[0].ymin, objects[0].xmax, objects[0].ymax
    boxfov_pc_velo = \
        get_lidar_in_image_fov(pc_velo, calib, xmin, ymin, xmax, ymax)
    #print(('2d box FOV point num: ', boxfov_pc_velo.shape))
    #podria ser acelerado si se usa otro metodo en vez de for
    for i in range(boxfov_pc_velo.shape[0]):
        result.append(np.where(arr_c3 == boxfov_pc_velo[i][0])[0][0]) #despues de analizar, se podria eliminar arrc3 y usar solo arrc4
    #arr_c3 = arr_c3[result]
    arr_c4 = arr_c4[result]
    print("hola: "+str(arr_c4.shape))
    indices = np.arange(0, len(arr_c4))
    pc_ori = arr_c4
    if len(pc_ori) > 1024:
            choice = np.random.choice(indices, size=1024, replace=True)
            point_cloud_ds = np.array(pc_ori[choice])
    elif len(pc_ori)==0:
        return None
    elif len(pc_ori)<1024:
        #se podria acelerar con otro metodo en vez de for
        for i in range(1024-len(pc_ori)):
            pc_ori = np.append(pc_ori, [arr_c4[np.random.randint(arr_c4.shape[0])]], axis=0)
        point_cloud_ds = pc_ori
    else:
            choice = np.random.choice(indices, size=1024, replace=False)
            point_cloud_ds = np.array(pc_ori[choice])
    pc_batch.extend([point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,
                 point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds,point_cloud_ds])
    pc_batch = np.asarray(pc_batch, dtype=np.float32)
    #print("batch shape:"+str(pc_batch.shape))
    oh = np.zeros((1, 3))
    oh[0][1]=1
    oh_batch.extend([oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh,oh])
    oh_batch = np.asarray(oh_batch, dtype=np.float32) # 32X3 --> 32 NUBES EN FORMATO ONE HOT
    oh_batch = np.reshape(oh_batch,(32,3))
    pred, center, dim = test_from_rgb_detection(sess,ops,pc_batch,oh_batch)
    return pred, center, dim

def MarkerPublish(pred,center, dim,msg):
    mrkr = Marker()
    mrkr.header.frame_id = msg.header.frame_id
    mrkr.id = 1
    mrkr.ns = "obstacle"
    mrkr.type = 1
    mrkr.action = 0
    mrkr.color.r = 255
    mrkr.color.g = 0
    mrkr.color.b = 0
    mrkr.color.a = 0.5
    mrkr.scale.x = dim[0]
    mrkr.scale.y = dim[1]
    mrkr.scale.z = dim[2]
    mrkr.pose.position.z = center[2]+0.198
    mrkr.pose.position.x = center[0]+0.27
    mrkr.pose.position.y = center[1]+0.4
    mrkr.pose.orientation.x = 0
    mrkr.pose.orientation.y = 0
    mrkr.pose.orientation.z = 0
    mrkr.pose.orientation.w = 1
    mrkr.lifetime = rospy.Duration(1)
    mrkr_pub.publish(mrkr)

def PC2CallBack(msg):
    pc = pypcd.PointCloud.from_msg(msg)
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    intensity = pc.pc_data['intensity']
    arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
    arr[::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity
    arr2 = arr.reshape((-1, 4))
    arr_comp_4 = arr2
    arr2 = arr2[:,0:3]
    arr_comp_3 = arr2
    #print(arr2.shape)
    #rospy.loginfo("Point Cloud Shape: "+str(arr2.shape))
    pred, center, dim = frustums_gen(arr2,arr_comp_3,arr_comp_4)
    MarkerPublish(pred,center,dim, msg)

rospy.init_node("bin_generator",anonymous=True)
rospy.Subscriber("velodyne_points",PointCloud2,PC2CallBack)
rospy.spin()