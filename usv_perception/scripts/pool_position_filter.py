#!/usr/bin/env python

#The code on top tells the compiler to use the main python

from unittest import case
import rospy
import math 
from math import radians, cos, sin, asin, sqrt






#from sensor_msgs.msg import Image
import numpy as np
from usv_perception.msg import obj_detected
from usv_perception.msg import obj_detected_list

from geometry_msgs.msg import Pose2D #Message to function Pose2D
from geometry_msgs.msg import Vector3 #Message to function Pose2D
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



class pool_filter_node:
    def __init__(self):

        #Initial declarations of variables and objects that'll be used in the node.
        rospy.Subscriber("/usv_perception/lidar/objects_detected", obj_detected_list, self.objs_callback)
        #rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.callback_yolo_det)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.callback_NED_pose)
        rospy.Subscriber("/vectornav/ins_2d/ins_pose", Pose2D, self.callback_ins_pose)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Pose2D, self.callback_ins_ref)

        self.objects_detected_lidar = obj_detected_list()
        #self.obj_detected_yolo = obj_detected_list()
        self.obj_trackId = {}
        self.NED_pose = Pose2D()
        self.ins_pose = Pose2D()
        self.ins_reference = Pose2D()
        self.objects_to_filter = obj_detected_list()
        
        self.limit_markers = MarkerArray()
        self.filter_markers = MarkerArray()
        self.pool_filtered_objects = obj_detected_list()
        self.pool_filter_node_pub = rospy.Publisher('/usv_perception/pool_filtered_objects', obj_detected_list, queue_size=10)
        self.limit_markers_pub = rospy.Publisher('/usv_perception/limit_markers', MarkerArray, queue_size=10)
        self.filter_markers_pub = rospy.Publisher('/usv_perception/filter_markers', MarkerArray, queue_size=10)
        self.Sides = np.array([0,0,0,0])
        self.XDistances = np.array([0,0,0,0])
        self.limits = np.array([0,0,0,0])
        self.storedRef = False
    def objs_callback(self,data):
        self.objects_detected_lidar = data
        #rospy.loginfo("Detecting currently: " + str(len(data.objects)) + " objects")

        
    def callback_NED_pose(self,data):
        self.NED_pose = data

    def callback_ins_pose(self,data):
        self.ins_pose = data

    def callback_ins_ref(self,data):
        self.ins_reference = data
        rospy.loginfo("Logged info from ins_ref")
        self.storedRef = True
    ## First function to change the object_registered list coordinates from BOAT frame to GPS frame - VN

    def rotation_matrix(self):
        '''
        @name: rotation_matrix
        @brief: Transformation matrix template.
        @param: angle: angle of rotation
        @return: J: transformation matrix
        '''
        J = np.array([[math.cos(self.NED_pose.theta), -1*math.sin(self.NED_pose.theta)],
                      [math.sin(self.NED_pose.theta), math.cos(self.NED_pose.theta)]])
        return (J)

    def coordBodyToNED(self):
        #Obstacle coordinates to NED
        rospy.loginfo("Transforming from body to NED")
        self.objects_to_filter = self.objects_detected_lidar
        for i in self.objects_to_filter.objects:      
            #rospy.loginfo("Body coords are " + str(i.X) + " X and " + str(i.Y) + " Y")
            p = np.array([i.X, -i.Y])
            J = self.rotation_matrix()
            n = J.dot(p)
            i.X = n[0] + self.NED_pose.x #Offset en X por el LIDAR
            i.Y = n[1] + self.NED_pose.y
            rospy.loginfo("NED coords are " + str(i.X) + " X and " + str(i.Y) + " Y")
    

    #Haversine formula function, used to calculate distance in meters between two ins coordinates
                    #lat = y, longitude = x 
    def haversine(self, lat1, lon1, lat2, lon2):

      R = 6372800 # this is in miles.  For Earth radius in kilometers use 6372.8 km
      dLat = radians(lat2 - lat1)
      dLon = radians(lon2 - lon1)
      lat1 = radians(lat1)
      lat2 = radians(lat2)
      a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
      c = 2*asin(sqrt(a))

      return R * c

    #Function to calculate length of each pool side
    def calculatePoolLengths(self, firstCorner, secondCorner, Side):
        distance = self.haversine(firstCorner[0], firstCorner[1], secondCorner[0], secondCorner[1])
        self.Sides[Side] = distance

    #Function to calculate distance from each corner to ins_reference (NED CENTER)
    def calculateDistancesCornerToReference(self, corner, side):
            self.XDistances[side] = self.haversine(corner[0], corner[1], self.ins_reference.x, self.ins_reference.y)
            rospy.loginfo("Using: " + str(corner[0]) + " " + str(corner[1]) + " " + str(self.ins_reference.y) + " " + str(self.ins_reference.x))
            rospy.loginfo("Giving: " + str(self.XDistances[side]))

    #Function to calculate distances from ins_reference to a pool side
    def heronHeightFormula(self, a, b, c):
        s = (a+b+c)/2
        area = sqrt((s*(s-a)*(s-b)*(s-c)))
        height = (2*area)/a
        return height

    #Function to calculate NED limits in each direction
    def calculateNEDLimits(self):
        for i in range(4):
                if (i != 3):
                    rospy.loginfo("Trying to obtain height of " + str(i) + " a:" + str(self.Sides[i]) + " b:" + str(self.XDistances[i]) + " c:" +  str(self.XDistances[i+1]))
                    result = self.heronHeightFormula(self.Sides[i], self.XDistances[i], self.XDistances[i+1])
                    rospy.loginfo("Height:" + str(result))
                elif (i == 3):
                    rospy.loginfo("Trying to obtain height of " + str(i) + " a:" + str(self.Sides[i]) + " b:" + str(self.XDistances[i]) + " c:" +  str(self.XDistances[i-3]))          
                    result = self.heronHeightFormula(self.Sides[i], self.XDistances[i], self.XDistances[i-3])
                    rospy.loginfo("Height:" + str(result))          
                self.limits[i] = result
            
    def filterCoordinates(self):

        temp = obj_detected()
        tempArr = obj_detected_list()
        
        rospy.loginfo("Limits are x:" + str(self.limits[0]) + " -y:" + str(self.limits[1]) + " -x:" + str(self.limits[2]) + " y:" + str(self.limits[3]))

        for detectedObject in self.objects_to_filter.objects:
            
            withinX = False
            withinY = False
            if(detectedObject.X < self.limits[1] and detectedObject.X > -self.limits[3]):
                withinX = True
            if(detectedObject.Y < self.limits[0] and detectedObject.Y > -self.limits[2]):
                withinY = True
            if (withinX & withinY):
                temp = detectedObject
                tempArr.objects.append(temp)
        
        self.pool_filtered_objects = tempArr
        
    def makeMarkers(self):
        
        tempMarkerArray = MarkerArray()

        for i in range(len(self.limits) + 1):

            x = 0
            y = 0
            
            tempMarker = Marker()
            
            if (i == 0):
                x = 0
                y = self.limits[i]
            elif (i == 1):
                x = self.limits[i]
                y = 0
            elif (i == 2):
                x = 0
                y = -self.limits[i]
            elif (i == 3):
                x = - self.limits[i] + 1
                y = 0
            elif (i == 4):
                x = 0
                y = 0
            
            #This sections transforms the NED coordinates to body to visualize in RViz
            rospy.loginfo("MarkNED coords " + str(x) + " X and " + str(y) + " Y")
            n = np.array([x - self.NED_pose.x, y - self.NED_pose.y])
            J = self.rotation_matrix()
            J = np.linalg.inv(J)
            b = J.dot(n)
            x = b[0]
            y = b[1]
            rospy.loginfo("MarkBody coords are " + str(x) + " X and " + str(y) + " Y")

            tempMarker.header.frame_id = "/velodyne"
            tempMarker.header.stamp = rospy.Time()
            tempMarker.type = tempMarker.SPHERE
            tempMarker.id = i
            tempMarker.pose.position.z = 0
            tempMarker.pose.position.x = x
            tempMarker.pose.position.y = -y
            tempMarker.color.a = 1
            if (i == 4):
                tempMarker.color.r = 0
                tempMarker.color.g = 0
                tempMarker.color.b = 1
            else:
                tempMarker.color.r = 0
                tempMarker.color.g = 1
                tempMarker.color.b = 0
            tempMarker.scale.x = 0.3
            tempMarker.scale.y = 0.3
            tempMarker.scale.z = 0.3

            tempMarkerArray.markers.append(tempMarker)
                
        self.limit_markers = tempMarkerArray

    def makeFilterMarkers(self):
        
        tempMarkerArray = MarkerArray()

        for filtered_object in self.pool_filtered_objects.objects:


        
            tempMarker = Marker()
            tempObjReg = obj_detected()
            
            
            tempObjReg.X = filtered_object.X
            tempObjReg.Y = filtered_object.Y
            tempObjReg.id = filtered_object.id
            tempObjReg.color = filtered_object.color

            #This sections transforms the NED coordinates to body to visualize in RViz
            rospy.loginfo("MarkNED coords " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")
            n = np.array([tempObjReg.X - self.NED_pose.x, tempObjReg.Y - self.NED_pose.y])
            J = self.rotation_matrix()
            J = np.linalg.inv(J)
            b = J.dot(n)
            tempObjReg.X = b[0]
            tempObjReg.Y = b[1]
            rospy.loginfo("MarkBody coords are " + str(tempObjReg.X) + " X and " + str(tempObjReg.Y) + " Y")

            tempMarker.header.frame_id = "velodyne"
            tempMarker.header.stamp = rospy.Time()
            tempMarker.type = tempMarker.SPHERE
            tempMarker.id = tempObjReg.id
            tempMarker.pose.position.z = 0
            tempMarker.pose.position.x = tempObjReg.X
            tempMarker.pose.position.y = -tempObjReg.Y
            tempMarker.color.a = 1
            tempMarker.color.r = 1
            tempMarker.color.g = 0
            tempMarker.color.b = 0
            tempMarker.scale.x = 0.3
            tempMarker.scale.y = 0.3
            tempMarker.scale.z = 0.3

            tempMarkerArray.markers.append(tempMarker)
                
        self.filter_markers = tempMarkerArray


def main():
                #First coordinate = Latitude, second = longitude
    cornerSE = np.array([25.6532937133, -100.291121541])
    cornerNE = np.array([25.653379321, -100.291116625])
    cornerNW = np.array([25.6533818532, -100.2916017181])
    cornerSW = np.array([25.65329823, -100.29156108])

    rospy.init_node("pool_filtering_node", anonymous=False)
    rate = rospy.Rate(20)
    poolFilterNode = pool_filter_node()
    poolFilterNode.calculatePoolLengths(cornerSE, cornerNE, 0)
    poolFilterNode.calculatePoolLengths(cornerNE, cornerNW, 1)
    poolFilterNode.calculatePoolLengths(cornerNW, cornerSW, 2)
    poolFilterNode.calculatePoolLengths(cornerSW, cornerSE, 3)
    
    while not rospy.is_shutdown() :
        if (poolFilterNode.storedRef):
            poolFilterNode.coordBodyToNED()
            poolFilterNode.calculateDistancesCornerToReference(cornerSE, 0)
            poolFilterNode.calculateDistancesCornerToReference(cornerNE, 1)
            poolFilterNode.calculateDistancesCornerToReference(cornerNW, 2)
            poolFilterNode.calculateDistancesCornerToReference(cornerSW, 3)
            poolFilterNode.calculateNEDLimits()
            poolFilterNode.filterCoordinates()
            poolFilterNode.pool_filter_node_pub.publish(poolFilterNode.pool_filtered_objects)
            poolFilterNode.makeMarkers()
            poolFilterNode.makeFilterMarkers()
            poolFilterNode.limit_markers_pub.publish(poolFilterNode.limit_markers)
            poolFilterNode.filter_markers_pub.publish(poolFilterNode.filter_markers)
            rospy.loginfo("Published pool-filtered obstacles")
            rate.sleep()
    rospy.spin()
    



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass