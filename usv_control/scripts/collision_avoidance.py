#!/usr/bin/env python

'''
----------------------------------------------------------
    @file: collision_avoidance.py
    @date: January 2019
    @date_modif: Dom April 12, 2020
    @author: Alejandro Gonzalez
    @e-mail: alexglzg97@gmail.com
    @co-author: Ivana Collado
    @e-mail: ivanacollado@gmail.com
    @brief: Implementation of line-of-sight (LOS) algorithm with collition 
    avoidance algorithm
    @version: 1.0
    @licence: Open source
----------------------------------------------------------
'''

import math
import os
import sys
import time

import numpy as np
import rospy
from std_msgs.msg import Float64,  Float32MultiArray, String
from geometry_msgs.msg import Pose2D, Vector3
from usv_perception.msg import obstacles_list

# Class definition for easy debugging
class Color():
    RED   = "\033[1;31m"  
    BLUE  = "\033[1;34m"
    CYAN  = "\033[1;36m"
    GREEN = "\033[0;32m"
    RESET = "\033[0;0m"
    BOLD    = "\033[;1m"
    REVERSE = "\033[;7m"

class LOS:
    def __init__(self):
        self.active = True

        self.desired_speed = 0
        self.desired_heading = 0
        self.distance = 0
        self.bearing = 0

        self.ned_x = 0
        self.ned_y = 0
        self.yaw = 0

        self.reference_latitude = 0
        self.reference_longitude = 0

        self.waypoint_array = []
        self.last_waypoint_array = []

        self.delta_max = 5
        self.delta_min = 0.5
        self.gamma = 0.5

        self.k = 1

        self.u_max = 1
        self.u_min = 0.3
        self.threshold_radius = 5
        self.chi_r = 1./self.threshold_radius
        self.chi_psi = 2/math.pi
        self.exp_gain = 10
        self.exp_offset = 0.5

        self.waypoint_path = Pose2D()
        self.ye = 0

        self.obstacles = []

        self.waypoint_mode = 0 # 0 for NED, 1 for GPS, 2 for body
        self.obstacle_mode = 1 # 0 for NED, 1 for Body

        self.boat_radius = .50 #meters
        self.safety_radius = .3 #meters
        self.offset = .55 #camera to ins offset
        self.avoid_angle = 0
        #self.psi_r = 0
        self.increase = 0 
        self.collision_flag = 0

        self.r_max = 1 #rad/sec
        self.u_psi = 0
        self.u_r = 0

        # ROS Subscribers
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)
        rospy.Subscriber("/vectornav/ins_2d/local_vel", Vector3, self.local_vel_callback)
        rospy.Subscriber("/vectornav/ins_2d/ins_ref", Pose2D, self.gpsref_callback)
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)
        rospy.Subscriber("/usv_perception/lidar_detector/obstacles",  obstacles_list, self.obstacles_callback)

        # ROS Publishers
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/usv_control/los/target", Pose2D, queue_size=10)
        self.ye_pub = rospy.Publisher("/usv_control/los/ye", Float64, queue_size=10)

    def ned_callback(self, ned):
        self.ned_x = ned.x
        self.ned_y = ned.y
        self.yaw = ned.theta
    
    def local_vel_callback(self, upsilon):
        self.u = upsilon.x
        self.v = upsilon.y
        self.r = upsilon.z

    def gpsref_callback(self, gps):
        self.reference_latitude = gps.x
        self.reference_longitude = gps.y

    def waypoints_callback(self, msg):
        waypoints = []
        leng = (msg.layout.data_offset)
        for i in range(int(leng)-1):
            waypoints.append(msg.data[i])
        self.waypoint_mode = msg.data[-1] # 0 for NED, 1 for GPS, 2 for body
        self.waypoint_array = waypoints

    def obstacles_callback(self, data):
        self.obstacles = []
        for i in range(data.len):
            self.obstacles.append({'X' : data.obstacles[i].x, #+ self.offset,
                                   'Y' : data.obstacles[i].y,
                                   'radius' : data.obstacles[i].z})

    def los_manager(self, listvar):
        '''
        @name: los_manager
        @brief: Waypoint manager to execute the LOS algorithm.
        @param: listvar: list of variable waypoints
        @return: --
        '''
        if self.k < len(listvar)/2:
            x1 = listvar[2*self.k - 2]
            y1 = listvar[2*self.k - 1]
            x2 = listvar[2*self.k]
            y2 = listvar[2*self.k + 1]
            self.waypoint_path.x = x2
            self.waypoint_path.y = y2
            self.target_pub.publish(self.waypoint_path)
            x_squared = math.pow(x2 - self.ned_x, 2)
            y_squared = math.pow(y2 - self.ned_y, 2)
            self.distance = math.pow(x_squared + y_squared, 0.5)

            if self.distance > 1:
                self.los(x1, y1, x2, y2)
            else:
                self.k += 1
        else:
            self.desired(0, self.yaw)

    def los(self, x1, y1, x2, y2):
        '''
        @name: los
        @brief: Implementation of the LOS algorithm.
        @param: x1: x coordinate of the path starting-waypoint
                y1: y coordinate of the path starting-waypoint
                x2: x coordinate of the path ending-waypoint
                y2: y coordinate of the path ending-waypoint
        @return: --
        '''
        ak = math.atan2(y2 - y1, x2 - x1)
        ye = -(self.ned_x - x1)*math.sin(ak) + (self.ned_y - y1)*math.cos(ak)
        xe = (self.ned_x - x1)*math.cos(ak) + (self.ned_y - y1)*math.sin(ak)
        x_total = (x2 - x1)*math.cos(ak) + (y2 - y1)*math.sin(ak)
        if xe > x_total: #Means the USV went farther than x2. 2 Alternatives:
            #This one makes the USV return
            ak = ak - math.pi
            if (abs(ak) > (math.pi)):
                ak = (ak/abs(ak))*(abs(ak) - 2*math.pi)
            ye = -(self.ned_x - x1)*math.sin(ak) + (self.ned_y - y1)*math.cos(ak)
            xe = (self.ned_x - x1)*math.cos(ak) + (self.ned_y - y1)*math.sin(ak)
            '''#This one changes the target to the next in line
            self.k += 1'''

        delta = (self.delta_max - self.delta_min)*math.exp(-(1/self.gamma)*abs(ye)) + self.delta_min
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r

        if (abs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing) - 2*math.pi)

        x_los = x1 + (delta+xe)*math.cos(ak)
        y_los = y1 + (delta+xe)*math.sin(ak)
        self.ye = ye
        self.ye_pub.publish(self.ye)

        e_psi = self.bearing - self.yaw
        abs_e_psi = abs(e_psi)
        if (abs_e_psi > (math.pi)):
            e_psi = (e_psi/abs_e_psi)*(abs_e_psi - 2*math.pi)
            abs_e_psi = abs(e_psi)
        self.u_psi = 1/(1 + math.exp(self.exp_gain*(abs_e_psi*self.chi_psi - self.exp_offset)))
        self.u_r = 1/(1 + math.exp(-self.exp_gain*(self.distance*self.chi_r - self.exp_offset)))

        self.vel = (self.u_max - self.u_min)*np.min([self.u_psi, self.u_r]) + self.u_min
        
        self.avoid(ak, x1, y1)

        if (abs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing) - 2*math.pi)

        self.desired(self.vel, self.bearing)

    def avoid(self, ak, x1, y1):
        '''
        @name: avoid
        @brief: Calculates if there is an impending collision.
        @param: x1: x coordinate of the path starting-waypoint
                y1: y coordinate of the path starting-waypoint
                ak: angle from NED reference frame to path
        @return: --
        '''
        u_obstacle_list = []
        vel_nedx,vel_nedy = self.body_to_ned(self.u,self.v,0,0)
        vel_ppx,vel_ppy =  self.ned_to_pp(ak,0,0,vel_nedx,vel_nedy)
        ppx,ppy = self.ned_to_pp(ak,x1,y1,self.ned_x,self.ned_y)
        crash = 0
        for i in range(0,len(self.obstacles),1):
            sys.stdout.write(Color.CYAN)
            print("obstacle"+str(i+1))
            sys.stdout.write(Color.RESET)
            obsx = self.obstacles[i]['X']
            obsy = self.obstacles[i]['Y']
            self.increase = (self.obstacles[i]['radius'])
            # NED obstacles
            if (self.obstacle_mode == 0):
                obs_ppx,obs_ppy =  self.ned_to_pp(ak,x1,y1,obsx,obsy)
            # Body obstacles
            if (self.obstacle_mode == 1):
                obs_nedx, obs_nedy = self.body_to_ned(obsx,obsy,self.ned_x,self.ned_y)
                obs_ppx,obs_ppy = self.ned_to_pp(ak,x1,y1,obs_nedx,obs_nedy)
            obstacle_radius = self.obstacles[i]['radius']
            total_radius = self.boat_radius + self.safety_radius + obstacle_radius
            x_pow = pow(obs_ppx - ppx,2) 
            y_pow = pow(obs_ppy - ppy,2) 
            distance = pow((x_pow + y_pow),0.5)
            print("Total Radius: " + str(total_radius))
    
            u_obstacle = 1/(1 + math.exp(-self.exp_gain*(distance*self.chi_r - self.exp_offset)))
            u_obstacle_list.append(u_obstacle)

            distance_free = distance - total_radius
            print("Distance_free: " + str(distance_free))
            '''
            #Almost crash prevention
            if abs(distance_free) < self.increase/10:
                rospy.logwarn("ALMOST CRASH")
                if distance_free <= 0: 
                    self.increase = -math.pi/2
                if distance_free > 0: 
                    self.increase = math.pi/2
                self.bearing = self.yaw + self.increase
                if (abs(self.bearing) > (math.pi)):
                    print("BEARING: " + str(self.bearing))
                    self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing)-2*math.pi)
                    #print("BEARING OVERFLOW: " + str(self.bearing))
                self.desired(self.vel, self.bearing)
            '''
            if distance < total_radius:
                rospy.logwarn("CRASH")
            alpha_params = (total_radius/distance)
            alpha = math.asin(alpha_params)
            beta = math.atan2(vel_ppy,vel_ppx)-math.atan2(obs_ppy-ppy,obs_ppx-ppx)
            if beta > math.pi: 
                beta = beta - 2*math.pi
            if beta < -math.pi: 
                beta = beta + 2*math.pi
            beta = abs(beta)
            if beta < alpha or beta == alpha or 1 == self.collision_flag:
                u_obs = np.amin(u_obstacle)
                self.vel = (self.u_max - self.u_min)*np.min([self.u_psi, self.u_r, u_obs]) + self.u_min
                
                self.calculate_avoid_angle(total_radius, ppy, obs_ppy, distance, ppx, obs_ppx)
                avoid_distance = self.calculate_avoid_distance( vel_ppx, vel_ppy, total_radius)
                if distance <= avoid_distance:
                    #self.collision_flag = 1
                    self.vel = 0.3
                    self.dodge(vel_ppx,vel_ppy,ppx,ppy,obs_ppx,obs_ppy)
                
                '''
                self.bearing =  ak + self.psi_r + self.avoid_angle
                if (abs(self.bearing) > (math.pi)):
                    print("BEARING: " + str(self.bearing))
                    self.bearing = (self.bearing/abs(self.bearing))*(abs(self.bearing)-2*math.pi)
                    print("BEARING OVERFLOW: " + str(self.bearing))
                #print("ak: " +str(ak))
                '''
                crash = crash + 1
        if crash == 0:
            sys.stdout.write(Color.BLUE)
            print ('free')
            sys.stdout.write(Color.RESET)
            '''
            #Gradual comeback
            if(self.avoid_angle > self.increase or self.avoid_angle < -self.increase):
                if(self.avoid_angle < -self.increase):
                    self.avoid_angle = self.avoid_angle + self.increase
                if (self.avoid_angle > self.increase ):
                    self.avoid_angle = self.avoid_angle -self.increase
            else:
                self.avoid_angle = 0
            #self.avoid_angle = 0
            '''
        sys.stdout.write(Color.BOLD)
        print("yaw: " + str(self.yaw))
        print("bearing: " + str(self.bearing))
        #print("avoid_angle: " + str(self.avoid_angle))
        sys.stdout.write(Color.RESET)
        
    def calculate_avoid_angle(self, total_radius, ppy, obs_ppy, distance, ppx, obs_ppx):
        print("ppy: " + str(ppy) + " obsppy: " + str(obs_ppy))
        total_radius = total_radius +.3
        b = total_radius #- abs(abs(obs_ppy)-abs(ppy))
        print("b: " + str(b))
        tangent_param = abs((distance - total_radius) * (distance + total_radius))
        print("distance: " + str(distance))
        tangent = pow(tangent_param, 0.5)
        print("tangent: " + str(tangent))
        if tangent >= b:
            self.teta = math.asin(b/tangent)
        else:
            self.teta = 90-math.asin(tangent/b)
            #if obs_ppx <= ppx:
                #self.collision_flag = 0
            #if self.teta <= 0:
            #self.collision_flag = 0

        print("teta: " + str(self.teta))

    def calculate_avoid_distance(self, vel_ppx, vel_ppy, total_radius):
        time = (self.teta/self.r_max)+2
        print("time: " + str(time))
        eucledian_vel = pow((pow(vel_ppx,2) + pow(vel_ppy,2)),0.5)
        print("vel: " + str(eucledian_vel))
        avoid_distance = time * eucledian_vel + total_radius +.3
        print("avoid_distance: " + str(avoid_distance)) 
        return (avoid_distance)
    
    def dodge(self, vel_ppx, vel_ppy , ppx, ppy, obs_ppx, obs_ppy):
        eucledian_vel = pow((pow(vel_ppx,2) + pow(vel_ppy,2)),0.5)
        eucledian_pos = pow((pow(obs_ppx - ppx,2) + pow(obs_ppy - ppy,2)),0.5)
        if eucledian_pos != 0 and eucledian_vel != 0:
            rospy.loginfo('collision')
            unit_vely = vel_ppy/eucledian_vel 
            unit_posy = (obs_ppy - ppy)/eucledian_pos #+ 0.1
            print("unit_vely " + str(unit_vely))
            print("unit_posy: " + str(unit_posy))
            if unit_vely <= unit_posy:
                #self.bearing = self.yaw - self.increase
                self.bearing = self.yaw - self.teta
                sys.stdout.write(Color.RED)
                print("left -")
                sys.stdout.write(Color.RESET)
                '''
                if (abs(self.avoid_angle) > (math.pi/2)):
                    self.avoid_angle = -math.pi/2
                '''
            else:
                self.bearing = self.yaw + self.teta
                #self.bearing = self.yaw + self.increase
                sys.stdout.write(Color.GREEN)
                print("right +")
                sys.stdout.write(Color.RESET)
                '''
                if (abs(self.avoid_angle) > (math.pi/3)):
                    self.avoid_angle = math.pi/2
                '''

    def gps_to_ned(self, latitude_2, longitude_2):
        '''
        @name: gps_to_ned
        @brief: Coordinate transformation between geodetic and NED reference frames.
        @param: latitude_2: target x coordinate in geodetic reference frame
                longitude_2: target y coordinate in geodetic reference frame
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        latitude_1 = self.reference_latitude
        longitude_1 = self.reference_longitude

        longitud_distance = (longitude_1 - longitude_2)
        y_distance = math.sin(longitud_distance)*math.cos(latitude_2)
        x_distance = (math.cos(latitude_1)*math.sin(latitude_2) 
                     - math.sin(latitude_1)*math.cos(latitude_2)*math.cos(longitud_distance))
        bearing = math.atan2(-y_distance, x_distance)
        phi_1 = math.radians(latitude_1)
        phi_2 = math.radians(latitude_2)
        delta_phi = math.radians(latitude_2 - latitude_1)
        delta_longitude = math.radians(longitude_2 - longitude_1)
        a = (math.sin(delta_phi/2)*math.sin(delta_phi/2) 
             + math.cos(phi_1)*math.cos(phi_2)*math.sin(delta_longitude/2)*math.sin(delta_longitude/2))
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6378137*c

        ned_x2 = distance*math.cos(bearing)
        ned_y2 = distance*math.sin(bearing)

        return (ned_x2,ned_y2)


    def body_to_ned(self, x2, y2, offsetx, offsety):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: x2: target x coordinate in body reference frame
                y2: target y coordinate in body reference frame
                offsetx: offset x in ned reference frame
                offsety: offset y in ned reference frame
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        p = np.array([x2, y2])
        J = np.array([[math.cos(self.yaw), -1*math.sin(self.yaw)],
                      [math.sin(self.yaw), math.cos(self.yaw)]])
        n = J.dot(p)
        ned_x2 = n[0] + offsetx
        ned_y2 = n[1] + offsety
        return (ned_x2, ned_y2)

    def ned_to_pp(self, ak, ned_x1, ned_y1, ned_x2, ned_y2):
        '''
        @name: ned_to_ned
        @brief: Coordinate transformation between NED and body reference frames.
        @param: ak: 
                ned_x1: origin of parallel path x coordinate in ned reference frame
                ned_y1: origin of parallel path y coordinate in ned reference frame
                ned_x2: target x coordinate in ned reference frame
                ned_y2: target y coordinate in ned reference frame
        @return: pp_x2: target x coordinate in parallel path reference frame
                 pp_y2: target y coordinate in parallel path reference frame
        '''
        n = np.array([ned_x2 - ned_x1, ned_y2 - ned_y1])
        J = np.array([[math.cos(ak), -1*math.sin(ak)],
                      [math.sin(ak), math.cos(ak)]])
        J = np.linalg.inv(J)
        pp = J.dot(n)
        pp_x2 = pp[0]
        pp_y2 = pp[1]
        return (pp_x2, pp_y2)

    def desired(self, _speed, _heading):
        self.desired_heading = _heading
        self.desired_speed = _speed
        self.d_heading_pub.publish(self.desired_heading)
        self.d_speed_pub.publish(self.desired_speed)

def main():
    rospy.init_node('collision_avoidance', anonymous=False)
    rate = rospy.Rate(10) # 100hz
    los = LOS()
    los.last_waypoint_array = []
    aux_waypoint_array = []

    while (not rospy.is_shutdown()) and los.active:
        if los.last_waypoint_array != los.waypoint_array:
            los.k = 1
            los.last_waypoint_array = los.waypoint_array
            aux_waypoint_array = los.last_waypoint_array
            x_0 = los.ned_x
            y_0 = los.ned_y
            
            if los.waypoint_mode == 0:
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif los.waypoint_mode == 1:
                for i in range(0, len(aux_waypoint_array), 2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = los.gps_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
            elif los.waypoint_mode == 2:
                for i in range(0, len(aux_waypoint_array), 2):
                    aux_waypoint_array[i], aux_waypoint_array[i+1] = los.body_to_ned(aux_waypoint_array[i],aux_waypoint_array[i+1])
                aux_waypoint_array.insert(0,x_0)
                aux_waypoint_array.insert(1,y_0)
        if len(aux_waypoint_array) > 1:
            los.los_manager(los.last_waypoint_array)
        rate.sleep()
    los.desired(0, los.yaw)
    rospy.logwarn('Finished')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass