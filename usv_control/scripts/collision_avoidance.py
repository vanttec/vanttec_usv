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
        self.safety_radius = 0.0 #meters
        self.offset = .55 #camera to ins offset
        self.avoid_angle = 0
        #self.psi_r = 0
        self.increase = 0 
        self.collision_flag = 0
        self.b = []

        self.r_max = 1 #rad/sec
        self.u_psi = 0
        self.u_r = 0
        self.teta = []
        self.vel_list = []

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
        teta_list_right = []
        teta_list_left = []
        collision_obs_list = []

        self.vel_list = []
        nearest_obs = []
        self.b = []
        self.teta = []

        obstacle_ppx = []
        obstacle_ppy = []

        vel_nedx,vel_nedy = self.body_to_ned(self.u,self.v,0,0)
        vel_ppx,vel_ppy =  self.ned_to_pp(ak,0,0,vel_nedx,vel_nedy)
        ppx,ppy = self.ned_to_pp(ak,x1,y1,self.ned_x,self.ned_y)

        obs_list = self.check_obstacles()
        for i in range(0,len(obs_list),3):

            sys.stdout.write(Color.CYAN)
            print("obstacle"+str(i/3))
            sys.stdout.write(Color.RESET)
            obs_ppx, obs_ppy = self.get_obstacle(i, ak, x1, y1, obs_list[i], obs_list[i+1])
            obstacle_ppx.append(obs_ppx)
            obstacle_ppy.append(obs_ppy)
            obstacle_radius = obs_list[i+2]
            total_radius = self.boat_radius + self.safety_radius + obstacle_radius
            collision, distance = self.get_collision(total_radius, ppx, ppy, obs_ppx, obs_ppy, vel_ppy, vel_ppx, (i/3))
            if collision:
                #u_obs = np.amin(u_obstacle)
                avoid_distance = self.calculate_avoid_distance( vel_ppx, vel_ppy, total_radius, (i/3))
                nearest_obs.append(avoid_distance - distance)
                print("avoid_distance: " + str(avoid_distance)) 
                print("distance: " + str(distance)) 
            else:
                nearest_obs.append(0)
                self.vel_list.append(0)
                self.teta.append(0)
                self.b.append(0)
                '''
                if distance <= avoid_distance and self.b > 0:
                    self.collision_flag = 1
                    if abs(ppy-obs_ppy) < 0.01:
                        teta_list_right.append(self.teta)
                    else:
                        self.dodge(vel_ppx,vel_ppy,ppx,ppy,obs_ppx,obs_ppy)
                        if self.teta > 0:
                            teta_list_left.append(self.teta)
                        else: 
                            self.teta = -self.teta
                            teta_list_right.append(self.teta)
                else:
                    rospy.loginfo("avoid_distance: " + str(avoid_distance)) 
        
        print("right_list: " +str(len(teta_list_right)))
        print("left_list: " +str(len(teta_list_left)))
        if len(teta_list_right) > 0 & len(teta_list_left) > 0:
            if len(teta_list_right) == len(teta_list_left):
                if np.amax(teta_list_left) < np.amax(teta_list_right):
                    self.bearing = np.amax(teta_list_left)
                    sys.stdout.write(Color.GREEN)
                    print("left +")
                    sys.stdout.write(Color.RESET)
                else:
                    self.bearing = -np.amax(teta_list_right)
                    sys.stdout.write(Color.RED)
                    print("right -")
                    sys.stdout.write(Color.RESET)
        elif len(teta_list_right) == 0 & len(teta_list_left) == 0:
            sys.stdout.write(Color.BLUE)
            print ('free')
            sys.stdout.write(Color.RESET)
        else:
            if len(teta_list_left) > 0:
                self.bearing = np.amax(teta_list_left)
                sys.stdout.write(Color.GREEN)
                print("left +")
                sys.stdout.write(Color.RESET)
            else:
                self.bearing = -np.amax(teta_list_right)
                sys.stdout.write(Color.RED)
                print("right -")
                sys.stdout.write(Color.RESET)
            '''
        print('nearest_obs max: ' + str(np.max(nearest_obs)))
        if np.max(nearest_obs)>0:
            index = nearest_obs.index(np.max(nearest_obs))
            sys.stdout.write(Color.BOLD)
            print('index: ' + str(index))
            sys.stdout.write(Color.RESET)
            if np.max(nearest_obs) > 0 and self.b[index] > 0:
                collision_obs_list.append(obs_ppx)
                collision_obs_list.append(obs_ppy)
                collision_obs_list.append(obstacle_radius)
                self.vel = np.min(self.vel_list)
                self.dodge(vel_ppx,vel_ppy,ppx,ppy,obstacle_ppy[index],obstacle_ppy[index], index)
            else:
                rospy.loginfo("nearest_obs: " + str(nearest_obs[index])) 
                sys.stdout.write(Color.BLUE)
                print ('free')
                sys.stdout.write(Color.RESET)
            #sys.stdout.write(Color.BOLD)
            #print("yaw: " + str(self.yaw))
            #print("bearing: " + str(self.bearing))
            #sys.stdout.write(Color.RESET)
        else:
            sys.stdout.write(Color.BLUE)
            print ('no obstacles')
            sys.stdout.write(Color.RESET)

    def check_obstacles(self):
        print("CHECK OBSTACLES")
        obs_list = []
        for i in range(0,len(self.obstacles),1):
            obs_list.append(self.obstacles[i]['X'])
            obs_list.append(self.obstacles[i]['Y'])
            obs_list.append(self.obstacles[i]['radius'])
        i = 0
        print("len_obs: " + str(len(obs_list)))
        '''
        for i in range(0,len(self.obstacles),1):
            obs_list.append(self.obstacles[i]['X'])
            obs_list.append(self.obstacles[i]['Y'])
            obs_list.append(self.obstacles[i]['radius'])
        '''
        while i <= (len(obs_list)-6):
            j = i + 3
            while j < len(obs_list):
                #print("i: " + str(i))
                #print("j: " + str(j))
                x = pow(obs_list[i]-obs_list[j],2)
                y = pow(obs_list[i+1]-obs_list[j+1],2)
                radius = obs_list[i+2] + obs_list[j+2]
                distance = pow(x+y,0.5)-radius
                if distance <= (self.boat_radius + self.safety_radius)*2:
                    x,y,radius = self.merge_obstacles(obs_list[i],obs_list[i+1],obs_list[i+2],obs_list[j], obs_list[j+1], obs_list[j+2])
                    del obs_list[j:j+3]
                    del obs_list[i:i+3]
                    obs_list.append(x)
                    obs_list.append(y)
                    obs_list.append(radius)
                    print("len_obs: " + str(len(obs_list)))
                    i = -3
                    j = len(obs_list)
                else:
                    j=j+3
                #print("i: " + str(i))
                #print("j: " + str(j))
            i = i+3
        return obs_list

    def merge_obstacles(self, x1, y1, r1, x2, y2, r2):
        # calculate centroid
        x = (x1+x2)/2
        y = (y1+y2)/2
        #calculte radius
        x1_radius = x1 + r1 - x
        x2_radius = x2 + r2 - x
        y1_radius = y1 + r1 - y
        y2_radius = y2 + r2 - y
        radius = max(x1_radius, x2_radius, y1_radius, y2_radius)
        print("Merged obstacle:" + str(radius))
        return(x,y,radius)

    def get_obstacle(self, i, ak, x1, y1, obsx,obsy):
        # NED obstacles
        if (self.obstacle_mode == 0):
            obs_ppx,obs_ppy =  self.ned_to_pp(ak,x1,y1,obsx,obsy)
        # Body obstacles
        if (self.obstacle_mode == 1):
            obs_nedx, obs_nedy = self.body_to_ned(obsx,obsy,self.ned_x,self.ned_y)
            obs_ppx,obs_ppy = self.ned_to_pp(ak,x1,y1,obs_nedx,obs_nedy)
        return(obs_ppx, obs_ppy)
    
    def get_collision(self, total_radius, ppx, ppy, obs_ppx, obs_ppy, vel_ppy, vel_ppx, i):
        collision = 0
        #print("Total Radius: " + str(total_radius))
        x_pow = pow(obs_ppx - ppx,2) 
        y_pow = pow(obs_ppy - ppy,2) 
        distance = pow((x_pow + y_pow),0.5)

        #u_obstacle_list.append(u_obstacle)

        distance_free = distance - total_radius
        print("Distance_free: " + str(distance_free))

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
        if beta <= alpha or 1 == self.collision_flag:
            collision = 1
            self.calculate_avoid_angle(total_radius, ppy, obs_ppy, distance, ppx, obs_ppx, i)
            self.get_velocity(distance_free, i)
        return collision, distance
    
    def calculate_avoid_angle(self, total_radius, ppy, obs_ppy, distance, ppx, obs_ppx, i):
        '''
        @name: calculate_avoid_angle
        @brief: Calculates angle needed to avoid obstacle
        @param: total_radius: total obstacle readius
                ppy: boat y coordiante in path reference frame 
                obs_ppy: osbtacle y coordiante in path reference frame
                distance: distance from center of boat to center of obstacle 
                ppx: boat x coordiante in path reference frame 
                obs_ppx: osbtacle x coordiante in path reference frame
        @return: --
        '''
        print("ppx: " + str(ppx) + " obsppx: " + str(obs_ppx))
        print("ppy: " + str(ppy) + " obsppy: " + str(obs_ppy))
        total_radius = total_radius +.3
        tangent_param = abs((distance - total_radius) * (distance + total_radius))
        print("distance: " + str(distance))
        tangent = pow(tangent_param, 0.5)
        #print("tangent: " + str(tangent))
        teta = math.atan2(total_radius,tangent)
        print("teta: " + str(teta))
        gamma1 = math.asin(abs(ppy-obs_ppy)/distance)
        #print("gamma1: " + str(gamma1))
        gamma = ((math.pi/2) - teta) + gamma1
        #print("gamma: " + str(gamma))
        alpha = (math.pi/2) - gamma
        #print("alpha: " + str(alpha))
        hb = abs(ppy-obs_ppy)/math.cos(alpha)
        print("hb: " + str(hb))
        self.b.append(total_radius - hb)
        print("i: " + str(i))
        print("b: " + str(self.b[i]))
        self.teta.append(math.atan2(self.b[i],tangent))
        print("teta: " + str(self.teta[i]))
        if self.b[i] <= 0:
            self.collision_flag = 0

    def get_velocity(self, distance_free, i):
        u_r_obs = 1/(1 + math.exp(-self.exp_gain*(distance_free*self.chi_r - self.exp_offset)))
        u_psi_obs = 1/(1 + math.exp(self.exp_gain*(np.max(self.teta[i])*self.chi_psi - self.exp_offset)))
        self.vel_list.append((self.u_max - self.u_min)*np.min([self.u_psi, self.u_r, u_r_obs, u_psi_obs]) + self.u_min)
        #self.vel = 0.3

    def calculate_avoid_distance(self, vel_ppx, vel_ppy, total_radius, i):
        '''
        @name: calculate_avoid_distance
        @brief: Calculates distance at wich it is necesary to leave path to avoid obstacle
        @param: vel_ppx: boat velocity x  in path reference frame 
                vel_ppy: boat velocity y  in path reference frame 
                total_radius: total obstacle readius
        @return: avoid_distance: returns distance at wich it is necesary to leave path to avoid obstacle
        '''
        time = (self.teta[i]/self.r_max) + 3
        #print("time: " + str(time))
        eucledian_vel = pow((pow(vel_ppx,2) + pow(vel_ppy,2)),0.5)
        #print("vel: " + str(eucledian_vel))
        #print("self.vel: " + str(self.vel))
        #avoid_distance = time * eucledian_vel + total_radius +.3
        avoid_distance = time * self.vel + total_radius +.3
        return (avoid_distance)
    
    def dodge(self, vel_ppx, vel_ppy , ppx, ppy, obs_ppx, obs_ppy, i):
        '''
        @name: dodge
        @brief: Calculates angle needed to avoid obstacle
        @param: vel_ppx: boat velocity x  in path reference frame 
                vel_ppy: boat velocity y  in path reference frame 
                ppx: boat x coordiante in path reference frame 
                ppy: boat y coordiante in path reference frame 
                obs_ppx: osbtacle x coordiante in path reference frame
                obs_ppy: osbtacle y coordiante in path reference frame
        @return: --
        '''
        self.collision_flag = 1
        if abs(ppy-obs_ppy) < 0.01:
            self.bearing = -self.teta[i]
            sys.stdout.write(Color.RED)
            print("right -")
            sys.stdout.write(Color.RESET)
        else:
            eucledian_vel = pow((pow(vel_ppx,2) + pow(vel_ppy,2)),0.5)
            eucledian_pos = pow((pow(obs_ppx - ppx,2) + pow(obs_ppy - ppy,2)),0.5)
            if eucledian_pos != 0 and eucledian_vel != 0:
                unit_vely = vel_ppy/eucledian_vel 
                unit_posy = (obs_ppy - ppy)/eucledian_pos
                #print("unit_vely " + str(unit_vely))
                #print("unit_posy: " + str(unit_posy))
                if unit_vely <= unit_posy:
                    self.bearing = -self.teta[i]
                    sys.stdout.write(Color.RED)
                    print("right -")
                    sys.stdout.write(Color.RESET)
                    '''
                    if (abs(self.avoid_angle) > (math.pi/2)):
                        self.avoid_angle = -math.pi/2
                    '''
                else:
                    self.bearing =  self.teta[i]
                    sys.stdout.write(Color.GREEN)
                    print("left +")
                    sys.stdout.write(Color.RESET)
                    '''
                    if (abs(self.avoid_angle) > (math.pi/3)):
                        self.avoid_angle = math.pi/2
                    '''
                '''
                if unit_vely <= unit_posy:
                    self.teta = -self.teta
                    
                else:
                    self.teta =  self.teta
                    
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
    rate = rospy.Rate(20) # 100hz
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