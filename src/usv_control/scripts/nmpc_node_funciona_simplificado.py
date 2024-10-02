#!/usr/bin/env python3
import os
import csv
import math 
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3, Pose2D
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from usv_interfaces.msg import WaypointList

from rockit import *
from casadi import *

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

nx    = 3                   # the system is composed of 9 states
nu    = 2                   # the system has 2 inputs
Tf    = 1.5                   # control horizon [s]
Nhor  = 150                  # number of control intervals
dt    = Tf/Nhor             # sample time

u_ref = 0.5


class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')
        self.x = 0.
        self.y = 0.
        self.psi = 0.
        self.u = 0.
        self.r = 0.

        self.target = np.array([[0., 0.], [10., 5.]])


        self.vel_sub = self.create_subscription(
            Vector3, '/usv/state/velocity', self.vel_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Pose2D, '/usv/state/pose', self.pose_callback, 10
        )
        self.left_thruster_sub = self.create_subscription(
            Float64, '/usv/left_thruster', self.lt_callback, 10
        )
        self.right_thruster_sub = self.create_subscription(
            Float64, '/usv/right_thruster', self.rt_callback, 10
        )
        # self.goals_sub = self.create_subscription(
        #     WaypointList, '/usv/goals', self.goals_callback, 10
        # )
        

        self.u_d_pub_ = self.create_publisher(Float64, "/guidance/desired_velocity", 10)
        self.r_d_pub_ = self.create_publisher(Float64, "/guidance/desired_angular_velocity", 10)

        self.u_d_msg = Float64()
        self.r_d_msg = Float64()


        self.ocp = Ocp(T=Tf)
        # Define states
        self.nedx_state = self.ocp.state()
        self.nedy_state = self.ocp.state()
        self.psi_state = self.ocp.state()
        # Defince controls
        self.u_control = self.ocp.control()
        self.r_control = self.ocp.control()
        
        # Define parameter
        self.X_0 = self.ocp.parameter(nx)

        # Specify ODE
        self.x_d = self.target[1][0]
        self.y_d = self.target[1][1]
        self.gamma_p = atan2( self.target[1][1] - self.target[0][1] , self.target[1][0] - self.target[0][0] )
        self.ye = -(self.nedx_state-self.x_d)*sin(self.gamma_p)+(self.nedy_state-self.y_d)*cos(self.gamma_p)
        self.xe = (self.nedx_state - self.x_d)*cos(self.gamma_p)+(self.nedy_state-self.y_d)*sin(self.gamma_p)
        self.ocp.set_der(self.nedx_state, self.u_control*cos(self.psi_state))
        self.ocp.set_der(self.nedy_state, self.u_control*sin(self.psi_state))
        self.ocp.set_der(self.psi_state, self.r_control)

        Q_xe = 3.0
        Q_ye = 30.0
        Q_psi = 10.0
        Q_u = 25.0

        # Lagrange objective
        self.ocp.add_objective(self.ocp.sum(Q_ye*(self.ye**2) + Q_psi*(sin(self.psi_state)-sin(self.gamma_p))**2 + Q_psi*(cos(self.psi_state)-cos(self.gamma_p))**2 + Q_xe*(self.xe**2) + Q_u*(self.u_control**2+self.r_control**2)))
        self.ocp.add_objective(self.ocp.at_tf(Q_ye*(self.ye**2) + Q_xe*(self.xe**2) + Q_u*(self.u_control**2+self.r_control**2)))
        self.ocp.add_objective(self.ocp.T)

        # Path constraints
        self.ocp.subject_to( (-0.5 <= self.u_control) <= 1.0 )
        self.ocp.subject_to( (-0.5 <= self.r_control) <= 0.5 )
        # ocp.subject_to( (0 <= d_s) <= d_s_max )   

        # for i,j,k in obs_regs:
        #     self.ocp.subject_to(((i-self.nedx_state)**2 + (j-self.nedy_state)**2) >= k)
        # ocp.subject_to(sqrt(pow(6 - self.nedx_state,2) + pow(3 - self.nedy_state,2)) >= k)

        # Initial constraints
        X = vertcat(self.nedx_state,self.nedy_state,self.psi_state)
        self.ocp.subject_to(self.ocp.at_t0(X)==self.X_0)

        # Pick a solution method
        options = {"ipopt": {"print_level": 0}}
        options["expand"] = True
        options["print_time"] = True
        self.ocp.solver('ipopt',options)
        self.ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))
        self.Sim_asv_dyn = self.ocp._method.discrete_system(self.ocp)

        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        self.ocp.set_initial(self.nedx_state,self.x)
        self.ocp.set_initial(self.nedy_state,self.y)
        self.ocp.set_initial(self.psi_state,self.psi)
        self.ocp.set_initial(self.u_control,self.u)
        self.ocp.set_initial(self.r_control,self.r)

        current_X = vertcat(self.x,self.y,self.psi)
        print(current_X)
        current_X = self.Sim_asv_dyn(x0=current_X, u=vertcat(self.u, self.r), T=dt)["xf"]

        self.ocp.set_value(self.X_0, vertcat(current_X))
        sol = self.ocp.solve()
        self.ocp._method.opti.set_initial(self.ocp._method.opti.x, self.ocp._method.opti.value(self.ocp._method.opti.x))

        # self.u_d_msg.data = (float)(current_X[3].full())
        _, u_sol = sol.sample(self.u_control, grid='control')
        _, r_sol = sol.sample(self.r_control, grid='control')
        self.u_d_msg.data = (float)(u_sol[0])
        self.r_d_msg.data = (float)(r_sol[0])
        self.u_d_pub_.publish(self.u_d_msg)
        self.r_d_pub_.publish(self.r_d_msg)

    def vel_callback(self, msg):
        self.u = max(msg.x,0.01)
        self.r = msg.z
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.psi = msg.theta
    def lt_callback(self, msg):
        self.Tp = msg.data
    def rt_callback(self, msg):
        self.Ts = msg.data

def main(args=None):
    rclpy.init(args=args)

    nmpc_node = NMPCNode()
    rclpy.spin(nmpc_node)
    nmpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()