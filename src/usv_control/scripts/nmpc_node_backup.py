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

from rockit import *
from casadi import *

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

# -------------------------------
# Problem parameters
# -------------------------------
X_u_dot = -2.25
Y_v_dot = -23.13
Y_r_dot = -1.31
N_v_dot = -16.41
N_r_dot = -2.79
Yvv = -99.99
Yvr = -5.49
Yrv = -5.49
Yrr = -8.8
Nvv = -5.49
Nvr = -8.8
Nrv = -8.8
Nrr = -3.49
m = 30
Iz = 4.1
B = 0.41

nx    = 5                   # the system is composed of 9 states
nu    = 2                   # the system has 2 inputs
Tf    = 1.5                   # control horizon [s]
Nhor  = 15                  # number of control intervals
dt    = Tf/Nhor             # sample time

u_ref = 0.5

# target = np.array([[0., 0.], [10., 5.]])
target = np.array([[0., 0.], [2., 0.]])
obs_regs = np.array([[6, 3, 0.1], 
                     [3.4, 1.8, 0.1], 
                     [6, 3.7, 0.1], 
                     ])

class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')
        self.x = 0.
        self.y = 0.
        self.psi = 0.
        self.u = 0.01
        self.r = 0.
        self.Tp = 0.
        self.Ts = 0.

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
        

        self.u_d_pub_ = self.create_publisher(Float64, "/guidance/desired_velocity", 10)
        self.r_d_pub_ = self.create_publisher(Float64, "/guidance/desired_angular_velocity", 10)

        self.u_d_msg = Float64()
        self.r_d_msg = Float64()


        self.ocp = Ocp(T=Tf)
        # Define states
        self.nedx_state = self.ocp.state()
        self.nedy_state = self.ocp.state()
        self.psi_state = self.ocp.state()
        self.u_state = self.ocp.state()
        self.r_state = self.ocp.state()
        # Defince controls
        Tport = self.ocp.control()
        Tstbd = self.ocp.control()
        
        self.ocp.set_initial(self.nedx_state,self.x)
        self.ocp.set_initial(self.nedy_state,self.y)
        self.ocp.set_initial(self.psi_state,self.psi)
        self.ocp.set_initial(self.u_state,self.u)
        self.ocp.set_initial(self.r_state,self.r)
        self.ocp.set_initial(Tport,self.Tp)
        self.ocp.set_initial(Tstbd,self.Ts)


        # Define parameter
        self.X_0 = self.ocp.parameter(nx)

        # Specify ODE
        x_d = target[1][0]
        y_d = target[1][1]
        gamma_p = atan2( target[1][0] - target[0][0], target[1][1] - target[0][1])
        ye = -(self.nedx_state-x_d)*sin(gamma_p)+(self.nedy_state-y_d)*cos(gamma_p)
        xe = (self.nedx_state - x_d)*cos(gamma_p)+(self.nedy_state-y_d)*sin(gamma_p)
        Tu = Tport +  Tstbd
        Tr = (Tport - Tstbd) * B / 2
        self.ocp.set_der(self.nedx_state, self.u_state*cos(self.psi_state))
        self.ocp.set_der(self.nedy_state, self.u_state*sin(self.psi_state))
        self.ocp.set_der(self.psi_state, self.r_state)
        self.ocp.set_der(self.u_state, Tu)
        self.ocp.set_der(self.r_state, Tr)

        Qxe = 25.0
        Qye = 30.0
        Qpsi = 1.0
        Qu = 25.0

        # Lagrange objective
        self.ocp.add_objective(self.ocp.sum(Qye*(ye**2) + Qpsi*(sin(self.psi_state)-sin(gamma_p))**2 + Qpsi*(cos(self.psi_state)-cos(gamma_p))**2 + Qu*(self.u_state-u_ref)**2 + Qxe*(xe**2)))
        self.ocp.add_objective(self.ocp.at_tf(Qye**2*(ye**2) + Qxe**2*(xe**2)))
        # self.ocp.add_objective(self.ocp.T)

        # Path constraints
        # self.ocp.subject_to( (-0.5 <= self.u_state) <= 10.0 )
        self.ocp.subject_to( (-30.0 <= Tport) <= 35.0 )
        self.ocp.subject_to( (-30.0 <= Tstbd) <= 35.0 )
        # self.ocp.subject_to( (-1.0 <= self.r_state) <= 1.0 )
        # ocp.subject_to( (0 <= d_s) <= d_s_max )   

        # for i,j,k in obs_regs:
        #     self.ocp.subject_to(((i-self.nedx_state)**2 + (j-self.nedy_state)**2) >= k)
        # ocp.subject_to(sqrt(pow(6 - self.nedx_state,2) + pow(3 - self.nedy_state,2)) >= k)

        # Initial constraints
        X = vertcat(self.nedx_state,self.nedy_state,self.psi_state,self.u_state, self.r_state)
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
        current_X = vertcat(self.x,self.y,self.psi,self.u,self.r)
        print(current_X)
        current_X = self.Sim_asv_dyn(x0=current_X, u=vertcat(self.Tp, self.Ts), T=dt)["xf"]

        self.ocp.set_value(self.X_0, vertcat(current_X))
        sol = self.ocp.solve()
        self.ocp._method.opti.set_initial(self.ocp._method.opti.x, self.ocp._method.opti.value(self.ocp._method.opti.x))

        # self.u_d_msg.data = (float)(current_X[3].full())
        self.u_d_msg.data = 0.5
        self.r_d_msg.data = (float)(current_X[4].full())

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