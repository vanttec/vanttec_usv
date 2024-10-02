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
Tf    = .5                  # control horizon [s]
Nhor  = 5                  # number of control intervals
dt    = Tf/Nhor             # sample time

u_ref = 0.5

class NMPCNode(Node):
    def __init__(self):
        super().__init__('nmpc_node')
        self.x = 0.
        self.y = 0.
        self.psi = 0.
        self.u = 0.
        # self.v = 0.
        self.r = 0.
        self.Tp = 0.
        self.Ts = 0.

        self.u_accum = 0.
        self.r_accum = 0.

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
        self.u_state = self.ocp.state()
        # self.v_state = self.ocp.state()
        self.r_state = self.ocp.state()
        # Defince controls
        self.tu_control = self.ocp.control()
        self.tr_control = self.ocp.control()
        
        # Define parameter
        self.X_0 = self.ocp.parameter(nx)

        # Specify ODE
        self.x_d = self.target[1][0]
        self.y_d = self.target[1][1]
        self.gamma_p = atan2( self.target[1][1] - self.target[0][1] , self.target[1][0] - self.target[0][0] )
        Xu = if_else(self.u_state > 1.25, 64.55, -25)
        Xuu = if_else(self.u_state > 1.25, -70.92, 0)
        # Yv = 0.5*(-40*1000*fabs(self.v_state))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*((0.27/0.09)*(0.27/0.09)))
        # Nr = (-0.52)*sqrt(self.u_state*self.u_state + self.v_state*self.v_state)
        Nr = (-0.52)*sqrt(self.u_state*self.u_state)
        # self.Tu = self.tp_control + self.ts_control
        # self.Tr = ((self.tp_control - self.ts_control) * B / 2)

        self.ye = -(self.nedx_state-self.x_d)*sin(self.gamma_p)+(self.nedy_state-self.y_d)*cos(self.gamma_p)
        self.xe =  (self.nedx_state-self.x_d)*cos(self.gamma_p)+(self.nedy_state-self.y_d)*sin(self.gamma_p)
        self.ocp.set_der(self.nedx_state, self.u_state*cos(self.psi_state))
        self.ocp.set_der(self.nedy_state, self.u_state*sin(self.psi_state))
        self.ocp.set_der(self.psi_state, self.r_state)
        # self.ocp.set_der(self.u_state, 
        #             ((self.Tu - (-m + 2 * Y_v_dot)*self.v_state 
        #               - (Y_r_dot + N_v_dot)*self.r_state*self.r_state
        #               - (-Xu*self.u_state - Xuu*fabs(self.u_state)*self.u_state))
        #                 / (m - X_u_dot)))
        self.ocp.set_der(self.u_state, 
                    ((self.tu_control
                      - (Y_r_dot + N_v_dot)*self.r_state*self.r_state
                      - (-Xu*self.u_state - Xuu*fabs(self.u_state)*self.u_state))
                        / (m - X_u_dot)))
        # self.ocp.set_der(self.v_state, 
        #             ((-(m - X_u_dot)*self.u_state*self.r_state
        #               - (- Yv - Yvv*fabs(self.v_state) - Yvr*fabs(self.r_state))*self.v_state)
        #                 / (m - Y_v_dot)))
        # self.ocp.set_der(self.r_state, 
        #             ((self.Tr - (-2*Y_v_dot*self.u_state*self.v_state 
        #                     - (Y_r_dot + N_v_dot)*self.r_state*self.u_state + X_u_dot*self.u_state*self.u_state) 
        #                     - (-Nr*self.r_state - Nrv*fabs(self.v_state)*self.r_state - Nrr*fabs(self.r_state)*self.r_state)) 
        #                     / (Iz - N_r_dot)))
        self.ocp.set_der(self.r_state, 
                    ((self.tr_control - ( 
                            - (Y_r_dot + N_v_dot)*self.r_state*self.u_state + X_u_dot*self.u_state*self.u_state) 
                            - (-Nr*self.r_state - Nrr*fabs(self.r_state)*self.r_state)) 
                            / (Iz - N_r_dot)))

        Q_xe = 500.0
        Q_ye = 500.0
        Q_psi = 500.0
        Q_u = 50.0
        Q_r = 50.0

        # Lagrange objective
        self.ocp.add_objective(self.ocp.sum(Q_ye*(self.ye**2) + Q_psi*(sin(self.psi_state)-sin(self.gamma_p))**2 + 
                                            Q_psi*(cos(self.psi_state)-cos(self.gamma_p))**2 + Q_xe*(self.xe**2) + 
                                            Q_u*(self.u_state**2) + Q_r*(self.r_state**2)))
        self.ocp.add_objective(self.ocp.at_tf(Q_ye*(self.ye**2) + Q_xe*(self.xe**2) + Q_u*(self.u_state**2) + Q_r*(self.r_state**2)))
        # self.ocp.add_objective(self.ocp.T)

        # Path constraints
        # self.ocp.subject_to( (-0.5 <= self.u_state) <= 1.0 )
        # self.ocp.subject_to( (-0.3 <= self.r_state) <= 0.3 )
        self.ocp.subject_to( (-100. <= self.tu_control) <= 100.0 )
        self.ocp.subject_to( (-50. <= self.tr_control) <= 50.0 )
        # ocp.subject_to( (0 <= d_s) <= d_s_max )   

        # for i,j,k in obs_regs:
        #     self.ocp.subject_to(((i-self.nedx_state)**2 + (j-self.nedy_state)**2) >= k)
        # ocp.subject_to(sqrt(pow(6 - self.nedx_state,2) + pow(3 - self.nedy_state,2)) >= k)

        # Initial constraints
        X = vertcat(self.nedx_state, self.nedy_state, self.psi_state,
                    self.u_state, self.r_state)
        self.ocp.subject_to(self.ocp.at_t0(X)==self.X_0)

        # Pick a solution method
        options = {"ipopt": {"print_level": 0}}
        options["expand"] = True
        options["print_time"] = True
        self.ocp.solver('ipopt',options)
        self.ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))
        self.Sim_asv_dyn = self.ocp._method.discrete_system(self.ocp)

        self.ocp.set_initial(self.nedx_state,self.x)
        self.ocp.set_initial(self.nedy_state,self.y)
        self.ocp.set_initial(self.psi_state,self.psi)
        self.ocp.set_initial(self.u_state,self.u)
        self.ocp.set_initial(self.r_state,self.r)
        self.ocp.set_initial(self.tu_control,self.Ts+self.Tp)
        self.ocp.set_initial(self.tr_control,(self.Tp - self.Ts) * B / 2)
        current_X = vertcat(self.x, self.y, self.psi,
            self.u, self.r)

        self.ocp.set_value(self.X_0, vertcat(current_X))

        self.sol = self.ocp.solve()

        self.timer = self.create_timer(0.05, self.timer_callback)


    def timer_callback(self):
        self.ocp.set_initial(self.nedx_state,self.x)
        self.ocp.set_initial(self.nedy_state,self.y)
        self.ocp.set_initial(self.psi_state,self.psi)
        self.ocp.set_initial(self.u_state,self.u)
        self.ocp.set_initial(self.r_state,self.r)
        # self.ocp.set_initial(self.tu_control,self.Ts+self.Tp)
        # self.ocp.set_initial(self.tr_control,(self.Tp - self.Ts) * B / 2)
        _, self.tu_sol = self.sol.sample(self.tu_control, grid='control')
        _, self.tr_sol = self.sol.sample(self.tr_control, grid='control')

        current_X = vertcat(self.x, self.y, self.psi,
                    self.u, self.r)
        print(current_X)
        current_X = self.Sim_asv_dyn(x0=current_X, u=vertcat(self.tu_sol[0], self.tr_sol[0]), T=dt)["xf"]
        # current_X = self.Sim_asv_dyn(x0=current_X, u=vertcat(self.Ts+self.Tp, self.Tp - self.Ts), T=dt)["xf"]

        self.ocp.set_value(self.X_0, vertcat(current_X))
        self.sol = self.ocp.solve()
        self.ocp._method.opti.set_initial(self.ocp._method.opti.x, self.ocp._method.opti.value(self.ocp._method.opti.x))

        # self.u_d_msg.data = (float)(current_X[3].full())
        _, u_sol = self.sol.sample(self.u_state, grid='control')
        _, r_sol = self.sol.sample(self.r_state, grid='control')
        _, xe_sol = self.sol.sample(self.xe, grid='control')
        _, ye_sol = self.sol.sample(self.ye, grid='control')

        print(fabs(xe_sol[0]) + fabs(ye_sol[0]), 'HHUUHUHU')
        if fabs(xe_sol[0]) + fabs(ye_sol[0]) < 0.0005:
            self.u_d_msg.data = 0.
            self.r_d_msg.data = 0.
        else: 
            self.u_d_msg.data = (float)(u_sol[0])
            self.r_d_msg.data = (float)(r_sol[0])
        # self.u_d_msg.data = (float)(u_sol[0])
        # self.r_d_msg.data = (float)(r_sol[0])

        # self.u_d_msg.data = (float)(self.u_accum)
        # self.r_d_msg.data = (float)(self.r_accum)
        self.u_d_pub_.publish(self.u_d_msg)
        self.r_d_pub_.publish(self.r_d_msg)

    def vel_callback(self, msg):
        # self.u = max(msg.x,0.01)
        self.u = msg.x
        # self.v = msg.y
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