#!/usr/bin/env python3
from rockit import *
from casadi import *
import rockit

import matplotlib.pyplot as plt
import numpy as np

from IPython import embed

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

nx    = 5               # the system is composed of 9 states
nu    = 2               # the system has 2 inputs
dt    = 0.1             # sample time
Tf    = 7.5            # control horizon [s]
Nhor  = (int)(Tf/dt)    # number of control intervals

starting_angle = -0.1
ned_x = 0.01
ned_y = 0.01

# obs_regs = np.array([[6., 2.8, 0.8], 
#                      [3.4, 1.8, 0.8], 
#                      [6., 4.9, 0.8], 
#                      ])

target = np.array([0., 0., 
10., 2.5
])

obs_regs = np.array([
        [3.5, -3.0, 0.5],
        [9.0, -4.1, 0.5],
        [18.0, -2.0, 0.5],
        [19.5, 2.0, 0.5],
        [20.0, 8.1, 0.5],
        [18.0, 15.0, 0.5],
        [3.7, 0.9, 0.5],
        [8.8, -1.0, 0.5],
        [15.0, 1.2, 0.5],
        [17.0, 3.0, 0.5],
        [18.5, 10.1, 0.5],
        [19.0, 18.0, 0.5],
])


Nsim  = int(25 * Nhor / Tf)         # how much samples to simulate

# -------------------------------
# Logging variables
# -------------------------------
x_history     = np.zeros(Nsim+1)
y_history   = np.zeros(Nsim+1)
yaw_history     = np.zeros(Nsim+1)
xd_history     = np.zeros(Nsim+1)
yd_history   = np.zeros(Nsim+1)
u_history     = np.zeros(Nsim+1)
r_history   = np.zeros(Nsim+1)
Tport_history       = np.zeros(Nsim+1)
Tstbd_history       = np.zeros(Nsim+1)
xe_hist = np.zeros(Nsim+1)
ye_hist = np.zeros(Nsim+1)
psie_hist = np.zeros(Nsim+1)
tmp_hist = np.zeros(Nsim+1)

# -------------------------------
# Set OCP
# -------------------------------
ocp = Ocp(T=Tf)
# Define states
nedx = ocp.state()
nedy = ocp.state()
psi = ocp.state()
u = ocp.state()
r = ocp.state()
Tport = ocp.control()
Tstbd = ocp.control()
ocp.set_initial(nedx,ned_x)
ocp.set_initial(nedy,ned_y)
ocp.set_initial(psi,starting_angle)
ocp.set_initial(u,0.0)
ocp.set_initial(r,0.0)
ocp.set_initial(Tport,0.0)
ocp.set_initial(Tstbd,0.0)

# Define parameter
# X_0 = ocp.parameter(nx)
current_X = vertcat(ned_x,ned_y,starting_angle,0.,0.)  # initial state
X_0 = ocp.register_parameter(MX.sym("X_0", nx))
tg = ocp.register_parameter(MX.sym("target", 4))
ocp.set_value(tg, target)
psid = 0.
# p_d = ocp.parameter("psi_d")
# ocp.set_value(p_d, psid)

# Specify ODE
# Xu = if_else(u > 1.25, 64.55, -25.)
# Xuu = if_else(u > 1.25, -70.92, 0.)
Xu = 64.55
Xuu = -70.92
# Yv = 0.5*(-40*1000*fabs(self.v_state))*(1.1+0.0045*(1.01/0.09)-0.1*(0.27/0.09)+0.016*((0.27/0.09)*(0.27/0.09)))
# Nr = (-0.52)*sqrt(u*u + self.v_state*self.v_state)
Nr = (-0.52)*fabs(u)
gamma_p = atan2(ocp._param_value(tg)[3] - ocp._param_value(tg)[1], ocp._param_value(tg)[2] - ocp._param_value(tg)[0])
# gamma_p = ocp._param_value(p_d)[0]
ye = -(nedx-tg[2])*sin(gamma_p)+(nedy-tg[3])*cos(gamma_p)
xe =  (nedx-tg[2])*cos(gamma_p)+(nedy-tg[3])*sin(gamma_p)

ocp.set_der(nedx, u*cos(psi))
ocp.set_der(nedy, u*sin(psi))
ocp.set_der(psi, r)
ocp.set_der(u, 
            ((Tport +  Tstbd
                - (Y_r_dot + N_v_dot)*r*r
                - (-Xu*u - Xuu*fabs(u)*u))
                / (m - X_u_dot)))
ocp.set_der(r,
            (( (Tport - Tstbd) * B / 2 + 
              Nrr*fabs(r)*r + Nr*r) / (Iz - N_r_dot)))

Qxe     = 100.
Qye     = 1000.
Qpsi    = 10.
Qu      = 1000.
Qr      = 1.
# Qds     = 50.

# Lagrange objective
ocp.add_objective(ocp.sum  (Qye*((ye)**2) + Qpsi*(sin(psi)-sin(gamma_p))**2 + 
                            Qpsi*(cos(psi)-cos(gamma_p))**2 + Qu*((u))**2 + Qr*((r))**2 +
                            Qxe*(xe)**2))
ocp.add_objective(ocp.at_tf(Qye*((ye)**2) + Qpsi*(sin(psi)-sin(gamma_p))**2 + 
                            Qpsi*(cos(psi)-cos(gamma_p))**2 + Qu*((u))**2 + Qr*((r))**2 +
                            Qxe*(xe)**2))
# ocp.add_objective(ocp.sum  (Qye*(ye**2) + Qpsi*(sin(psi)-sin(gamma_p))**2 + 
#                             Qpsi*(cos(psi)-cos(gamma_p))**2 + Qu*(u)**2 + Qr*(r)**2 +
#                             Qxe*(xe**2)))
# ocp.add_objective(ocp.at_tf(Qye*(ye**2) + Qpsi*(sin(psi)-sin(gamma_p))**2 + 
#                             Qpsi*(cos(psi)-cos(gamma_p))**2 + Qu*(u)**2 + Qr*(r)**2 +
#                             Qxe*(xe**2)))
# ocp.add_objective(ocp.at_tf(Qye*(ye**2) + Qu*(u**2 + r**2) + Qxe*(xe**2)))
# ocp.add_objective(ocp.T)

# Path constraints
ocp.subject_to( (-0.5 <= u) <= 1. )
ocp.subject_to( (-30.0 <= Tport) <= 36.5 )
ocp.subject_to( (-30.0 <= Tstbd) <= 36.5 )
# ocp.subject_to( (-1.5 <= r) <= 1.5 )

    
# for i,j,k in obs_regs:
#     ocp.add_objective(ocp.sum(Qds/sqrt((i-nedx)**2 + (j-nedy)**2)))
#     ocp.add_objective(ocp.at_tf(Qds/sqrt((i-nedx)**2 + (j-nedy)**2)))


# Initial constraints
X = vertcat(nedx,nedy,psi,u,r)
ocp.subject_to(ocp.at_t0(X)==X_0)

# Pick a solution method
options = {"ipopt": {"print_level": 5}}
options["expand"] = True
options["print_time"] = True
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))

# -------------------------------
# Solve the OCP wrt a parameter value (for the first time)
# -------------------------------
# Set initial value for parameters
ocp.set_value(X_0, current_X)
# Solve
#sol = ocp.solve()

Sim_asv_dyn = ocp._method.discrete_system(ocp)
# Code Gen:
code_gen = True
if code_gen:
    ocp.method(external_method("fatrop", N=Nhor, intg="rk"))
    ocp._method.set_name("/code_gen")

sol = ocp.solve()

# Log data for post-processing
x_history[0]   = current_X[0]
y_history[0] = current_X[1]
yaw_history[0]   = current_X[2]
xd_history[0]   = target[2]
yd_history[0] = target[3]
u_history[0]   = current_X[3]
r_history[0] = current_X[4]
Tport_history[0] = 0.
Tstbd_history[0] = 0.
xe_hist[0] = 0.
ye_hist[0] = 0.
psie_hist[0] = 0.
tmp_hist[0] = 0.

# -------------------------------
# Simulate the MPC solving the OCP (with the updated state) several times
# -------------------------------

for i in range(Nsim):
    # print("timestep", i+1, "of", Nsim)
    # Get the solution from sol
    tsa, Tpsol = sol.sample(Tport, grid='control')
    _, Tssol = sol.sample(Tstbd, grid='control')
    _, xesol = sol.sample(xe, grid='control')
    _, yesol = sol.sample(ye, grid='control')
    _, psisol = sol.sample(psi, grid='control')
    _, gammasol = sol.sample(gamma_p, grid='control')
    # Simulate dynamics (applying the first control input) and update the current state
    current_X = Sim_asv_dyn(x0=current_X, u=vertcat(Tpsol[0],Tssol[0]), T=dt)["xf"]
    ocp.set_value(X_0, vertcat(current_X))
    # Solve the optimization problem
    sol = ocp.solve()
    ocp._method.opti.set_initial(ocp._method.opti.x, ocp._method.opti.value(ocp._method.opti.x))

    # Log data for post-processing
    x_history[i+1]   = current_X[0].full()
    y_history[i+1] = current_X[1].full()
    yaw_history[i+1]   = current_X[2].full()
    u_history[i+1]   = current_X[3].full()
    r_history[i+1] = current_X[4].full()
    Tport_history[i+1] = Tpsol[0]
    Tstbd_history[i+1] = Tssol[0]
    xd_history[i+1]   = target[2]
    yd_history[i+1] = target[3]
    xe_hist[i+1] = xesol[0]
    ye_hist[i+1] = yesol[0]
    psie_hist[i+1] = sqrt((sin(psisol[0])-sin(gamma_p))**2 + (cos(psisol[0])-cos(gamma_p))**2)
    tmp_hist[i+1] = gammasol[0]
    # psie_hist[i+1] = psisol[0]

# -------------------------------
# Plot the results
# -------------------------------
time_sim = np.linspace(0, dt*Nsim, Nsim+1)
time_sim2 = np.linspace(0, dt*Nsim, Nsim)

fig2, ax3 = plt.subplots()
ax3.plot(yd_history, xd_history, 'r-')
ax3.plot(y_history, x_history, 'b--')
for i,j,k in obs_regs:
    ax3.add_patch(plt.Circle((j, i), k, color='black'))
ax3.plot(target[3], target[2], 'go')
ax3.set_xlabel('Y [m]')
ax3.set_ylabel('X [m]')
ax3.set_aspect('equal', adjustable='box')

fig3, ax4 = plt.subplots()
ax4.plot(time_sim, Tport_history, 'r-')
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('Tport [N]', color='r')
ax4.tick_params('y', colors='r')
ax5 = ax4.twinx()
ax5.plot(time_sim, Tstbd_history, 'b-')
ax5.set_ylabel('Tstbd [N]', color='b')
ax5.tick_params('y', colors='b')
fig3.tight_layout()

fig4, ax6 = plt.subplots()
ax6.plot(time_sim, yaw_history, 'b-')
ax6.set_xlabel('Time [s]')
ax6.set_ylabel('yaw [rad]')
fig4.tight_layout()

fig5, ax7 = plt.subplots()
ax7.plot(time_sim, u_history, 'r-')
ax7.set_xlabel('Time [s]')
ax7.set_ylabel('u [m/s]', color='r')
ax7.tick_params('y', colors='r')
ax8 = ax7.twinx()
ax8.plot(time_sim, r_history, 'b-')
ax8.set_ylabel('r [rad/s]', color='b')
ax8.tick_params('y', colors='b')
fig5.tight_layout()

fig6, ax9 = plt.subplots()
ax9.plot(time_sim, xe_hist, 'r-')
ax9.set_xlabel('Time [s]')
ax9.set_ylabel('x_e', color='r')
ax9.tick_params('y', colors='r')
ax10 = ax9.twinx()
ax10.plot(time_sim, ye_hist, 'b-')
ax10.set_ylabel('y_e', color='b')
ax10.tick_params('y', colors='b')
ax11 = ax9.twinx()
ax11.plot(time_sim, psie_hist, 'g-')
ax11.set_ylabel('psi_e', color='g')
ax11.tick_params('y', colors='g')
ax12 = ax9.twinx()
ax12.plot(time_sim, tmp_hist, 'k-')
ax12.set_ylabel('gammap', color='k')
ax12.tick_params('y', colors='k')
fig6.tight_layout()

plt.show()