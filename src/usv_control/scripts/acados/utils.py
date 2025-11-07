#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#
import matplotlib.pyplot as plt
import numpy as np
from acados_template import latexify_plot
from math import fabs


def plot_pendulum(t, u_max, U, X_true, latexify=False, plt_show=True, time_label='$t$', x_labels=None, u_labels=None):
    """
    Params:
        t: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    nx = X_true.shape[1]
    fig, axes = plt.subplots(nx+1, 1, sharex=True)

    for i in range(nx):
        axes[i].plot(t, X_true[:, i])
        axes[i].grid()
        if x_labels is not None:
            axes[i].set_ylabel(x_labels[i])
        else:
            axes[i].set_ylabel(f'$x_{i}$')

    axes[-1].step(t, np.append([U[0]], U))

    if u_labels is not None:
        axes[-1].set_ylabel(u_labels[0])
    else:
        axes[-1].set_ylabel('$u$')

    axes[-1].hlines(u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    axes[-1].hlines(-u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    axes[-1].set_ylim([-1.2*u_max, 1.2*u_max])
    axes[-1].set_xlim(t[0], t[-1])
    axes[-1].set_xlabel(time_label)
    axes[-1].grid()

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    fig.align_ylabels()

    if plt_show:
        plt.show()


def plot_asv(t, tb, U, X_true, goal, latexify=False, plt_show=True, time_label='$t$', x_labels=None, u_labels=None):
    """
    Params:
        t: time values of the discretization
        tb: bounds of thrust u's (t_port, t_stbd)
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    nx = X_true.shape[1]
    nu = U.shape[1]
    fig, axes = plt.subplots(nx + 1, 1, sharex=True, figsize=(6, 2*(nx+1)))

    # Plot states
    for i in range(nx):
        axes[i].plot(t, X_true[:, i])
        axes[i].grid()
        axes[i].hlines(goal[i], t[0], t[-1], linestyles='dashed', alpha=0.7)

        if x_labels is not None:
            axes[i].set_ylabel(x_labels[i])
        else:
            axes[i].set_ylabel(f'$x_{i}$')

    # Plot controls
    colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
    for i in range(nu):
        axes[-1].step(t, np.append(U[:, i][0], U[:, i]), where='post', linewidth=2, color=colors[i % len(colors)], label=(u_labels[i] if u_labels else f'$u_{i}$'))
    axes[-1].legend()
    axes[-1].set_ylabel('$u$')
    axes[-1].hlines(tb[0], t[0], t[-1], linestyles='dashed', alpha=0.7)
    axes[-1].hlines(tb[1], t[0], t[-1], linestyles='dashed', alpha=0.7)
    axes[-1].set_ylim([1.2*tb[0], 1.2*tb[1]])
    axes[-1].set_xlim(t[0], t[-1])
    axes[-1].set_xlabel(time_label)
    axes[-1].grid(True)

    plt.subplots_adjust(hspace=0.4)

    fig.align_ylabels()

    # --- XY trajectory figure ---
    fig_xy, ax_xy = plt.subplots(figsize=(6, 6))
    ax_xy.plot(X_true[:, 1], X_true[:, 0], linewidth=2, color='tab:blue', label='Trajectory')
    ax_xy.scatter(X_true[0, 1], X_true[0, 0], color='green', marker='o', label='Start')
    ax_xy.scatter(X_true[-1, 1], X_true[-1, 0], color='red', marker='x', label='End')
    ax_xy.scatter(goal[1], goal[0], color='blue', marker='o', label='Goal')
    ax_xy.set_xlabel('$y_{pos}$ [m]')
    ax_xy.set_ylabel('$x_{pos}$ [m]')
    ax_xy.axis('equal')
    ax_xy.grid(True)
    ax_xy.legend()
    ax_xy.set_title('ASV Trajectory (XY plane)')

    plt.show(block=False)
    input("Press Enter to close plots...")

def plot_asv_rti(shooting_nodes, tb, U, X_true, goal, X_est=None, Y_measured=None, latexify=True, plt_show=True, X_true_label=None,
    time_label='$t$', 
    x_labels=['$x_pos$ [m]', '$y_pos$ [m]', '$psi$ [rad]', '$surge$ [m/s]', '$yaw$ [rad/s]'],
    u_labels = ['$t_port$', '$t_stbd$'],
    fig_filename = None,
    title = None
                  ):
    """
    Params:
        shooting_nodes: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        X_est: arrray with shape (N_sim-N_mhe, nx)
        Y_measured: array with shape (N_sim, ny)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    WITH_ESTIMATION = X_est is not None and Y_measured is not None

    N_sim = X_true.shape[0]
    nx = X_true.shape[1]
    nu = U.shape[1]

    Tf = shooting_nodes[N_sim-1]
    t = shooting_nodes

    Ts = t[1] - t[0]
    if WITH_ESTIMATION:
        N_mhe = N_sim - X_est.shape[0]
        t_mhe = np.linspace(N_mhe * Ts, Tf, N_sim-N_mhe)

    plt.subplot(nx+1, 1, 1)
    colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
    for i in range(nu):
        line, = plt.step(t, np.append(U[:, i][0], U[:, i]), where='post', linewidth=2,
                        color=colors[i % len(colors)], label=(u_labels[i] if u_labels else f'$u_{i}$'))
    if X_true_label is not None:
        line.set_label(X_true_label)
    else:
        line.set_color('r')
    if title is not None:
        plt.title(title)
    plt.ylabel('$u$')
    plt.xlabel(time_label)
    plt.hlines(tb[0], t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.hlines(tb[1], t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.ylim([1.2*tb[0], 1.2*tb[1]])
    plt.grid()

    for i in range(nx):
        plt.subplot(nx+1, 1, i+2)
        line, = plt.plot(t, X_true[:, i], label='true')
        plt.hlines(goal[i], t[0], t[-1], linestyles='dashed', alpha=0.7)
        if X_true_label is not None:
            line.set_label(X_true_label)

        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, i], '--', label='estimated')
            plt.plot(t, Y_measured[:, i], 'x', label='measured')

        plt.ylabel(x_labels[i])
        plt.xlabel('$t$')
        plt.grid()
        plt.legend(loc=1)

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    if plt_show:
        plt.show()

    if fig_filename is not None:
        plt.savefig(fig_filename, bbox_inches="tight", transparent=True, pad_inches=0.05)
        print(f"\nstored figure in {fig_filename}")
    
    return