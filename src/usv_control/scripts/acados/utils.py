import numpy as np
import matplotlib.pyplot as plt
from asv_mpc import evaluate_spline

# Plot simulation results
def plot_results(simX, simU, spline_params, dt_sim, algorithm='RTI'):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    time = np.arange(simX.shape[0]) * dt_sim
    
    # Plot 1: XY trajectory with spline
    ax = axes[0, 0]
    # Plot spline
    t_spline = np.linspace(0, 1, 100)
    spline_points = np.array([evaluate_spline(t, spline_params) for t in t_spline])
    ax.plot(spline_points[:, 0], spline_points[:, 1], 'b--', linewidth=2, label='Spline reference')
    ax.plot(simX[:, 0], simX[:, 1], 'r-', linewidth=1.5, label='ASV trajectory')
    ax.plot(simX[0, 0], simX[0, 1], 'go', markersize=10, label='Start')
    ax.plot(simX[-1, 0], simX[-1, 1], 'rs', markersize=10, label='End')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'XY Trajectory ({algorithm})')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    # Plot 2: States vs time
    ax = axes[0, 1]
    ax.plot(time, simX[:, 2], label='Ïˆ [rad]')
    ax.plot(time, simX[:, 3], label='surge [m/s]')
    ax.plot(time, simX[:, 4], label='yaw rate [rad/s]')
    ax.plot(time, simX[:, 5], label='t (spline param)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('State')
    ax.set_title('States vs Time')
    ax.legend()
    ax.grid(True)
    
    # Plot 3: Controls vs time
    ax = axes[1, 0]
    time_u = np.arange(simU.shape[0]) * dt_sim
    ax.plot(time_u, simU[:, 0], label='Ï„_port')
    ax.plot(time_u, simU[:, 1], label='Ï„_stbd')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Thrust [N]')
    ax.set_title('Control Inputs')
    ax.legend()
    ax.grid(True)
    
    # Plot 4: dt (progress rate) vs time
    ax = axes[1, 1]
    ax.plot(time_u, simU[:, 2], 'g-')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('dt (progress rate)')
    ax.set_title('Spline Progress Rate')
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig(f'asv_spline_tracking_{algorithm}.png', dpi=150)
    plt.show()
