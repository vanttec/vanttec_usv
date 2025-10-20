# Common functions for all scripts
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle, Polygon
import matplotlib.patches as mpatches
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, interp1d

def create_boat_polygon(x, y, theta, length=2.0, width=0.8):
    """
    Create a boat-shaped polygon (pointy at front, wider at back)
    """
    # Boat shape in local coordinates (pointing right)
    boat_shape = np.array([
        [length/4, 0],           # Bow (front point)
        [length/6, width/3],     # Front right
        [-length/8, width/3.8],    # Back right
        [-length/8, -width/3.8],   # Back left
        [length/6, -width/3],    # Front left
        [length/4, 0]            # Back to bow
    ])
    
    # Rotation matrix
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Rotate and translate
    rotated = boat_shape @ R.T
    rotated[:, 0] += x
    rotated[:, 1] += y
    
    return rotated

def animate_results(self, X_opt, U_opt, obstacles=None, success=True, save_as=None):
    """
    Animate the vehicle moving along the optimized trajectory
    
    Args:
        X_opt: State trajectory [x, y, psi, u, r] of shape (5, N+1)
        U_opt: Control inputs [t_port, t_stbd] of shape (2, N)
        obstacles: List of obstacles [(x, y, radius), ...]
        success: Whether optimization converged
        save_as: Optional filename to save animation (e.g., 'animation.mp4' or 'animation.gif')
    """
    time = np.linspace(0, self.T, self.N + 1)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    ax_traj = fig.add_subplot(gs[:2, :2])  # Large trajectory plot
    ax_vel = fig.add_subplot(gs[0, 2])
    ax_heading = fig.add_subplot(gs[1, 2])
    ax_port = fig.add_subplot(gs[2, 0])
    ax_stbd = fig.add_subplot(gs[2, 1])
    ax_error = fig.add_subplot(gs[2, 2])
    
    title_suffix = " (Converged)" if success else " (Debug - Not Converged)"
    color = 'blue' if success else 'red'
    
    # ========== Trajectory Plot ==========
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_title('Vehicle Trajectory Animation' + title_suffix)
    ax_traj.grid(True, alpha=0.3)
    ax_traj.axis('equal')
    
    # Plot full trajectory (faded)
    ax_traj.plot(X_opt[0, :], X_opt[1, :], '--', color=color, alpha=0.3, linewidth=1)
    ax_traj.plot(X_opt[0, 0], X_opt[1, 0], 'go', markersize=12, label='Start', zorder=5)
    ax_traj.plot(X_opt[0, -1], X_opt[1, -1], 'ro', markersize=12, label='End', zorder=5)
    
    # Goal
    ax_traj.plot(self.wp_x, self.wp_y, 
                'k*', markersize=20, alpha=0.8, label='Goal', zorder=6)
    
    # Obstacles
    if obstacles is not None:
        for obs in obstacles:
            circle = Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.3, zorder=1)
            ax_traj.add_patch(circle)
            circle_safe = Circle((obs[0], obs[1]), obs[2] + 0.5, 
                            color='orange', alpha=0.15, linestyle='--', fill=False, zorder=1)
            ax_traj.add_patch(circle_safe)
    
    # Animated elements for trajectory
    trail_line, = ax_traj.plot([], [], color=color, linewidth=2.5, label='Path', zorder=3)
    vehicle_body = Polygon([[0, 0]], color=color, alpha=0.7, zorder=4)
    ax_traj.add_patch(vehicle_body)
    vehicle_heading, = ax_traj.plot([], [], 'k-', linewidth=3, zorder=4)
    time_text = ax_traj.text(0.02, 0.98, '', transform=ax_traj.transAxes, 
                            verticalalignment='top', fontsize=12, 
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    ax_traj.legend(loc='upper right')
    
    # ========== Velocity Plot (Surge) ==========
    ax_vel.plot(time, X_opt[3, :], 'k-', alpha=0.3, linewidth=1)
    ax_vel.axhline(y=self.u_lims[0], color='r', linestyle='--', alpha=0.5, label='Limits')
    ax_vel.axhline(y=self.u_lims[1], color='r', linestyle='--', alpha=0.5)
    vel_line, = ax_vel.plot([], [], color=color, linewidth=2.5)
    vel_point, = ax_vel.plot([], [], 'o', color=color, markersize=8)
    ax_vel.set_xlabel('Time [s]')
    ax_vel.set_ylabel('Surge u [m/s]')
    ax_vel.set_title('Surge Velocity')
    ax_vel.grid(True, alpha=0.3)
    ax_vel.set_xlim(0, self.T)
    ax_vel.set_ylim(self.u_lims[0] * 1.2, self.u_lims[1] * 1.2)
    ax_vel.legend(loc='upper right', fontsize=8)
    
    # ========== Heading Rate Plot (Yaw rate) ==========
    ax_heading.plot(time, X_opt[4, :], 'k-', alpha=0.3, linewidth=1)
    ax_heading.axhline(y=self.r_lims[0], color='r', linestyle='--', alpha=0.5, label='Limits')
    ax_heading.axhline(y=self.r_lims[1], color='r', linestyle='--', alpha=0.5)
    heading_line, = ax_heading.plot([], [], color=color, linewidth=2.5)
    heading_point, = ax_heading.plot([], [], 'o', color=color, markersize=8)
    ax_heading.set_xlabel('Time [s]')
    ax_heading.set_ylabel('Yaw rate r [rad/s]')
    ax_heading.set_title('Yaw Rate')
    ax_heading.grid(True, alpha=0.3)
    ax_heading.set_xlim(0, self.T)
    ax_heading.set_ylim(self.r_lims[0] * 1.2, self.r_lims[1] * 1.2)
    ax_heading.legend(loc='upper right', fontsize=8)
    
    # ========== Port Thruster Plot ==========
    time_control = np.linspace(0, self.T - self.dt, self.N)
    ax_port.plot(time_control, U_opt[0, :], 'k-', alpha=0.3, linewidth=1)
    ax_port.axhline(y=self.tp_lims[0], color='r', linestyle='--', alpha=0.5, label='Limits')
    ax_port.axhline(y=self.tp_lims[1], color='r', linestyle='--', alpha=0.5)
    ax_port.axhline(y=0, color='gray', linestyle='-', alpha=0.3, linewidth=0.8)
    port_line, = ax_port.plot([], [], 'g-', linewidth=2.5)
    port_point, = ax_port.plot([], [], 'go', markersize=8)
    ax_port.set_xlabel('Time [s]')
    ax_port.set_ylabel('Thrust [N]')
    ax_port.set_title('Port Thruster')
    ax_port.grid(True, alpha=0.3)
    ax_port.set_xlim(0, self.T)
    ax_port.set_ylim(self.tp_lims[0] * 1.1, self.tp_lims[1] * 1.1)
    ax_port.legend(loc='upper right', fontsize=8)
    
    # ========== Starboard Thruster Plot ==========
    ax_stbd.plot(time_control, U_opt[1, :], 'k-', alpha=0.3, linewidth=1)
    ax_stbd.axhline(y=self.tp_lims[0], color='r', linestyle='--', alpha=0.5, label='Limits')
    ax_stbd.axhline(y=self.tp_lims[1], color='r', linestyle='--', alpha=0.5)
    ax_stbd.axhline(y=0, color='gray', linestyle='-', alpha=0.3, linewidth=0.8)
    stbd_line, = ax_stbd.plot([], [], 'purple', linewidth=2.5)
    stbd_point, = ax_stbd.plot([], [], 'o', color='purple', markersize=8)
    ax_stbd.set_xlabel('Time [s]')
    ax_stbd.set_ylabel('Thrust [N]')
    ax_stbd.set_title('Starboard Thruster')
    ax_stbd.grid(True, alpha=0.3)
    ax_stbd.set_xlim(0, self.T)
    ax_stbd.set_ylim(self.tp_lims[0] * 1.1, self.tp_lims[1] * 1.1)
    ax_stbd.legend(loc='upper right', fontsize=8)
    
    # ========== Error Plot ==========
    pos_error = np.sqrt((X_opt[0, :] - self.wp_x)**2 + 
                    (X_opt[1, :] - self.wp_y)**2)
    heading_error = np.abs(X_opt[2, :] - self.wp_psi)
    # Handle angle wrapping
    heading_error = np.minimum(heading_error, 2*np.pi - heading_error)
    
    ax_error.plot(time, pos_error, 'k-', alpha=0.3, linewidth=1)
    ax_error.plot(time, heading_error, 'k-', alpha=0.3, linewidth=1)
    error_line1, = ax_error.plot([], [], 'r-', linewidth=2.5, label='Position [m]')
    error_line2, = ax_error.plot([], [], 'b-', linewidth=2.5, label='Heading [rad]')
    error_point1, = ax_error.plot([], [], 'ro', markersize=8)
    error_point2, = ax_error.plot([], [], 'bo', markersize=8)
    ax_error.set_xlabel('Time [s]')
    ax_error.set_ylabel('Error')
    ax_error.set_title('Tracking Error')
    ax_error.grid(True, alpha=0.3)
    ax_error.set_xlim(time[0], time[-1])
    ax_error.legend(loc='upper right', fontsize=8)
    
    # Vehicle dimensions
    vehicle_length = 0.8
    
    def init():
        trail_line.set_data([], [])
        vel_line.set_data([], [])
        vel_point.set_data([], [])
        heading_line.set_data([], [])
        heading_point.set_data([], [])
        port_line.set_data([], [])
        port_point.set_data([], [])
        stbd_line.set_data([], [])
        stbd_point.set_data([], [])
        error_line1.set_data([], [])
        error_point1.set_data([], [])
        error_line2.set_data([], [])
        error_point2.set_data([], [])
        return (trail_line, vehicle_body, vehicle_heading, time_text, 
                vel_line, vel_point, heading_line, heading_point,
                port_line, port_point, stbd_line, stbd_point,
                error_line1, error_point1, error_line2, error_point2)
    
    def animate(frame):
        # Update trail
        trail_line.set_data(X_opt[0, :frame+1], X_opt[1, :frame+1])
        
        # Update vehicle position and orientation
        x, y, psi = X_opt[0, frame], X_opt[1, frame], X_opt[2, frame]
        
        # Vehicle body as boat polygon
        boat_vertices = create_boat_polygon(x, y, psi, length=2.0, width=0.8)
        vehicle_body.set_xy(boat_vertices)

        # Heading indicator (arrow)
        arrow_length = vehicle_length * 0.6
        vehicle_heading.set_data([x, x + arrow_length * np.cos(psi)],
                                [y, y + arrow_length * np.sin(psi)])
        
        # Time text
        time_text.set_text(f'Time: {time[frame]:.2f} s\n'
                          f'Surge: {X_opt[3, frame]:.2f} m/s\n'
                          f'Yaw: {X_opt[4, frame]:.2f} rad/s')
        
        # Update velocity plot (surge)
        vel_line.set_data(time[:frame+1], X_opt[3, :frame+1])
        vel_point.set_data([time[frame]], [X_opt[3, frame]])
        
        # Update yaw rate plot
        heading_line.set_data(time[:frame+1], X_opt[4, :frame+1])
        heading_point.set_data([time[frame]], [X_opt[4, frame]])
        
        # Update control plots
        if frame > 0:
            ctrl_idx = min(frame - 1, self.N - 1)
            port_line.set_data(time_control[:ctrl_idx+1], U_opt[0, :ctrl_idx+1])
            port_point.set_data([time_control[ctrl_idx]], [U_opt[0, ctrl_idx]])
            
            stbd_line.set_data(time_control[:ctrl_idx+1], U_opt[1, :ctrl_idx+1])
            stbd_point.set_data([time_control[ctrl_idx]], [U_opt[1, ctrl_idx]])
        
        # Update error plot
        pos_error_frame = np.sqrt((X_opt[0, :frame+1] - self.wp_x)**2 + 
                                  (X_opt[1, :frame+1] - self.wp_y)**2)
        heading_error_frame = np.abs(X_opt[2, :frame+1] - self.wp_psi)
        heading_error_frame = np.minimum(heading_error_frame, 2*np.pi - heading_error_frame)

        error_line1.set_data(time[:frame+1], pos_error_frame)
        error_point1.set_data([time[frame]], [pos_error_frame[-1]])
        error_line2.set_data(time[:frame+1], heading_error_frame)
        error_point2.set_data([time[frame]], [heading_error_frame[-1]])
        
        return (trail_line, vehicle_body, vehicle_heading, time_text,
                vel_line, vel_point, heading_line, heading_point,
                port_line, port_point, stbd_line, stbd_point,
                error_line1, error_point1, error_line2, error_point2)
    
    # Create animation
    anim = FuncAnimation(fig, animate, init_func=init, frames=self.N + 1,
                        interval=self.dt * 1000, blit=True, repeat=True)
    
    # Save animation if requested
    if save_as is not None:
        if save_as.endswith('.gif'):
            anim.save(save_as, writer='pillow', fps=int(1/self.dt))
        elif save_as.endswith('.mp4'):
            anim.save(save_as, writer='ffmpeg', fps=int(1/self.dt))
        print(f"Animation saved as {save_as}")
    
    plt.show()
    return anim