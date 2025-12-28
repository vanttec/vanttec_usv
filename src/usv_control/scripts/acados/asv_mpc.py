"""
ASV Spline Tracking MPC using ACADOS
"""

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from asv_dynamics import export_asv_model
import numpy as np
import scipy.linalg
from casadi import vertcat, sin, cos, SX
import matplotlib.pyplot as plt

# Setup the OCP
def setup_spline_tracking_ocp(x0, params, Tf, N_horizon, algorithm='RTI'):    
    # Create OCP object
    ocp = AcadosOcp()
    
    # Set model
    model = export_asv_model()
    ocp.model = model

    w_along = model.p[8] 
    w_cross = model.p[9]
    w_heading = model.p[10]
    w_input = model.p[11]
    w_slack = model.p[12]
    w_surge = model.p[13]
    w_yaw = model.p[14]
    w_terminal = model.p[15]
    w_avoidance = model.p[16]

    nx = model.x.rows()  # No. states
    nu = model.u.rows()  # No. controls
    np_param = model.p.rows()  # No. parameters
    
    # Set prediction horizon
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf
    
    # --- COST FUNCTION ---
    # Using EXTERNAL cost type for custom formulation
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    
    
    # Extract states and controls
    x_pos = model.x[0]
    y_pos = model.x[1]
    psi = model.x[2]
    surge = model.x[3]
    yaw = model.x[4]
    t_param = model.x[5]

    obs_n = model.obs_n
    obs = []
    for i in range(obs_n):
        obs.append(model.x[6+2*i])
        obs.append(model.x[7+2*i])
    t_la_param = model.p[17]
    
    tau_port = model.u[0]
    tau_stbd = model.u[1]
    dt = model.u[2]
    slack_u = model.u[3]
    
    # Spline evaluation (defined in model)
    s_x = model.s_x
    s_y = model.s_y
    s_la_x = model.s_la_x
    s_la_y = model.s_la_y
    psi_ref = model.psi_ref
    
    # Stage cost
    L = 0.1
    # Ls = [-L,0.0,L]
    Ls = [0.0]
    # crosstrack_error = (x_pos - s_x)**2 + (y_pos - s_y)**2
    crosstrack_error = 0.0
    for l in Ls:
        crosstrack_error += ((x_pos + l*cos(psi)) - s_x)**2 + ((y_pos + l*sin(psi)) - s_y)**2
    crosstrack_error/=len(Ls)

    alongtrack_error = (x_pos - s_la_x)**2 + (y_pos - s_la_y)**2
    heading_error = sin((psi - psi_ref) / 2)**2
    input_cost = tau_port**2 + tau_stbd**2
    slack_cost = slack_u**2
    surge_cost = surge**2
    yaw_cost = yaw**2

    avoidance_list = [
        [1.2,0.],
        [0.35, 0.0],
        [0.45,-0.25], [0.45,0.25],
        [-0.35,-0.3], [-0.35,0.3],
        [-0.35, 0.0]
    ]
    avoidance_cost = 0.0
    for i in range((int)(obs_n)):
        for avo in avoidance_list:
            x_virt = x_pos + avo[0]*cos(psi) - avo[1]*sin(psi)
            y_virt = y_pos + avo[0]*sin(psi) + avo[1]*cos(psi)
            avoidance_cost += 1.0 / ((np.sqrt((obs[i*2]-x_virt)**2 + (obs[i*2+1]-y_virt)**2) / 3.)**1.5)

    stage_cost = (w_along * alongtrack_error + 
                  w_cross * crosstrack_error + 
                  w_heading * heading_error + 
                  w_input * input_cost +
                  w_slack * slack_cost +
                  w_surge * surge_cost +
                  w_yaw * yaw_cost +
                  w_avoidance * avoidance_cost
                  )
    
    ocp.model.cost_expr_ext_cost = stage_cost
    
    # Terminal cost (same without input term)
    terminal_cost = w_terminal * (w_along * alongtrack_error + 
                    w_cross * crosstrack_error + 
                    w_heading * heading_error +
                    w_surge * surge_cost +
                    w_yaw * yaw_cost +
                    w_avoidance * avoidance_cost
                    )
    
    ocp.model.cost_expr_ext_cost_e = terminal_cost
    
    # --- CONSTRAINTS ---
    # Initial state constraint
    ocp.constraints.x0 = x0
    
    # Control bounds
    tau_max = 36.5
    tau_min = -30.5
    dt_max = 0.1
    dt_min = -0.01
    slack_u_max = 1.0
    slack_u_min = 0.0
    
    ocp.constraints.lbu = np.array([tau_min, tau_min, dt_min, slack_u_min])
    ocp.constraints.ubu = np.array([tau_max, tau_max, dt_max, slack_u_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    # Path constraints: 
    # First: surge + slack_u >= 0 (from surge>=0-slack)
    # Second: t <= t_la
    # ocp.constraints.lh = np.array([0.0,-1e10])
    # ocp.constraints.uh = np.array([1e10,0.0])
    # ocp.model.con_h_expr = vertcat(
    #     model.x[3] + model.u[3],  # surge + slack_u >= 0
    #     model.x[5] - model.p[16]  # t - t_la <= 0
    # )
    
    # State bounds
    ocp.constraints.lbx = np.array([0.0,-1.5,-1.5])
    ocp.constraints.ubx = np.array([1.0,1.5,1.5])
    ocp.constraints.idxbx = np.array([5,3,4])  # Index in state vector (t, surge, yaw)
    
    # State bounds at terminal stage
    ocp.constraints.lbx_e = np.array([0.0,0.0,0.0])
    ocp.constraints.ubx_e = np.array([1.0,0.0,0.0])
    ocp.constraints.idxbx_e = np.array([5,3,4])
    
    # Set spline parameters
    ocp.parameter_values = params
    
    # --- SOLVER OPTIONS ---
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.qp_solver_iter_max = 200
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.qp_solver_cond_N = N_horizon

    # Claude SUGGESTION
    # ============================================
    # STRONG REGULARIZATION
    # ============================================
    ocp.solver_options.regularize_method = 'PROJECT_REDUC_HESS'  # Better than CONVEXIFY
    ocp.solver_options.levenberg_marquardt = 1e-2  # Strong regularization
    
    # Relax QP tolerances
    ocp.solver_options.qp_solver_tol_stat = 1e-6
    ocp.solver_options.qp_solver_tol_eq = 1e-6
    ocp.solver_options.qp_solver_tol_ineq = 1e-6
    ocp.solver_options.qp_solver_tol_comp = 1e-6
    
    # ============================================
    # CRITICAL: Enable globalization for robustness
    # ============================================
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.alpha_min = 0.01
    ocp.solver_options.alpha_reduction = 0.7
    # Claude SUGGESTION FINISH
    
    # Configure algorithm type
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    # Code generation
    ocp.code_export_directory = f'c_generated_code_asv_ocp'
    
    # Create solver
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file='asv_ocp.json')
    
    return acados_ocp_solver


# Setup simulator for closed-loop testing
def setup_integrator(dt, params):
    sim = AcadosSim()
    sim.model = export_asv_model()
    
    sim.solver_options.T = dt
    sim.solver_options.num_steps = 2
    sim.code_export_directory = 'c_generated_code_asv_sim'
    
    # Set spline parameters for simulation
    sim.parameter_values = params
    
    acados_integrator = AcadosSimSolver(sim)
    return acados_integrator


# Compute Catmull-Rom spline coefficients for segment between p1 and p2.
def get_catmull_rom_segment(p0, p1, p2, p3, alpha=1.0, tension=0.2):
    def distance(a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
    t01 = distance(p0, p1)**alpha
    t12 = distance(p1, p2)**alpha
    t23 = distance(p2, p3)**alpha
    
    # Tangent vectors
    m1 = (1.0 - tension) * (p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 + t12)))
    m2 = (1.0 - tension) * (p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12 + t23)))
    
    # Hermite basis coefficients
    a = 2.0 * (p1 - p2) + m1 + m2
    b = -3.0 * (p1 - p2) - m1 - m1 - m2
    c = m1
    d = p1
    
    # Return as flat array [a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y]
    return np.array([a[0], b[0], c[0], d[0], a[1], b[1], c[1], d[1]])


# Evaluate spline at parameter t
def evaluate_spline(t, params):
    a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y = params
    x = a_x * t**3 + b_x * t**2 + c_x * t + d_x
    y = a_y * t**3 + b_y * t**2 + c_y * t + d_y
    return np.array([x, y])

def main(algorithm='RTI', simulate=True):
    # --- Simulation setup ---
    Tf = 2.50        # MPC prediction horizon [s]
    N_horizon = 50  # Number of shooting nodes
    dt = Tf / N_horizon

    x0 = np.array([
        2.0,          # x_dot
        1.0,          # y_dot
        0.0,          # psi
        0.0,          # surge
        0.0,          # yaw
        0.0,           # t (start at beginning of spline)
        3.0,
        0.0,
        10.0,
        0.0,
        20.0,
        0.0,
    ])
    
    T_sim = 20.0    # Total simulation time [s]
    Nsim = int(T_sim / dt)
    nx = 12
    nu = 4
    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))
    simX[0, :] = x0
    t_preparation = np.zeros(Nsim)
    t_feedback = np.zeros(Nsim)
    
    # --- Define spline ---
    # Control points for Catmull-Rom spline
    p0 = np.array([0.0, 0.0])
    p1 = np.array([2.0, 1.0])
    p2 = np.array([12.0, -3.0])
    p3 = np.array([14.0, 2.0])
    
    spline_params = get_catmull_rom_segment(p0, p1, p2, p3)
    print(f"Spline parameters: {spline_params}")
    
    w_params = np.array([
        1.0,
        0.06,
        1.0,
        0.01,
        1000.0,
        0.0,
        0.0,
        5.0,
        1.0
    ])
    add_params = np.array([1.0])
    ov_params = np.zeros(6)

    params = np.concatenate((spline_params,w_params, add_params, ov_params))

    # --- Setup solver and integrator ---
    print("Setting up OCP solver...")
    ocp_solver = setup_spline_tracking_ocp(x0, params, Tf, N_horizon, algorithm)

    if simulate:
        integrator = setup_integrator(dt, params)

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        plt.ion()  # Turn on interactive mode
        
        # Plot spline reference path
        ax_traj = axes[0, 0]
        t_spline = np.linspace(0, 1, 100)
        spline_points = np.array([evaluate_spline(t, spline_params) for t in t_spline])
        ax_traj.plot(spline_points[:, 0], spline_points[:, 1], 'b--', linewidth=2, label='Spline reference')
        ax_traj.plot(x0[0], x0[1], 'go', markersize=10, label='Start')
        ax_traj.set_xlabel('X [m]')
        ax_traj.set_ylabel('Y [m]')
        ax_traj.set_title(f'XY Trajectory ({algorithm})')
        ax_traj.legend()
        ax_traj.grid(True)
        ax_traj.axis('equal')
        
        # Plot obstacles
        for j in range(3):  # 3 obstacles
            circle = plt.Circle((x0[6+2*j], x0[7+2*j]), radius=0.30, color='red', alpha=0.2)
            ax_traj.add_patch(circle)
        
        # Initialize empty line objects for updating
        line_traj, = ax_traj.plot([], [], 'r-', linewidth=1.5, label='ASV trajectory')
        point_current, = ax_traj.plot([], [], 'ko', markersize=8)
        
        # Setup other subplots
        ax_states = axes[0, 1]
        ax_states.set_xlabel('Time [s]')
        ax_states.set_ylabel('State')
        ax_states.set_title('States vs Time')
        ax_states.grid(True)
        line_psi, = ax_states.plot([], [], label='ψ [rad]')
        line_surge, = ax_states.plot([], [], label='surge [m/s]')
        line_yaw, = ax_states.plot([], [], label='yaw rate [rad/s]')
        line_t, = ax_states.plot([], [], label='t (spline param)')
        ax_states.legend()
        
        ax_controls = axes[1, 0]
        ax_controls.set_xlabel('Time [s]')
        ax_controls.set_ylabel('Thrust [N]')
        ax_controls.set_title('Control Inputs')
        ax_controls.grid(True)
        line_port, = ax_controls.plot([], [], label='τ_port')
        line_stbd, = ax_controls.plot([], [], label='τ_stbd')
        ax_controls.legend()
        
        ax_dt = axes[1, 1]
        ax_dt.set_xlabel('Time [s]')
        ax_dt.set_ylabel('dt (progress rate)')
        ax_dt.set_title('Spline Progress Rate')
        ax_dt.grid(True)
        line_dt, = ax_dt.plot([], [], 'g-')
        
        plt.tight_layout()
        
        print(f"\nRunning closed-loop simulation for {T_sim}s with {algorithm}...")
        print(f"Number of steps: {Nsim}")
        
        # --- Closed-loop simulation ---
        for i in range(Nsim):
            # Update spline parameters for all stages
            for j in range(N_horizon + 1):
                ocp_solver.set(j, "p", params)
            
            # ===== RTI TWO-PHASE APPROACH =====
            
            # PREPARATION PHASE
            ocp_solver.options_set('rti_phase', 1)
            status = ocp_solver.solve()
            t_preparation[i] = ocp_solver.get_stats('time_tot')
            
            if status not in [0, 2, 5]:
                print(f"Warning: Preparation phase returned status {status} at step {i}")
            
            # Set initial state constraint
            ocp_solver.set(0, "lbx", simX[i, :])
            ocp_solver.set(0, "ubx", simX[i, :])
            
            # FEEDBACK PHASE
            ocp_solver.options_set('rti_phase', 2)
            status = ocp_solver.solve()
            t_feedback[i] = ocp_solver.get_stats('time_tot')
            
            if status not in [0, 2, 5]:
                print(f"Warning: Feedback phase returned status {status} at step {i}")
            
            # Get control input
            simU[i, :] = ocp_solver.get(0, "u")
            
            # Simulate system
            simX[i + 1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :], p=params)
            
            # ===== UPDATE PLOTS =====
            time = np.arange(i + 2) * dt
            time_u = np.arange(i + 1) * dt
            
            # Update trajectory
            line_traj.set_data(simX[:i+2, 0], simX[:i+2, 1])
            point_current.set_data([simX[i+1, 0]], [simX[i+1, 1]])
            
            # Update states
            line_psi.set_data(time, simX[:i+2, 2])
            line_surge.set_data(time, simX[:i+2, 3])
            line_yaw.set_data(time, simX[:i+2, 4])
            line_t.set_data(time, simX[:i+2, 5])
            ax_states.relim()
            ax_states.autoscale_view()
            
            # Update controls
            line_port.set_data(time_u, simU[:i+1, 0])
            line_stbd.set_data(time_u, simU[:i+1, 1])
            ax_controls.relim()
            ax_controls.autoscale_view()
            
            # Update dt
            line_dt.set_data(time_u, simU[:i+1, 2])
            ax_dt.relim()
            ax_dt.autoscale_view()
            
            # Refresh display
            plt.pause(0.001)  # Very brief pause for smooth animation
            
            # Print progress
            if (i + 1) % 50 == 0 or i == 0:
                print(f"Step {i+1}/{Nsim}: t={simX[i,5]:.3f}, "
                        f"pos=({simX[i,0]:.2f}, {simX[i,1]:.2f}), "
                        f"prep={t_preparation[i]*1000:.2f}ms, feedback={t_feedback[i]*1000:.2f}ms")
        
        # --- Results ---
        print(f"\n=== Simulation Complete ({algorithm}) ===")
        t_preparation *= 1000  # Convert to ms
        t_feedback *= 1000
        print(f"Preparation phase [ms]: min={np.min(t_preparation):.3f}, "
                f"median={np.median(t_preparation):.3f}, max={np.max(t_preparation):.3f}")
        print(f"Feedback phase [ms]: min={np.min(t_feedback):.3f}, "
                f"median={np.median(t_feedback):.3f}, max={np.max(t_feedback):.3f}")
        print(f"Total computation [ms]: min={np.min(t_preparation+t_feedback):.3f}, "
                f"median={np.median(t_preparation+t_feedback):.3f}, max={np.max(t_preparation+t_feedback):.3f}")
        
        # Turn off interactive mode and show final plot
        plt.ioff()
        plt.show(block=True)

if __name__ == '__main__':
    main(algorithm='RTI', simulate=False)