"""
ASV Spline Tracking MPC using ACADOS
"""

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from asv_dynamics import export_asv_model
import numpy as np
import scipy.linalg
from casadi import vertcat, sin, SX
import matplotlib.pyplot as plt

# Setup the OCP
def setup_spline_tracking_ocp(x0, spline_params, Tf, N_horizon, algorithm='RTI'):    
    # Create OCP object
    ocp = AcadosOcp()
    
    # Set model
    model = export_asv_model()
    ocp.model = model

    # Cost weights
    w_along = 200.0      # Alongtrack error weight
    w_cross = 5000.0     # Crosstrack error weight
    w_heading = 100.0    # Heading alignment weight
    w_input = 10.0      # Input regularization weight
    w_slack = 100.0  # Large weight
    w_surge = 10.0
    w_yaw = 10.0
    w_terminal = 100.0  # Terminal cost multiplier weight

    weight_params = np.array([w_along, w_cross, w_heading, w_input, w_slack, w_surge, w_yaw, w_terminal])
    
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
    
    tau_port = model.u[0]
    tau_stbd = model.u[1]
    dt = model.u[2]
    slack_u = model.u[3]
    
    # Spline evaluation (defined in model)
    s_x = model.s_x
    s_y = model.s_y
    psi_ref = model.psi_ref
    
    # Stage cost
    alongtrack_error = (1 - t_param)**2
    crosstrack_error = (x_pos - s_x)**2 + (y_pos - s_y)**2
    heading_error = sin((psi - psi_ref) / 2)**2
    input_cost = tau_port**2 + tau_stbd**2
    slack_cost = slack_u**2
    yaw_cost = yaw**2
    surge_cost = surge**2
    
    stage_cost = (w_along * alongtrack_error + 
                  w_cross * crosstrack_error + 
                  w_heading * heading_error + 
                  w_input * input_cost +
                  w_slack * slack_cost +
                  w_surge * surge_cost +
                  w_yaw * yaw_cost                  
                  )
    
    ocp.model.cost_expr_ext_cost = stage_cost
    
    # Terminal cost (same without input term)
    terminal_cost = w_terminal * (w_along * alongtrack_error + 
                    w_cross * crosstrack_error + 
                    w_heading * heading_error +
                    w_surge * surge_cost +
                    w_yaw * yaw_cost
                    )
    
    ocp.model.cost_expr_ext_cost_e = terminal_cost
    
    # --- CONSTRAINTS ---
    # Initial state constraint
    ocp.constraints.x0 = x0
    
    # Control bounds
    tau_max = 36.5
    tau_min = -30.5
    dt_max = 0.5  # Maximum progress rate along spline per time step
    dt_min = 0.0
    slack_u_max = 10.0
    slack_u_min = 0.0
    
    ocp.constraints.lbu = np.array([tau_min, tau_min, dt_min, slack_u_min])
    ocp.constraints.ubu = np.array([tau_max, tau_max, dt_max, slack_u_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    # Soft constraints: surge + slack_u >= 0 (from surge>=0-slack)
    ocp.constraints.lh = np.array([0.0])
    ocp.constraints.uh = np.array([1e10])
    ocp.model.con_h_expr = model.x[3] + model.u[3]  # surge + slack_u >= 0
    
    # State bounds
    ocp.constraints.lbx = np.array([0.0,-0.5,-1.5])
    ocp.constraints.ubx = np.array([1.0,1.5,1.5])
    ocp.constraints.idxbx = np.array([5,3,4])  # Index in state vector (t, surge)
    
    # Apply same bounds at terminal stage
    ocp.constraints.lbx_e = np.array([0.0,0.0,0.0])
    ocp.constraints.ubx_e = np.array([1.0,0.0,0.0])
    ocp.constraints.idxbx_e = np.array([5,3,4])
    
    # Set spline parameters
    ocp.parameter_values = np.concatenate((spline_params,weight_params))
    
    # --- SOLVER OPTIONS ---
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.qp_solver_iter_max = 200
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.qp_solver_cond_N = N_horizon

    # Claude SUGGESTION
    # ============================================
    # KEY: STRONG REGULARIZATION
    # ============================================
    ocp.solver_options.regularize_method = 'PROJECT_REDUC_HESS'  # Better than CONVEXIFY
    ocp.solver_options.levenberg_marquardt = 1e-2  # Strong regularization
    
    # Relax QP tolerances
    ocp.solver_options.qp_solver_tol_stat = 1e-3
    ocp.solver_options.qp_solver_tol_eq = 1e-3
    ocp.solver_options.qp_solver_tol_ineq = 1e-3
    ocp.solver_options.qp_solver_tol_comp = 1e-3
    
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
    plt.show()


def main(algorithm='RTI', simulate=True):
    # --- Simulation setup ---
    Tf = 2.50        # MPC prediction horizon [s]
    N_horizon = 50  # Number of shooting nodes
    dt = Tf / N_horizon
    
    T_sim = 20.0    # Total simulation time [s]
    Nsim = int(T_sim / dt)
    
    # --- Define spline ---
    # Control points for Catmull-Rom spline
    p0 = np.array([0.0, 0.0])
    p1 = np.array([2.0, 1.0])
    p2 = np.array([6.0, -3.0])
    p3 = np.array([10.0, 2.0])
    
    spline_params = get_catmull_rom_segment(p0, p1, p2, p3)
    print(f"Spline parameters: {spline_params}")
    
    # --- Initial conditions ---
    # Start slightly off the spline to see tracking behavior
    x0 = np.array([
        5.0,          # x_dot
        1.0,          # y_dot
        0.0,          # psi
        0.0,          # surge
        0.0,          # yaw
        0.3           # t (start at beginning of spline)
    ])
    
    # --- Setup solver and integrator ---
    print("Setting up OCP solver...")
    ocp_solver = setup_spline_tracking_ocp(x0, spline_params, Tf, N_horizon, algorithm)

    if simulate:
        print("Setting up integrator...")
        integrator = setup_integrator(dt, spline_params)
        
        # --- Simulation arrays ---
        nx = 6
        nu = 4
        simX = np.zeros((Nsim + 1, nx))
        simU = np.zeros((Nsim, nu))
        simX[0, :] = x0
        
        # Timing arrays - different structure for RTI vs SQP
        t_preparation = np.zeros(Nsim)
        t_feedback = np.zeros(Nsim)
        
        print(f"\nRunning closed-loop simulation for {T_sim}s with {algorithm}...")
        print(f"Number of steps: {Nsim}")
        
        # --- Closed-loop simulation ---
        for i in range(Nsim):
            # Update spline parameters for all stages
            for j in range(N_horizon + 1):
                ocp_solver.set(j, "p", spline_params)
            
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
            simX[i + 1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :], p=spline_params)
            
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

        print(f"Final t parameter: {simX[-1, 5]:.3f}")
        print(f"Final position: ({simX[-1, 0]:.2f}, {simX[-1, 1]:.2f})")
        
        # Calculate final tracking errors
        final_spline_point = evaluate_spline(simX[-1, 5], spline_params)
        final_crosstrack = np.linalg.norm(simX[-1, :2] - final_spline_point)
        print(f"Final crosstrack error: {final_crosstrack:.3f} m")
        
        # Plot results
        print("\nGenerating plots...")
        plot_results(simX, simU, spline_params, dt, algorithm)

if __name__ == '__main__':
    main(algorithm='RTI', simulate=False)