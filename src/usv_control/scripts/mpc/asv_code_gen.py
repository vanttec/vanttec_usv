import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, interp1d
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
import matplotlib.patches as mpatches
import math

from mpc import *

wp_x = 1.0
wp_y = 0.0
wp_psi = 0.5

class AsvOpt:
    def __init__(self):
        # Parameters
        self.u_lims = [-0.3 , 1.5] # min/max surge [m/s]
        self.r_lims = [-1.5 , 1.5] # min/max yaw [rad/s]
        self.tp_lims = [-30.0, 36.5] # min/max port thrust
        self.ts_lims = [-30.0, 36.5] # min/max stbd thrust
        
        # Optimization parameters
        self.T = 5.0  # time horizon [s]
        self.N = 100  # number of control intervals (K-1 in fatrop notation)
        self.K = self.N + 1  # number of time steps (K in fatrop notation)
        self.dt = self.T / self.N  # time step
        
        # Goal tracking weights
        self.Q_pos = 100.0  # position tracking weight
        self.Q_heading = 10.0  # heading tracking weight
        self.R_u = 0.1  # surge penalty weight
        self.R_r = 0.1  # yaw penalty weight
        self.R_tp = 0.001  # tp penalty weight
        self.R_ts = 0.001  # ts penalty weight
        
        # Define dimensions following fatrop structure
        self.nx = [5 for _ in range(self.K)]  # state dimensions
        self.nu = [2 for _ in range(self.N)] + [0]  # control dimensions

        self.wp_x = wp_x
        self.wp_y = wp_y
        self.wp_psi = wp_psi
                     
    def discrete_dynamics(self, uk, xk, k):
        """
        Discrete dynamics using RK4 integration
        Following fatrop structure: returns x_{k+1}
        """
        # RK4 integration of bicycle model
        def bicycle_model_continuous(x, u):
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
        
            x_pos, y_pos, psi, surge, yaw = x[0], x[1], x[2], x[3], x[4]
            t_port, t_stbd = u[0], u[1]
            
            Xu = 64.55
            Xuu = -70.92
            Nr = (-0.52)*ca.fabs(surge)

            x_dot = surge * ca.cos(psi)
            y_dot = surge * ca.sin(psi)
            psi_dot = yaw

            surge_dot = ((t_port +  t_stbd
                - (Y_r_dot + N_v_dot)*yaw*yaw
                - (-Xu*surge - Xuu*ca.fabs(surge)*surge))
                / (m - X_u_dot))
            yaw_dot = (( (t_port - t_stbd) * B / 2 + 
              Nrr*ca.fabs(yaw)*yaw + Nr*yaw) / (Iz - N_r_dot))
            
            return ca.vertcat(x_dot, y_dot, psi_dot, surge_dot, yaw_dot)
        
        k1 = bicycle_model_continuous(xk, uk)
        k2 = bicycle_model_continuous(xk + self.dt/2 * k1, uk)
        k3 = bicycle_model_continuous(xk + self.dt/2 * k2, uk)
        k4 = bicycle_model_continuous(xk + self.dt * k3, uk)
        
        return xk + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    
    def cost(self, uk, xk, k, goal_state):
        """
        Stage cost function for reference tracking
        """
        cost_val = 0

        # Goal position tracking error
        pos_error = (xk[0] - goal_state[0])**2 + (xk[1] - goal_state[1])**2
        cost_val += self.Q_pos * pos_error

        # Goal heading tracking error (handle angle wrapping)
        heading_error = ca.sin(xk[2] - goal_state[2])**2 + (1 - ca.cos(xk[2] - goal_state[2]))**2
        cost_val += self.Q_heading * heading_error

        # Surge error
        surge_error = (xk[3] - goal_state[3]) ** 2
        cost_val += self.R_u + surge_error

        # Yaw rate error
        yaw_error = (xk[4] - goal_state[4]) ** 2
        cost_val += self.R_r + yaw_error
                
        # Control effort penalties
        if k < self.N:
            # cost_val += self.R_u * xk[3]**2  # surge penalty
            # cost_val += self.R_r * xk[4]**2  # yaw penalty
            cost_val += self.R_tp * uk[0]**2  # tp penalty
            cost_val += self.R_ts * uk[1]**2  # ts penalty
        
        # Terminal weight
        if k == self.K - 1:
            terminal_weight = 1000.0  # 100x heavier
            cost_val += terminal_weight * self.Q_pos * pos_error
            cost_val += terminal_weight * self.Q_heading * heading_error
            cost_val += terminal_weight * self.R_u * surge_error
            cost_val += terminal_weight * self.R_r * yaw_error

        return cost_val
    
    def path_constraints(self, uk, xk, k, start_state, goal_state, obstacles=None):
        """
        Path constraints following fatrop structure
        For reference tracking, goal_state can be None (no terminal constraints)
        """
        cc = []
        
        # Equality constraints
        if k == 0:
            # Initial condition
            cc.append(xk[0] - start_state[0] == 0.)
            cc.append(xk[1] - start_state[1] == 0.)
            cc.append(ca.sin(xk[2] - start_state[2])**2 + (1 - ca.cos(xk[2] - start_state[2]))**2 == 0.)
            cc.append(xk[3] - start_state[3] == 0.)
            cc.append(xk[4] - start_state[4] == 0.)
        # elif k == self.K - 1:
            # Terminal condition
            # cc.append(xk[0] - goal_state[0] == 0.)
            # cc.append(xk[1] - goal_state[1] == 0.)
            # cc.append(ca.sin(xk[2] - goal_state[2])**2 + (1 - ca.cos(xk[2] - goal_state[2]))**2 == 0.)
            # cc.append(xk[3] - goal_state[3] == 0.)
            # cc.append(xk[4] - goal_state[4] == 0.)
        
        # Inequality constraints
        # State bounds
        if k > 0:
            cc.append(self.u_lims[0]<= (xk[3] <= self.u_lims[1]))  # surge bounds
            cc.append(self.r_lims[0]<= (xk[4] <= self.r_lims[1]))  # yaw bounds
        
        # Control bounds (only if controls exist)
        if k < self.N:
            cc.append(self.tp_lims[0] <= (uk[0] <= self.tp_lims[1]))  # tp bounds
            cc.append(self.ts_lims[0] <= (uk[1] <= self.ts_lims[1]))  # ts bounds
        
        # Obstacle avoidance constraints
        # if obstacles is not None:
            # for obs in obstacles:
            #     obs_x, obs_y, obs_radius = obs
            #     dist = (xk[0] - obs_x)**2 + (xk[1] - obs_y)**2
            #     cc.append(obs_radius**2 <= dist)  # minimum clearance
        
        return cc
    
    def setup_optimization_problem(self, start_state, goal_state, obstacles=None):
        """
        Set up the trajectory optimization problem following fatrop structure
        """
        # Create optimization problem
        opti = ca.Opti()
        
        # Decision variables - following fatrop structure
        x = []
        u = []
        ng = []

        p_start = opti.parameter(5)  # start state parameter
        p_goal = opti.parameter(5)   # goal state parameter

        for k in range(self.K):
            x.append(opti.variable(self.nx[k]))
            u.append(opti.variable(self.nu[k]))
        
        # Add constraints - following fatrop order
        for k in range(self.K):
            # Dynamics constraints
            if k < self.K - 1:
                opti.subject_to(x[k+1] == self.discrete_dynamics(u[k], x[k], k))
            
            # Path constraints
            path_constr = self.path_constraints(u[k], x[k], k, p_start, p_goal, obstacles)
            for constr in path_constr:
                opti.subject_to(constr)
            ng.append(sum([ci.nnz() for ci in path_constr]))  # number of constraints for this step
        
        # Set the objective - following fatrop structure
        J = 0
        for k in range(self.K):
            J += self.cost(u[k], x[k], k, p_goal)
        
        opti.minimize(J)
        
        # Initial guess - use reference trajectory if available, otherwise linear interpolation
        for k in range(self.K):
            # Linear interpolation between start and goal
            for i in range(self.nx[0]):
                opti.set_initial(x[k][i], start_state[i] + (goal_state[i] - start_state[i]) * k / self.N)
            
            # Zero initial guess for controls
            if k < self.N:
                opti.set_initial(u[k], ca.vertcat(0, 0))

        # Set parameter values for code generation
        opti.set_value(p_start, start_state)
        opti.set_value(p_goal, goal_state)
        
        # Solver options - try fatrop first, fallback to ipopt
        opti.solver('fatrop', {
            'structure_detection': 'manual', 
            'nx': self.nx, 
            'nu': self.nu, 
            'ng': ng, 
            'N': self.N, 
            "expand": True, 
            "fatrop.tol": 1e-6,
            "jit": False,
            "fatrop.print_level": 0  # 0 = silent, 5 = verbose
        })

        opti.to_function("opti_func", [p_start, p_goal], [opti.x]).generate('asv.c', {"with_header": True})

        # ipopt with lbfgs and a large memory size
        # opti.solver("ipopt", {"ipopt.hessian_approximation": "limited-memory", "ipopt.tol": 1e-6})
        print("Using FATROP solver")
        
        return opti, x, u
    
    def solve_opt(self, start_state, warm_start_data=None):
        """
        Solve the optimization problem
        """
        
        print("Setting up optimization problem...")
        goal_state = np.array([wp_x,wp_y,wp_psi, 0.0, 0.0])
        opti, x, u = self.setup_optimization_problem(start_state, goal_state)
        
        # Apply warm start if provided
        if warm_start_data is not None:
            print("Applying warm start from previous solution...")
            prev_opti, prev_x, prev_u, prev_sol = warm_start_data
            
            # Set initial guess using previous solution
            for k in range(self.K):
                opti.set_initial(x[k], prev_sol.value(prev_x[k]))
                if k < self.N:
                    opti.set_initial(u[k], prev_sol.value(prev_u[k]))
        
        print("Solving optimization problem...")
        try:
            # Solve the problem
            sol = opti.solve()
            
            # Extract solution
            X_opt = np.zeros((self.nx[0], self.K))
            U_opt = np.zeros((self.nu[0], self.N))
            
            for k in range(self.K):
                X_opt[:, k] = sol.value(x[k])
                if k < self.N:
                    U_opt[:, k] = sol.value(u[k])
            
            cost = sol.value(opti.f)
            
            return X_opt, U_opt, cost, True, (opti, x, u, sol)
            
        except Exception as e:
            print(f"Optimization failed: {e}")
            # Try to get debug solution
            try:
                X_opt = np.zeros((self.nx[0], self.K))
                U_opt = np.zeros((self.nu[0], self.N))
                
                for k in range(self.K):
                    X_opt[:, k] = opti.debug.value(x[k])
                    if k < self.N:
                        U_opt[:, k] = opti.debug.value(u[k])
                
                cost = opti.debug.value(opti.f)
                return X_opt, U_opt, cost, False, None
            except:
                return None, None, None, False, None

def main():
    """
    Main function to run trajectory optimization example
    """
    # Create optimizer
    optimizer = AsvOpt()
    
    print("ASV Optimization")
    print("=" * 60)
    print(f"Time horizon: {optimizer.T:.1f} s")
    print(f"Control intervals: {optimizer.N}")
    print()
    
    # Define start state
    start_state = np.array([0., 0., 0., 0., 0.])
    
    # Define obstacles that the vehicle must avoid while reaching goal
    obstacles = None
    # obstacles = [
    #     (2, 8, 6),
    #     (12, 8, 2.0),
    #     (22, 10, 1.5),
    # ]
    
    print(f"Start state: x={start_state[0]:.1f}, y={start_state[1]:.1f}, ψ={start_state[2]:.1f} rad, u={start_state[3]:.1f} m/s, r={start_state[4]:.1f} rad/s")
    # print(f"Number of obstacles: {len(obstacles)}")
    print()
    
    # Solve reference tracking optimization
    print("ASV GOAL OPTIMIZATION")
    print("-" * 40)
    
    result = optimizer.solve_opt(start_state)
    X_opt, U_opt, cost, success = result[:4]
    
    if X_opt is not None:
        if success:
            print(f"Reference tracking optimization completed successfully!")
        else:
            print(f"Reference tracking optimization did not converge, showing debug solution...")
        
        print(f"Final cost: {cost:.4f}")
        print(f"Final position: x={X_opt[0, -1]:.2f}, y={X_opt[1, -1]:.2f}")
        print(f"Final heading: {X_opt[2, -1]*180/np.pi:.1f}°")
        print(f"Final velocity: {X_opt[3, -1]:.2f} m/s")
        
        # Plot results
        animate_results(optimizer, X_opt, U_opt, obstacles, success)        

if __name__ == "__main__":
    main()
