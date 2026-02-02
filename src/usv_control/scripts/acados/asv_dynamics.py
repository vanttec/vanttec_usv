from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs, if_else, atan2
import casadi as ca

def export_asv_model() -> AcadosModel:
    """
    ASV with spline parameter 't' for path tracking.
    
    States: [x_pos, y_pos, psi, surge, yaw, t]
    Controls: [t_port, t_stbd, dt]
    Parameters: [a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y]
    """
    
    model_name = 'asv_dynamics'

    # ASV constants
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
    obs_n = 3

    # set up states & controls
    x_pos = SX.sym('x_pos')
    y_pos = SX.sym('y_pos')
    psi = SX.sym('psi')
    surge = SX.sym('surge')
    yaw = SX.sym('yaw')
    t = SX.sym('t')  # Spline parameter
    obs_states = [] # obstacles
    for i in range(obs_n):
        obs_states.append(SX.sym(f'obs_x_{i}'))
        obs_states.append(SX.sym(f'obs_y_{i}'))
    x = vertcat(x_pos, y_pos, psi, surge, yaw, t, *obs_states)

    t_port = SX.sym('t_port')
    t_stbd = SX.sym('t_stbd')
    dt = SX.sym('dt')  # Progress along spline
    slack_u = SX.sym('slack_u') # Slack control var. for surge
    u = vertcat(t_port, t_stbd, dt, slack_u)

    # Spline coefficients as parameters
    # s_x(t) = a_x*t^3 + b_x*t^2 + c_x*t + d_x
    # s_y(t) = a_y*t^3 + b_y*t^2 + c_y*t + d_y
    a_x = SX.sym('a_x')
    b_x = SX.sym('b_x')
    c_x = SX.sym('c_x')
    d_x = SX.sym('d_x')
    a_y = SX.sym('a_y')
    b_y = SX.sym('b_y')
    c_y = SX.sym('c_y')
    d_y = SX.sym('d_y')

    # Next spline's coefficients
    a2_x = SX.sym('a2_x')
    b2_x = SX.sym('b2_x')
    c2_x = SX.sym('c2_x')
    d2_x = SX.sym('d2_x')
    a2_y = SX.sym('a2_y')
    b2_y = SX.sym('b2_y')
    c2_y = SX.sym('c2_y')
    d2_y = SX.sym('d2_y')

    w_along = SX.sym('w_along')
    w_cross = SX.sym('w_cross')
    w_heading = SX.sym('w_heading')
    w_input = SX.sym('w_input')
    w_slack = SX.sym('w_slack')
    w_surge = SX.sym('w_surge')
    w_yaw = SX.sym('w_yaw')
    w_terminal = SX.sym('w_terminal')
    w_avoidance = SX.sym('w_avoidance')
    t_la = SX.sym('t_la')
    in_last_s = SX.sym('in_last_s')
    spline_ceil = SX.sym('spline_ceil')
    
    # Obstacle velocities (for dynamic obstacles)
    obs_velocities = []
    for i in range(obs_n):
        obs_velocities.append(SX.sym(f'obs_vx_{i}'))
        obs_velocities.append(SX.sym(f'obs_vy_{i}'))
    
    p = vertcat(
            a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y, # 0 - 7
            a2_x, b2_x, c2_x, d2_x, a2_y, b2_y, c2_y, d2_y, # 8 - 15 
            w_along, w_cross, w_heading, w_input, w_slack, w_surge, w_yaw, w_terminal, w_avoidance, # 16 - 24
            t_la, in_last_s, spline_ceil, *obs_velocities) # 25 - 27+obs_n*2

    # state-dependent parameters for ASV
    Xu = if_else(surge > 1.2, 64.55, -25.0)
    Xuu = if_else(surge > 1.2, -70.92, 0.0)
    Nr = (-0.52)*fabs(surge)

    # xdot
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    psi_dot = SX.sym('psi_dot')
    surge_dot = SX.sym('surge_dot')
    yaw_dot = SX.sym('yaw_dot')
    t_dot = SX.sym('t_dot')
    # Obstacle derivatives
    obs_dots = []
    for i in range(obs_n):
        obs_dots.append(SX.sym(f'obs_x_dot_{i}'))
        obs_dots.append(SX.sym(f'obs_y_dot_{i}'))

    xdot = vertcat(x_dot, y_dot, psi_dot, surge_dot, yaw_dot, t_dot, *obs_dots)
    
    # ASV dynamics
    cos_psi = cos(psi)
    sin_psi = sin(psi)
    # Obstacle dynamics (controlled by parameters)
    obs_dynamics = []
    for i in range(obs_n):
        obs_dynamics.append(obs_velocities[2*i])      # dx/dt = vx
        obs_dynamics.append(obs_velocities[2*i + 1])  # dy/dt = vy

    f_expl = vertcat(
        surge * cos_psi,
        surge * sin_psi,
        yaw,
        ((t_port + t_stbd - (Y_r_dot + N_v_dot)*yaw*yaw
        - (-Xu*surge - Xuu*fabs(surge)*surge)) / (m - X_u_dot)),
        (((t_port - t_stbd) * B / 2 + 
        Nrr*fabs(yaw)*yaw + Nr*yaw) / (Iz - N_r_dot)),
        dt, # Progress along spline
        *obs_dynamics
    )

    f_impl = xdot - f_expl

    # Spline evaluation expressions (for cost function)
    # s(t) = [s_x(t), s_y(t)]
    t_mod = ca.fmod(t, 1.0)
    t_mod = ca.if_else(ca.logic_and(t_mod < 1e-6, t > 0.1), 1.0, t_mod)
    t_mod = ca.if_else(ca.logic_and(t > spline_ceil, in_last_s), 1.0, t_mod)
    t_la_mod = ca.fmod(t_la, 1.0)
    t_la_mod = ca.if_else(ca.logic_and(t_la_mod < 1e-6, t_la > 0.1), 1.0, t_la_mod)
    t_la_mod = ca.if_else(ca.logic_and(t_la > spline_ceil, in_last_s), 1.0, t_la_mod)

    s_x = a_x * t_mod**3 + b_x * t_mod**2 + c_x * t_mod + d_x
    s_y = a_y * t_mod**3 + b_y * t_mod**2 + c_y * t_mod + d_y

    # s(la)
    s_la_x = a_x * t_la_mod**3 + b_x * t_la_mod**2 + c_x * t_la_mod + d_x
    s_la_y = a_y * t_la_mod**3 + b_y * t_la_mod**2 + c_y * t_la_mod + d_y
    
    # s'(t) = [s_x'(t), s_y'(t)]
    s_x_dot = 3 * a_x * t_mod**2 + 2 * b_x * t_mod + c_x
    s_y_dot = 3 * a_y * t_mod**2 + 2 * b_y * t_mod + c_y
    
    # Reference heading from spline tangent
    psi_ref = atan2(s_y_dot, s_x_dot)

    # Next Spline evaluation expressions (for cost function)
    # s(t) = [s_x(t), s_y(t)]
    s2_x = a2_x * t_mod**3 + b2_x * t_mod**2 + c2_x * t_mod + d2_x
    s2_y = a2_y * t_mod**3 + b2_y * t_mod**2 + c2_y * t_mod + d2_y

    # s(la)
    s2_la_x = a2_x * t_la_mod**3 + b2_x * t_la_mod**2 + c2_x * t_la_mod + d2_x
    s2_la_y = a2_y * t_la_mod**3 + b2_y * t_la_mod**2 + c2_y * t_la_mod + d2_y
    
    # s'(t) = [s2_x'(t), s2_y'(t)]
    s2_x_dot = 3 * a2_x * t_mod**2 + 2 * b2_x * t_mod + c2_x
    s2_y_dot = 3 * a2_y * t_mod**2 + 2 * b2_y * t_mod + c2_y
    
    # Reference heading from spline tangent
    psi2_ref = atan2(s2_y_dot, s2_x_dot)

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name

    # Store additional expressions for cost function
    model.s_x = s_x
    model.s_y = s_y
    model.s_la_x = s_la_x
    model.s_la_y = s_la_y
    model.psi_ref = psi_ref
    model.s2_x = s2_x
    model.s2_y = s2_y
    model.s2_la_x = s2_la_x
    model.s2_la_y = s2_la_y
    model.psi2_ref = psi2_ref
    model.obs_n = obs_n

    # Store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$\\psi$ [rad]', '$u$ [m/s]', '$r$ [rad/s]', '$t$']
    model.u_labels = ['$\\tau_{port}$', '$\\tau_{stbd}$', '$dt$', '$\\sigma_u$']
    model.t_label = '$t$ [s]'

    return model
