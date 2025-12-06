from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs, if_else, atan2

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

    # set up states & controls
    x_pos = SX.sym('x_pos')
    y_pos = SX.sym('y_pos')
    psi = SX.sym('psi')
    surge = SX.sym('surge')
    yaw = SX.sym('yaw')
    t = SX.sym('t')  # Spline parameter
    x = vertcat(x_pos, y_pos, psi, surge, yaw, t)

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
    w_along = SX.sym('w_along')
    w_cross = SX.sym('w_cross')
    w_heading = SX.sym('w_heading')
    w_input = SX.sym('w_input')
    w_slack = SX.sym('w_slack')
    w_surge = SX.sym('w_surge')
    w_yaw = SX.sym('w_yaw')
    w_terminal = SX.sym('w_terminal')
    t_la = SX.sym('t_la')

    p = vertcat(a_x, b_x, c_x, d_x, a_y, b_y, c_y, d_y, 
                w_along, w_cross, w_heading, w_input, w_slack, w_surge, w_yaw, w_terminal,
                t_la)

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

    xdot = vertcat(x_dot, y_dot, psi_dot, surge_dot, yaw_dot, t_dot)
    
    # ASV dynamics
    cos_psi = cos(psi)
    sin_psi = sin(psi)

    f_expl = vertcat(
        surge * cos_psi,
        surge * sin_psi,
        yaw,
        ((t_port + t_stbd - (Y_r_dot + N_v_dot)*yaw*yaw
        - (-Xu*surge - Xuu*fabs(surge)*surge)) / (m - X_u_dot)),
        (((t_port - t_stbd) * B / 2 + 
        Nrr*fabs(yaw)*yaw + Nr*yaw) / (Iz - N_r_dot)),
        dt, # Progress along spline
    )

    f_impl = xdot - f_expl

    # Spline evaluation expressions (for cost function)
    # s(t) = [s_x(t), s_y(t)]
    s_x = a_x * t**3 + b_x * t**2 + c_x * t + d_x
    s_y = a_y * t**3 + b_y * t**2 + c_y * t + d_y

    # s(la)
    s_la_x = a_x * t_la**3 + b_x * t_la**2 + c_x * t_la + d_x
    s_la_y = a_y * t_la**3 + b_y * t_la**2 + c_y * t_la + d_y
    
    # s'(t) = [s_x'(t), s_y'(t)]
    s_x_dot = 3 * a_x * t**2 + 2 * b_x * t + c_x
    s_y_dot = 3 * a_y * t**2 + 2 * b_y * t + c_y
    
    # Reference heading from spline tangent
    psi_ref = atan2(s_y_dot, s_x_dot)

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

    # Store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]', '$\\psi$ [rad]', '$u$ [m/s]', '$r$ [rad/s]', '$t$']
    model.u_labels = ['$\\tau_{port}$', '$\\tau_{stbd}$', '$dt$', '$\\sigma_u$']
    model.t_label = '$t$ [s]'

    return model
