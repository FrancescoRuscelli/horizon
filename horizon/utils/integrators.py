import casadi as cs

def RK4(dae, opts, casadi_type):
    x = dae['x']
    qddot = dae['p']
    xdot = dae['ode']
    L = dae['quad']

    f_RK = cs.Function('f_RK', [x, qddot], [xdot, L])

    nx = x.size1()
    nv = qddot.size1()

    if casadi_type == 'MX':
        X0_RK = cs.MX.sym('X0_RK', nx)
        U_RK = cs.MX.sym('U_RK', nv)
    elif casadi_type == 'SX':
        X0_RK = cs.SX.sym('X0_RK', nx)
        U_RK = cs.SX.sym('U_RK', nv)
    else:
        raise Exception('Input casadi_type can be only SX or MX!')

    DT_RK = opts['tf']
    X_RK = X0_RK
    Q_RK = 0

    k1, k1_q = f_RK(X_RK, U_RK)
    k2, k2_q = f_RK(X_RK + DT_RK / 2. * k1, U_RK)
    k3, k3_q = f_RK(X_RK + DT_RK / 2. * k2, U_RK)
    k4, k4_q = f_RK(X_RK + DT_RK * k3, U_RK)

    X_RK = X_RK + DT_RK / 6. * (k1 + 2. * k2 + 2. * k3 + k4)
    Q_RK = Q_RK + DT_RK / 6. * (k1_q + 2. * k2_q + 2. * k3_q + k4_q)

    return cs.Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf'])