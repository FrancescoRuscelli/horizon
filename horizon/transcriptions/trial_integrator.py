import casadi as cs

def RK4(dae, opts=None, casadi_type=cs.SX):
    """
        Implements an integration scheme based on 4th-order Runge-Kutta integration:
        Args:
            dae: a dictionary containing
                'x': state
                'p': control
                'ode': a function of the state and control returning the derivative of the state
                'quad': quadrature term
            opts: a dictionary containing 'tf': integration time
            NOTE: this term can be used to take into account also a final time to optimize
            casadi_type: 'SX' or 'MX'

        Returns: Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf']) or
             Function('F_RK', [X0_RK, U_RK, DT_RK], [X_RK, Q_RK], ['x0', 'p', 'time'], ['xf', 'qf'])
            which given in input the actual state X0_RK and control U_RK returns the integrated state X_RK and quadrature term Q_RK

    """
    if opts is None:
        opts = dict()

    x = dae['x']
    qddot = dae['p']
    xdot = dae['ode']
    L = dae['quad']

    f_RK = cs.Function('f_RK', [x, qddot], [xdot, L])

    nx = x.size1()
    nv = qddot.size1()

    X0_RK = casadi_type.sym('X0_RK', nx)
    U_RK = casadi_type.sym('U_RK', nv)

    if 'tf' in opts:
        DT_RK = opts['tf']
    else:
        DT_RK = casadi_type.sym('DT_RK', 1)

    X_RK = X0_RK
    Q_RK = 0

    k1, k1_q = f_RK(X_RK, U_RK)
    k2, k2_q = f_RK(X_RK + DT_RK / 2. * k1, U_RK)
    k3, k3_q = f_RK(X_RK + DT_RK / 2. * k2, U_RK)
    k4, k4_q = f_RK(X_RK + DT_RK * k3, U_RK)

    X_RK = X_RK + DT_RK / 6. * (k1 + 2. * k2 + 2. * k3 + k4)
    Q_RK = Q_RK + DT_RK / 6. * (k1_q + 2. * k2_q + 2. * k3_q + k4_q)

    if 'tf' in opts:
        f = cs.Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf'])
    else:
        f = cs.Function('F_RK', [X0_RK, U_RK, DT_RK], [X_RK, Q_RK], ['x0', 'p', 'time'], ['xf', 'qf'])
    return f

if __name__ == '__main__':
    dae = dict()
    dae['x'] = cs.SX.sym('x', 1)
    dae['p'] = cs.SX.sym('p', 1)
    dae['ode'] = cs.SX.sym('xdot', 1)
    dae['quad'] = 0  # note: we don't use the quadrature fn here

    integrator = RK4(dae=dae, casadi_type=cs.SX)

    x0 = cs.SX.sym('x0', 1)
    u0 = cs.SX.sym('p0', 1)
    # dt = cs.SX.sym('dt', 1)
    dt = 1
    int = integrator(x0, u0, dt)

    print(int)
