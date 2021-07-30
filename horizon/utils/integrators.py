import casadi as cs
import horizon.problem as prb
import numpy as np

def EULER(dae, opts=None, casadi_type=cs.SX):
    """
    Implements an integration scheme based on Euler integration (http://www.cmth.ph.ic.ac.uk/people/a.mackinnon/Lectures/compphys/node4.html):
        x = x + dt*xdot(x,u)
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

    X_RK = X_RK + DT_RK * k1
    Q_RK = Q_RK + DT_RK * k1_q

    if 'tf' in opts:
        f = cs.Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf'])
    else:
        f = cs.Function('F_RK', [X0_RK, U_RK, DT_RK], [X_RK, Q_RK], ['x0', 'p', 'time'], ['xf', 'qf'])
    return f

def RK2(dae, opts=None, casadi_type=cs.SX):
    """
        Implements an integration scheme based on 2nd-order Runge-Kutta integration (http://www.cmth.ph.ic.ac.uk/people/a.mackinnon/Lectures/compphys/node11.html):
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

    X_RK = X_RK + DT_RK * k2
    Q_RK = Q_RK + DT_RK * k2_q

    if 'tf' in opts:
        f = cs.Function('F_RK', [X0_RK, U_RK], [X_RK, Q_RK], ['x0', 'p'], ['xf', 'qf'])
    else:
        f = cs.Function('F_RK', [X0_RK, U_RK, DT_RK], [X_RK, Q_RK], ['x0', 'p', 'time'], ['xf', 'qf'])
    return f

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


def LEAPFROG(dae, opts=None, casadi_type=cs.SX):
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
    X0_PREV_RK = casadi_type.sym('X0_PREV_RK', nx)
    U_RK = casadi_type.sym('U_RK', nv)

    if 'tf' in opts:
        DT_RK = opts['tf']
    else:
        DT_RK = casadi_type.sym('DT_RK', 1)

    Q_RK = 0

    k1, k1_q = f_RK(X0_RK, U_RK)

    X_RK = X0_PREV_RK + 2. * DT_RK * k1
    X_PREV_RK = X0_RK

    if 'tf' in opts:
        f = cs.Function('F_RK', [X0_RK, X0_PREV_RK, U_RK], [X_RK, X_PREV_RK, Q_RK], ['x0', 'x0_prev', 'p'], ['xf', 'xf_prev', 'qf'])
    else:
        f = cs.Function('F_RK', [X0_RK, X0_PREV_RK, U_RK, DT_RK], [X_RK, X_PREV_RK, Q_RK], ['x0', 'x0_prev', 'p', 'time'], ['xf', 'xf_prev', 'qf'])
    return f

def make_direct_collocation(prob: prb.Problem, 
                            degree: int, 
                            dt: float) -> None:

    """
    Add auxiliary input variables and constraints that implement
    the Direct Collocation transcription scheme. This could also be
    seen as multiple shooting with a different integration strategy
    that is based on approximating polynomials.

    Parameters:
        prob (prb.Problem): the horizon problem
        x (cs.SX): the state variable
        x_prev (cs.SX): the state variable at the previous node
        xdot (cs.SX): the system dynamics
        degree (int): degree of approximating polynomial
        dt (float|StateVariable): discretization interval (cal be a control input)
    """    

    # handle input
    d = degree

    # some constants
    N = prob.getNNodes() - 1

    # get dynamics, state at current and previous iter
    xdot = prob.getDynamics()
    x = prob.getState().getVars()
    x_prev = prob.getState().getVarOffset(-1).getVars()

    # create additional variables (states at collocation points)
    collo = [prob.createInputVariable(f'collo_x_{i}', dim=x.shape[0]) for i in range(d)]
    collo_prev = [var.getVarOffset(-1) for var in collo]
    collo.insert(0, x)  # append state at beginning of interval
    collo_prev.insert(0, x_prev) # append state at beginning of previous interval

    # generate coefficients from magic polynomial
    collo_points = cs.collocation_points(d, 'legendre')  # or 'radau'

    # roots of basis polynomials (0 + collocation points)
    tau_root = np.append(0, collo_points)

    # coefficients of the collocation equation (i.e., dynamics)
    # note: first row is not used (dynamics constraint at beginning of interval),
    # as dynamics is only enforced at collocation points (excluding t = 0)
    C = np.zeros((d+1, d+1))  

    # coefficients of the continuity equation
    D = np.zeros(d+1)

    # coefficients of the quadrature function
    B = np.zeros(d+1)

    # loop over polynomial basis
    for j in range(d+1):

        # construct j-th Lagrange polynomial, s.t. pj(tj) = 1 && pj(ti) = 0 if j != i
        # note: these d+1 polynomials form a basis for all d-th degree polynomials
        p = np.poly1d([1])
        for r in range(d+1):
            if r != j:
                p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j]-tau_root[r])

        # evaluate the polynomial at the final time to get the coefficients of the continuity equation
        D[j] = p(1.0)

        # evaluate the time derivative of the polynomial at all collocation points to get the coefficients 
        # of the collocation equation
        pder = np.polyder(p)
        for r in range(d + 1):
            C[r, j] = pder(tau_root[r])  # note: r = 0 is actually not used!

        # evaluate the integral of the polynomial to get the coefficients of the quadrature function
        pint = np.polyint(p)
        B[j] = pint(1.0)

    
    # continuity constraint
    x_prev_int = 0  # this will be the previous state, integrated according to the previous polynomial

    for i in range(d+1):
        # loop on basis polynomials
        x_prev_int += collo_prev[i] * D[i]

    cc = prob.createConstraint('collo_continuity', x_prev_int - x, nodes=range(1, N+1))

    # dynamics constraint (one per collocation point)
    for i in range(d):
        # loop on collocation points
        xder = 0
        for j in range(d+1):
            # loop on basis polynomials
            xder += C[i+1, j]*collo[j]
        dyn = prob.createConstraint(f'collo_dyn_{i}', xder - xdot*dt, nodes=range(N))



