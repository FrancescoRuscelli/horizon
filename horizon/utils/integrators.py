import casadi as cs
import horizon.problem as prb
import casadi as cs 
import numpy as np

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


def make_direct_collocation(prob: prb.Problem, 
                            x: cs.SX, 
                            x_prev: cs.SX,
                            xdot: cs.SX, 
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
    nx = x.shape[0]
    N = prob.getNNodes() - 1

    # create additional variables (states at collocation points)
    collo = [prob.createInputVariable(f'collo_x_{i}', dim=x.shape[0]) for i in range(d)]
    collo.insert(0, x)  # append state at beginning of interval
    collo_prev = [prob.createInputVariable(f'collo_x_{i}', dim=x.shape[0], prev_nodes=-1) for i in range(d)]
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
    cc.setBounds(lb=np.zeros(nx), ub=np.zeros(nx))

    # dynamics constraint (one per collocation point)
    for i in range(d):
        # loop on collocation points
        xder = 0
        for j in range(d+1):
            # loop on basis polynomials
            xder += C[i+1, j]*collo[j]
        dyn = prob.createConstraint(f'collo_dyn_{i}', xder - xdot*dt, nodes=range(N))
        dyn.setBounds(lb=np.zeros(nx), ub=np.zeros(nx))