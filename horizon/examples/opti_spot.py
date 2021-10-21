import casadi as cs
import kiwisolver

from horizon.utils import utils, kin_dyn, mat_storer
import numpy as np
import time
import os
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn


def RK4(dae, opts=None, casadi_type=cs.SX):
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


urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

N = 100
N_states = N + 1
N_control = N
dt = 0.01

node_start_step = 30
node_end_step = N
# ====================
# unraveled dimensions
# ====================

q_dim = N_states * n_q
q_dot_dim = N_states * n_v
q_ddot_dim = N_control * n_v
f_dim = N_control * n_c * n_f

q = cs.SX.sym("q", n_q)
q_dot = cs.SX.sym("q_dot", n_v)
q_ddot = cs.SX.sym("q_ddot", n_v)

# forces
f_list = list()
for i in range(n_c):
    f_list.append(cs.SX.sym(f"f{i}", n_f))

# SET CONTACTS MAP
contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, f_list))


g_tau = kin_dyn.InverseDynamics(kindyn,
                              contact_map.keys(),
                              cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot, q_ddot, contact_map)

fs = cs.vertcat(*f_list)
state = cs.vertcat(q, q_dot)
input = cs.vertcat(q_ddot, fs)

x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
integrator = RK4(dae, opts=dict(tf=dt))

# technically should be faster
# integrator = integrator.expand()
int_map = integrator.map(N_control, 'thread', 15)

opti = cs.Opti()

q_i = opti.variable(n_q, N_states)
q_dot_i = opti.variable(n_v, N_states)

q_ddot_i = opti.variable(n_v, N_control)
f_i = opti.variable(n_f * n_c, N_control)


X = cs.vertcat(q_i, q_dot_i)
U = cs.vertcat(q_ddot_i, f_i)
X_int = int_map(X[:, :N], U) # because it's N+1
# starting from node 1
g_multi_shoot = X_int[0] - X[:, 1:]

mu = 1
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

contact_map_i = dict(lf_foot=f_i[0:3, :],
                     rf_foot=f_i[3:6, :],
                     lh_foot=f_i[6:9, :],
                     rh_foot=f_i[9:12, :])

g_tau_i = kin_dyn.InverseDynamicsMap(N, kindyn,
                              contact_map.keys(),
                              cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_i[:, :N], q_dot_i[:, :N], q_ddot_i, contact_map_i)


tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
                    1000., 1000., 1000.,  # Contact 1
                    1000., 1000., 1000.,  # Contact 2
                    1000., 1000., 1000.,  # Contact 3
                    1000., 1000., 1000.])  # Contact 4

opti.subject_to(opti.bounded(-tau_lim, g_tau_i, tau_lim))
active_leg = ['lf_foot']

opti.subject_to(g_multi_shoot == 0.)

active_slice = slice(node_start_step, node_end_step)

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

i = 0
for frame, f in contact_map.items():
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q_i, qdot=q_dot_i)['ee_vel_linear']
    if frame in active_leg:
        # can move ee
        opti.subject_to(v[:, 0:node_start_step] == 0)
    else:
        # 0 vel for ee
        opti.subject_to(v == 0)


    if frame in active_leg:
        g_fc = kin_dyn.linearized_friction_cone_map(f_i[3 * i:3 * i + 3, 0:node_start_step], mu, R, node_start_step)
    else:
        g_fc = kin_dyn.linearized_friction_cone_map(f_i[3 * i:3 * i + 3, :], mu, R, N_control)

    opti.subject_to(cs.vec(g_fc) <= 0)


    i += 1
    if frame in active_leg:
        FK = cs.Function.deserialize(kindyn.fk(frame))
        p_i = FK(q=q_i)['ee_pos']
        p_start = FK(q=q_init)['ee_pos']
        p_goal = p_start + [0., 0., 0.2]

        opti.subject_to(p_i[:, node_start_step + 20] - p_goal == 0)

# no force during lift of one leg
opti.subject_to(f_i[0:3, node_start_step:] == 0)

opti.minimize(1e-2 * cs.sumsqr(f_i[0:3, :]))
opti.minimize(1e-2 * cs.sumsqr(f_i[3:6, :]))
opti.minimize(1e-2 * cs.sumsqr(f_i[6:9, :]))
opti.minimize(1e-2 * cs.sumsqr(f_i[9:12, :]))

opti.minimize(1e3 * cs.sumsqr(q_dot_i))

# ===============================
# ==== BOUNDS INITIALIZATION ====
# ===============================
# initial guess q
opti.set_initial(q_i[:, 0], q_init)

q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

opti.subject_to(q_i[:, 0] == q_init)
opti.subject_to(opti.bounded(q_min, q_i, q_max))

# q_dot bounds
q_dot_lim = 100. * np.ones(n_v)
opti.subject_to(opti.bounded(-q_dot_lim, q_dot_i, q_dot_lim))
opti.subject_to(q_dot_i[:, 0] == 0)
opti.subject_to(q_dot_i[:, N] == 0)

# q_ddot bounds
q_ddot_lim = 100. * np.ones(n_v)
opti.subject_to(opti.bounded(-q_ddot_lim, q_ddot_i, q_ddot_lim))


# f bounds
f_lim = 1000. * np.ones(n_c * n_f)
opti.subject_to(opti.bounded(-f_lim, f_i, f_lim))
opti.subject_to(opti.bounded(0, f_i[2, :], 1000))
opti.subject_to(opti.bounded(0, f_i[5, :], 1000))
opti.subject_to(opti.bounded(0, f_i[8, :], 1000))
opti.subject_to(opti.bounded(0, f_i[11, :], 1000))

s_opts = {
           'ipopt.tol': 0.001,
           'ipopt.constr_viol_tol': 0.001,
           'ipopt.max_iter': 2000,
           'ipopt.linear_solver': 'ma57'}
        # 'verbose': True,
        # banned options:
        # 'ipopt.hessian_approximation': 'limited-memory',
        # 'expand': True,


opti.solver('ipopt', s_opts)

tic = time.time()
sol = opti.solve()
toc = time.time()
print('time elapsed solving:', toc - tic)


solution = dict()
solution['q_i'] = sol.value(q_i)
solution['q_dot_i'] = sol.value(q_dot_i)
solution['q_ddot_i'] = sol.value(q_ddot_i)
solution['f_i'] = sol.value(f_i)


ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')

ms.store(dict(a=solution))
