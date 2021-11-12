import casadi as cs
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


# 0 sx
# 1 sx with map
# 2 full mx with map

type_sol = 3

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

N = 50

# ====================
# unraveled dimensions
# ====================
q_dim = N * n_q
q_dot_dim = N * n_v
u_dim = (N-1) * (n_v - 6)
f_dim = (N-1) * n_c * n_f

q = cs.SX.sym('q', n_q)
q_dot = cs.SX.sym('q_dot', n_v)

u = cs.SX.sym("actuated_torques", n_v - 6)
tau = cs.vertcat(cs.SX.zeros(6, 1), u)

# forces
f_list = list()
for i in range(n_c):
    f_list.append(cs.SX.sym(f'f{i}', n_f))

contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, f_list))
fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
q_ddot = fd.call(q, q_dot, tau, contact_map)

x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

fs = cs.vertcat(*f_list)
state = cs.vertcat(q, q_dot)
input = cs.vertcat(u, fs)

dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
integrator = RK4(dae, opts=dict(tf=0.01))
# technically should be faster
# integrator = integrator.expand()
int_map = integrator.map(N-1, 'thread', 15)

q_i = cs.MX.sym("q_i", n_q, N)
q_dot_i = cs.MX.sym("q_dot_i", n_v, N)
u_i = cs.MX.sym("u_i", n_v - 6, N-1)

# all the forces
f_i = cs.MX.sym('f_i', n_f * n_c, N-1)

X = cs.vertcat(q_i, q_dot_i)
U = cs.vertcat(u_i, f_i)

X_int = int_map(X[:, :N-1], U)
# starting from node 1
g_multi_shoot = X_int[0] - X[:, 1:]

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

# =====================================================================================
# =====================================================================================
#
mu = 0.8
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

g_v_list = list()
for frame, f in contact_map.items():
    # velocity of each end effector must be zero
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q_i, qdot=q_dot_i)['ee_vel_linear']
    g_v_list.append(v)

#     # friction cones must be satisfied
#     fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
#     g_fc_list.append(), bounds=dict(lb=fc_lb, ub=fc_ub))

f_list = list()
f_list.append(1000 * cs.sumsqr(q_i - q_init))
f_list.append(10e-10 * cs.sumsqr(f_i))
f_list.append(100. * cs.sumsqr(q_dot_i))
# =====================================================================================
# =====================================================================================
w = cs.veccat(q_i, q_dot_i, u_i, f_i)
g = cs.veccat(g_multi_shoot, *g_v_list)
f = sum(f_list)

# ====================
# setting initial condition
# ====================
q0 = np.zeros([n_q, N])
q_dot0 = np.zeros([n_v, N])
u0 = np.zeros([n_v - 6, N-1])
f0 = np.zeros([n_c * n_f, N-1])

# ====================
# lower bound variables
# ====================
q_lbw = -np.inf * np.ones([n_q, N])
q_dot_lbw = -np.inf * np.ones([n_v, N])
u_lbw = -np.inf * np.ones([n_v - 6, N-1])
f_lbw = -np.inf * np.ones([n_c * n_f, N-1])

# ====================
# upper bound variables
# ====================
q_ubw = np.inf * np.ones([n_q, N])
q_dot_ubw = np.inf * np.ones([n_v, N])
u_ubw = np.inf * np.ones([n_v - 6, N-1])
f_ubw = np.inf * np.ones([n_c * n_f, N-1])

# ===============================
# ==== BOUNDS INITIALIZATION ====
# ===============================
# initial guess q
q0[:, 0] = q_init

# zero final velocity
# q_dot_lbw[:, 49] = np.zeros([1, n_v])
# q_dot_ubw[:, 49] = np.zeros([1, n_v])

q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_lbw[:, :] = np.tile(q_min, (50, 1)).T
q_ubw[:, :] = np.tile(q_max, (50, 1)).T

# initial q
q_lbw[:, 0] = q_init
q_ubw[:, 0] = q_init

# q_dot bounds
q_dot_lim = 100. * np.ones(n_v)
q_dot_lbw[:, :] = np.tile(-q_dot_lim, (50, 1)).T
q_dot_ubw[:, :] = np.tile(q_dot_lim, (50, 1)).T

# f bounds
f_lim = 1000. * np.ones(n_c * n_f)
f_lbw[:, :] = np.tile(-f_lim, (N-1, 1)).T
f_ubw[:, :] = np.tile(f_lim, (N-1, 1)).T

f_lbw[2, :] = np.zeros(f_lbw[2, :].shape)
f_lbw[5, :] = np.zeros(f_lbw[5, :].shape)
f_lbw[8, :] = np.zeros(f_lbw[8, :].shape)
f_lbw[11, :] = np.zeros(f_lbw[11, :].shape)

# torques bounds
u_lim = 1000. * np.ones(n_v - 6)
u_lbw[:, :] = np.tile(-u_lim, (N-1, 1)).T
u_ubw[:, :] = np.tile(u_lim, (N-1, 1)).T

# bounds variable
q0_flat = np.reshape(q0, [1, q_dim], order='F')
q_dot0_flat = np.reshape(q_dot0, [1, q_dot_dim], order='F')
u0_flat = np.reshape(u0, [1, u_dim], order='F')
f0_flat = np.reshape(f0, [1, f_dim], order='F')
w0 = np.concatenate((q0_flat, q_dot0_flat, u0_flat, f0_flat), axis=1)

q_lbw_flat = np.reshape(q_lbw, [1, q_dim], order='F')
q_dot_lbw_flat = np.reshape(q_dot_lbw, [1, q_dot_dim], order='F')
u_lbw_flat = np.reshape(u_lbw, [1, u_dim], order='F')
f_lbw_flat = np.reshape(f_lbw, [1, f_dim], order='F')
lbw = np.concatenate((q_lbw_flat, q_dot_lbw_flat, u_lbw_flat, f_lbw_flat), axis=1)

q_ubw_flat = np.reshape(q_ubw, [1, q_dim], order='F')
q_dot_ubw_flat = np.reshape(q_dot_ubw, [1, q_dot_dim], order='F')
u_ubw_flat = np.reshape(u_ubw, [1, u_dim], order='F')
f_ubw_flat = np.reshape(f_ubw, [1, f_dim], order='F')
ubw = np.concatenate((q_ubw_flat, q_dot_ubw_flat, u_ubw_flat, f_ubw_flat), axis=1)

# lower bound constraints
multi_shoot_g_lbw = np.zeros(g_multi_shoot.shape)
g_v_0_lbw = np.zeros(g_v_list[0].shape)
g_v_1_lbw = np.zeros(g_v_list[1].shape)
g_v_2_lbw = np.zeros(g_v_list[2].shape)
g_v_3_lbw = np.zeros(g_v_list[3].shape)

# upper bound constraints
multi_shoot_g_ubw = np.zeros(g_multi_shoot.shape)
g_v_0_ubw = np.zeros(g_v_list[0].shape)
g_v_1_ubw = np.zeros(g_v_list[1].shape)
g_v_2_ubw = np.zeros(g_v_list[2].shape)
g_v_3_ubw = np.zeros(g_v_list[3].shape)

multi_shoot_g_lbw_flat = np.reshape(multi_shoot_g_lbw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')
multi_shoot_g_ubw_flat = np.reshape(multi_shoot_g_ubw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')

g_v_0_lbw_flat = np.reshape(g_v_0_lbw, [1, g_v_0_lbw.shape[0] * g_v_0_lbw.shape[1]], order='F')
g_v_1_lbw_flat = np.reshape(g_v_1_lbw, [1, g_v_1_lbw.shape[0] * g_v_1_lbw.shape[1]], order='F')
g_v_2_lbw_flat = np.reshape(g_v_2_lbw, [1, g_v_2_lbw.shape[0] * g_v_2_lbw.shape[1]], order='F')
g_v_3_lbw_flat = np.reshape(g_v_3_lbw, [1, g_v_3_lbw.shape[0] * g_v_3_lbw.shape[1]], order='F')

g_v_0_ubw_flat = np.reshape(g_v_0_ubw, [1, g_v_0_ubw.shape[0] * g_v_0_ubw.shape[1]], order='F')
g_v_1_ubw_flat = np.reshape(g_v_1_ubw, [1, g_v_1_ubw.shape[0] * g_v_1_ubw.shape[1]], order='F')
g_v_2_ubw_flat = np.reshape(g_v_2_ubw, [1, g_v_2_ubw.shape[0] * g_v_2_ubw.shape[1]], order='F')
g_v_3_ubw_flat = np.reshape(g_v_3_ubw, [1, g_v_3_ubw.shape[0] * g_v_3_ubw.shape[1]], order='F')

lbg = np.concatenate((multi_shoot_g_lbw_flat, g_v_0_lbw_flat, g_v_1_lbw_flat, g_v_2_lbw_flat, g_v_3_lbw_flat), axis=1)
ubg = np.concatenate((multi_shoot_g_ubw_flat, g_v_0_ubw_flat, g_v_1_ubw_flat, g_v_2_ubw_flat, g_v_3_ubw_flat), axis=1)

prob_dict = {'f': f, 'x': w, 'g': cs.vec(g)}

opts = {
        'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}
        # 'verbose': True,
        # banned options:
        # 'ipopt.hessian_approximation': 'limited-memory',
        # 'expand': True,

# create solver from prob
tic = time.time()
solver = cs.nlpsol('solver', 'ipopt', prob_dict, opts)
toc = time.time()
print('time elapsed:', toc - tic)

# print(ubw)
# exit()
solution = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

print(solution)
ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')
ms.store(dict(a=np.array(solution['x'])))
