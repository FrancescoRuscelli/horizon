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
# ====================
# unraveled dimensions
# ====================
q_dim = N_states * n_q
q_dot_dim = N_states * n_v
q_ddot_dim = N_control * n_v
f_dim = N_control * n_c * n_f

q = cs.SX.sym('q', n_q)
q_dot = cs.SX.sym('q_dot', n_v)
q_ddot = cs.SX.sym('q_ddot', n_v)

# forces
f_list = list()
for i in range(n_c):
    f_list.append(cs.SX.sym(f'f{i}', n_f))

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

q_i = cs.MX.sym("q_i", n_q, N_states)
q_dot_i = cs.MX.sym("q_dot_i", n_v, N_states)

q_ddot_i = cs.MX.sym("q_dot_i", n_v, N_control)
f_i = cs.MX.sym('f_i', n_f * n_c, N_control)


X = cs.vertcat(q_i, q_dot_i)
U = cs.vertcat(q_ddot_i, f_i)

X_int = int_map(X[:, :N], U) # because it's N+1
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
g_fc_list = list()
g_no_force_list = list()
# active_leg = slice(0,3)


contact_map_i = dict(lf_foot=f_i[0:3, :],
                     rf_foot=f_i[3:6, :],
                     lh_foot=f_i[6:9, :],
                     rh_foot=f_i[9:12, :])

g_tau_i = kin_dyn.InverseDynamicsMap(N, kindyn,
                              contact_map.keys(),
                              cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_i[:, :N], q_dot_i[:, :N], q_ddot_i, contact_map_i)

active_leg = 'lf_foot'
v_dict = dict()
for frame, f in contact_map.items():
    # velocity of each end effector must be zero
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q_i, qdot=q_dot_i)['ee_vel_linear']
    v_dict[frame] = v

active_slice = slice(30, 85)

for name, elem in v_dict.items():
    if name == active_leg:
        # can move ee
        g_v_list.append(elem[0:30])
        # g_v_list.append(elem[85:])
    else:
        # 0 vel for ee
        g_v_list.append(elem)

# friction cones must be satisfied
# fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f_i, mu, R)
# g_fc_list.append(fc)

FK = cs.Function.deserialize(kindyn.fk(active_leg))
p_i = FK(q=q_i)['ee_pos']
p_start = FK(q=q_init)['ee_pos']
p_goal = p_start + [0., 0., 0.2]

# no force during lift of one leg
g_no_force_list.append(f_i[0:3, 30:])

g_lift = p_i[:, 40] - p_goal
# g_land = p_i[:, 90] - p_start
# prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step + 2)

# prb.createConstraint(f"lift_{frame}_leg", p - p_goal, nodes=25)
# prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step + 2)

f_list = list()
# f_list.append(1000 * cs.sumsqr(q_i - q_init))
f_list.append(10e-3 * cs.sumsqr(f_i))
# f_list.append(10e-3 * cs.sumsqr(f_i[0:3, :]))
# f_list.append(10e-3 * cs.sumsqr(f_i[3:6, :]))
# f_list.append(10e-3 * cs.sumsqr(f_i[6:9, :]))
# f_list.append(10e-3 * cs.sumsqr(f_i[9:12, :]))
f_list.append(1000. * cs.sumsqr(q_dot_i))
# =====================================================================================
# =====================================================================================

# ====================
# setting initial condition
# ====================
q0 = np.zeros([n_q, N_states])
q_dot0 = np.zeros([n_v, N_states])
q_ddot0 = np.zeros([n_v, N_control])
f0 = np.zeros([n_c * n_f, N_control])

# ====================
# lower bound variables
# ====================
q_lbw = -np.inf * np.ones([n_q, N_states])
q_dot_lbw = -np.inf * np.ones([n_v, N_states])
q_ddot_lbw = -np.inf * np.ones([n_v, N_control])
f_lbw = -np.inf * np.ones([n_c * n_f, N_control])

# ====================
# upper bound variables
# ====================
q_ubw = np.inf * np.ones([n_q, N_states])
q_dot_ubw = np.inf * np.ones([n_v, N_states])
q_ddot_ubw = np.inf * np.ones([n_v, N_control])
f_ubw = np.inf * np.ones([n_c * n_f, N_control])

# ===============================
# ==== BOUNDS INITIALIZATION ====
# ===============================
# initial guess q
q0[:, 0] = q_init

# zero final velocity
q_dot_lbw[:, 49] = np.zeros([1, n_v])
q_dot_ubw[:, 49] = np.zeros([1, n_v])

q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_lbw[:, :] = np.tile(q_min, (N_states, 1)).T
q_ubw[:, :] = np.tile(q_max, (N_states, 1)).T

# initial q
q_lbw[:, 0] = q_init
q_ubw[:, 0] = q_init

# q_dot bounds
q_dot_lim = 100. * np.ones(n_v)
q_dot_lbw[:, :] = np.tile(-q_dot_lim, (N_states, 1)).T
q_dot_ubw[:, :] = np.tile(q_dot_lim, (N_states, 1)).T

# q_dot bounds
q_ddot_lim = 1000. * np.ones(n_v)
q_ddot_lbw[:, :] = np.tile(-q_ddot_lim, (N_control, 1)).T
q_ddot_ubw[:, :] = np.tile(q_ddot_lim, (N_control, 1)).T

# f bounds
f_lim = 1000. * np.ones(n_c * n_f)
f_lbw[:, :] = np.tile(-f_lim, (N_control, 1)).T
f_ubw[:, :] = np.tile(f_lim, (N_control, 1)).T

f_lbw[2, :] = np.zeros(f_lbw[2, :].shape)
f_lbw[5, :] = np.zeros(f_lbw[5, :].shape)
f_lbw[8, :] = np.zeros(f_lbw[8, :].shape)
f_lbw[11, :] = np.zeros(f_lbw[11, :].shape)

# reshape stuff
q0_flat = np.reshape(q0, [1, q_dim], order='F')
q_dot0_flat = np.reshape(q_dot0, [1, q_dot_dim], order='F')
q_ddot0_flat = np.reshape(q_ddot0, [1, q_ddot_dim], order='F')
f0_flat = np.reshape(f0, [1, f_dim], order='F')
w0 = np.concatenate((q0_flat, q_dot0_flat, q_ddot0_flat, f0_flat), axis=1)

q_lbw_flat = np.reshape(q_lbw, [1, q_dim], order='F')
q_dot_lbw_flat = np.reshape(q_dot_lbw, [1, q_dot_dim], order='F')
q_ddot_lbw_flat = np.reshape(q_ddot_lbw, [1, q_ddot_dim], order='F')
f_lbw_flat = np.reshape(f_lbw, [1, f_dim], order='F')
lbw = np.concatenate((q_lbw_flat, q_dot_lbw_flat, q_ddot_lbw_flat, f_lbw_flat), axis=1)

q_ubw_flat = np.reshape(q_ubw, [1, q_dim], order='F')
q_dot_ubw_flat = np.reshape(q_dot_ubw, [1, q_dot_dim], order='F')
q_ddot_ubw_flat = np.reshape(q_ddot_ubw, [1, q_ddot_dim], order='F')
f_ubw_flat = np.reshape(f_ubw, [1, f_dim], order='F')
ubw = np.concatenate((q_ubw_flat, q_dot_ubw_flat, q_ddot_ubw_flat, f_ubw_flat), axis=1)

# lower bound constraints
tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
                    1000., 1000., 1000.,  # Contact 1
                    1000., 1000., 1000.,  # Contact 2
                    1000., 1000., 1000.,  # Contact 3
                    1000., 1000., 1000.])  # Contact 4

tau_g_lbw = np.tile(-tau_lim, (N_control, 1)).T
multi_shoot_g_lbw = np.zeros(g_multi_shoot.shape)
g_lift_lbw = np.zeros(g_lift.shape)
# g_land_lbw = np.zeros(g_land.shape)

g_v_lbw = list()
for elem in g_v_list:
    g_v_lbw.append(np.zeros(elem.shape))

g_v_ubw = list()
for elem in g_v_list:
    g_v_ubw.append(np.zeros(elem.shape))

g_no_force_lbw = list()
for elem in g_no_force_list:
    g_no_force_lbw.append(np.zeros(elem.shape))

g_no_force_lbw_flat = list()
for elem in g_no_force_lbw:
    g_no_force_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

# g_fc_lbw = fc_lb
# print(fc_lb)
# exit()
# upper bound constraints
tau_g_ubw = np.tile(tau_lim, (N_control, 1)).T
multi_shoot_g_ubw = np.zeros(g_multi_shoot.shape)
g_lift_ubw = np.zeros(g_lift.shape)
# g_land_ubw = np.zeros(g_land.shape)

tau_g_lbw = np.reshape(tau_g_lbw, [1, g_tau_i.shape[0] * g_tau_i.shape[1]], order='F')
tau_g_ubw = np.reshape(tau_g_ubw, [1, g_tau_i.shape[0] * g_tau_i.shape[1]], order='F')

multi_shoot_g_lbw_flat = np.reshape(multi_shoot_g_lbw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')
multi_shoot_g_ubw_flat = np.reshape(multi_shoot_g_ubw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')

g_lift_lbw_flat = np.reshape(g_lift_lbw, [1, g_lift.shape[0] * g_lift.shape[1]], order='F')
g_lift_ubw_flat = np.reshape(g_lift_ubw, [1, g_lift.shape[0] * g_lift.shape[1]], order='F')

# g_land_lbw_flat = np.reshape(g_lift_lbw, [1, g_land.shape[0] * g_land.shape[1]], order='F')
# g_land_ubw_flat = np.reshape(g_lift_ubw, [1, g_land.shape[0] * g_land.shape[1]], order='F')

g_v_lbw_flat = list()
for elem in g_v_lbw:
    g_v_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_v_ubw_flat = list()
for elem in g_v_ubw:
    g_v_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))
#
g_no_force_ubw = list()
for elem in g_no_force_list:
    g_no_force_ubw.append(np.zeros(elem.shape))

g_no_force_ubw_flat = list()
for elem in g_no_force_ubw:
    g_no_force_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

w = cs.veccat(q_i, q_dot_i, q_ddot_i, f_i)
g = cs.veccat(g_multi_shoot, g_tau_i, *g_v_list, *g_no_force_list, g_lift) # *g_no_force_list # g_land
f = sum(f_list)
lbg = np.concatenate((multi_shoot_g_lbw_flat, tau_g_lbw, *g_v_lbw_flat, *g_no_force_lbw_flat, g_lift_lbw_flat), axis=1)  #*g_no_force_lbw_flat,  g_land_lbw_flat
ubg = np.concatenate((multi_shoot_g_ubw_flat, tau_g_ubw, *g_v_ubw_flat, *g_no_force_ubw_flat, g_lift_ubw_flat), axis=1) #*g_no_force_ubw_flat,  g_land_ubw_flat

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

ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')
ms.store(dict(a=np.array(solution['x'])))
