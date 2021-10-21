import casadi as cs
from horizon.utils import utils, kin_dyn, mat_storer
import numpy as np
import time
import os
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import horizon.transcriptions.integrators as integ

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
dt = 0.02

node_start_step = 30
node_end_step = N
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
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, f_list))


g_tau = kin_dyn.InverseDynamics(kindyn,
                              contact_map.keys(),
                              cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot, q_ddot, contact_map)

fs = cs.vertcat(*f_list)
state = cs.vertcat(q, q_dot)
input = cs.vertcat(q_ddot, fs)

x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
dae = {'x': state, 'p': input, 'ode': x_dot, 'quad': 1}
integrator = integ.RK4(dae, opts=dict(tf=dt))

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
mu = 1
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

g_v_dict = list()
g_lift_dict = list()
g_fc_dict = list()
g_no_force_dict = list()
# active_leg = slice(0,3)


contact_map_i = dict(lf_foot=f_i[0:3, :],
                     rf_foot=f_i[3:6, :],
                     lh_foot=f_i[6:9, :],
                     rh_foot=f_i[9:12, :])

g_tau_i = kin_dyn.InverseDynamicsMap(N, kindyn,
                              contact_map.keys(),
                              cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q_i[:, :N], q_dot_i[:, :N], q_ddot_i, contact_map_i)

active_leg = ['lf_foot', 'rf_foot']

active_slice = slice(node_start_step, node_end_step)
i = 0

for frame, f in contact_map.items():
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q_i, qdot=q_dot_i)['ee_vel_linear']
    if frame in active_leg:
        # can move ee
        g_v_dict.append(v[:, 0:node_start_step])
    else:
        # 0 vel for ee
        g_v_dict.append(v)

    if frame in active_leg:
        g_fc_dict.append(kin_dyn.linearized_friction_cone_map(contact_map_i[frame][:, 0:node_start_step], mu, R, node_start_step))
    else:
        g_fc_dict.append(kin_dyn.linearized_friction_cone_map(contact_map_i[frame], mu, R, N_control))

    i += 1

    if frame in active_leg:
        FK = cs.Function.deserialize(kindyn.fk(frame))
        p_i = FK(q=q_i)['ee_pos']
        p_start = FK(q=q_init)['ee_pos']
        p_goal = p_start + [0., 0., 0.2]

        g_lift_dict.append(p_i[:, node_start_step + 20] - p_goal)
        # no force during lift
        g_no_force_dict.append(contact_map_i[frame][:, node_start_step:node_end_step])


# g_land = p_i[:, 90] - p_start
# prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step + 2)

# prb.createConstraint(f"lift_{frame}_leg", p - p_goal, nodes=25)
# prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step + 2)

f_list = list()
# f_list.append(1000 * cs.sumsqr(q_i - q_init))
# f_list.append(1e-4 * cs.sumsqr(f_i))
f_list.append(1e-2 * cs.sumsqr(f_i[0:3, :]))
f_list.append(1e-2 * cs.sumsqr(f_i[3:6, :]))
f_list.append(1e-2 * cs.sumsqr(f_i[6:9, :]))
f_list.append(1e-2 * cs.sumsqr(f_i[9:12, :]))

f_list.append(1e3 * cs.sumsqr(q_dot_i))
# f_list.append(1 * cs.sumsqr(q_ddot_i))
# =====================================================================================
# =====================================================================================
# f_list.append(cs.sumsqr(q_base))
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

# zero initial velocity
q_dot_lbw[:, 0] = np.zeros([1, n_v])
q_dot_ubw[:, 0] = np.zeros([1, n_v])

# zero final velocity
q_dot_lbw[:, N] = np.zeros([1, n_v])
q_dot_ubw[:, N] = np.zeros([1, n_v])

# q bounds
# q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
# q_min.extend(kindyn.q_min()[7:])
# q_min = np.array(q_min)
#
# q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
# q_max.extend(kindyn.q_max()[7:])
# q_max = np.array(q_max)
#
# q_lbw[:, :] = np.tile(q_min, (N_states, 1)).T
# q_ubw[:, :] = np.tile(q_max, (N_states, 1)).T

# initial q
q_lbw[:, 0] = q_init
q_ubw[:, 0] = q_init

# q_dot bounds
q_dot_lim = np.inf * np.ones(n_v)
q_dot_lbw[:, :] = np.tile(-q_dot_lim, (N_states, 1)).T
q_dot_ubw[:, :] = np.tile(q_dot_lim, (N_states, 1)).T

# q_ddot bounds
q_ddot_lim = np.inf * np.ones(n_v)
q_ddot_lbw[:, :] = np.tile(-q_ddot_lim, (N_control, 1)).T
q_ddot_ubw[:, :] = np.tile(q_ddot_lim, (N_control, 1)).T

# f bounds
f_lim = np.inf * np.ones(n_c * n_f)
f_lbw[:, :] = np.tile(-f_lim, (N_control, 1)).T
f_ubw[:, :] = np.tile(f_lim, (N_control, 1)).T

# z component of f cannot go lower than 0
f_lbw[2, :] = np.zeros(f_lbw[2, :].shape)
f_lbw[5, :] = np.zeros(f_lbw[5, :].shape)
f_lbw[8, :] = np.zeros(f_lbw[8, :].shape)
f_lbw[11, :] = np.zeros(f_lbw[11, :].shape)

# lower bound constraints
tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
                    1000., 1000., 1000.,  # Contact 1
                    1000., 1000., 1000.,  # Contact 2
                    1000., 1000., 1000.,  # Contact 3
                    1000., 1000., 1000.])  # Contact 4

# ======================================
# tau_g_lbw = np.tile(-tau_lim, (N_control, 1)).T
# tau_g_ubw = np.tile(tau_lim, (N_control, 1)).T

tau_g_lbw = np.zeros(g_tau_i[0:6, :].shape)
tau_g_ubw = np.zeros(g_tau_i[0:6, :].shape)
# ======================================

multi_shoot_g_lbw = np.zeros(g_multi_shoot.shape)
multi_shoot_g_ubw = np.zeros(g_multi_shoot.shape)

# ======================================
g_lift_lbw = list()
for elem in g_lift_dict:
    g_lift_lbw.append(np.zeros(elem.shape))

g_lift_ubw = list()
for elem in g_lift_dict:
    g_lift_ubw.append(np.zeros(elem.shape))

# ======================================
g_v_lbw = list()
for elem in g_v_dict:
    g_v_lbw.append(np.zeros(elem.shape))

g_v_ubw = list()
for elem in g_v_dict:
    g_v_ubw.append(np.zeros(elem.shape))

# ======================================
g_fc_lbw = list()
for elem in g_fc_dict:
    g_fc_lbw.append(-np.inf * np.ones(elem.shape))

g_fc_ubw = list()
for elem in g_fc_dict:
    g_fc_ubw.append(np.zeros(elem.shape))

# ======================================
g_no_force_lbw = list()
for elem in g_no_force_dict:
    g_no_force_lbw.append(np.zeros(elem.shape))

g_no_force_ubw = list()
for elem in g_no_force_dict:
    g_no_force_ubw.append(np.zeros(elem.shape))

# ======================================


# ======================================================================================================================
# reshape stuff ========================================================================================================
# ======================================================================================================================

# VARIABLES
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

tau_g_lbw_flat = np.reshape(tau_g_lbw, [1, g_tau_i[0:6, :].shape[0] * g_tau_i[0:6, :].shape[1]], order='F')
tau_g_ubw_flat = np.reshape(tau_g_ubw, [1, g_tau_i[0:6, :].shape[0] * g_tau_i[0:6, :].shape[1]], order='F')

# CONSTRAINTS
multi_shoot_g_lbw_flat = np.reshape(multi_shoot_g_lbw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')
multi_shoot_g_ubw_flat = np.reshape(multi_shoot_g_ubw, [1, g_multi_shoot.shape[0] * g_multi_shoot.shape[1]], order='F')

g_lift_lbw_flat = list()
for elem in g_lift_lbw:
    g_lift_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_lift_ubw_flat = list()
for elem in g_lift_ubw:
    g_lift_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_v_lbw_flat = list()
for elem in g_v_lbw:
    g_v_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_v_ubw_flat = list()
for elem in g_v_ubw:
    g_v_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_fc_lbw_flat = list()
for elem in g_fc_lbw:
    g_fc_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_fc_ubw_flat = list()
for elem in g_fc_ubw:
    g_fc_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_no_force_lbw_flat = list()
for elem in g_no_force_lbw:
    g_no_force_lbw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))

g_no_force_ubw_flat = list()
for elem in g_no_force_ubw:
    g_no_force_ubw_flat.append(np.reshape(elem, [1, elem.shape[0] * elem.shape[1]], order='F'))


# ======================================================================================================================
# ======================================================================================================================
# ======================================================================================================================
w = cs.veccat(q_i, q_dot_i, q_ddot_i, f_i)
g = cs.veccat(g_multi_shoot, g_tau_i[0:6, :], *g_v_dict, *g_no_force_dict, *g_lift_dict, *g_fc_dict) # *g_no_force_list # g_land
f = sum(f_list)

#
# print('g_tau_i', g_tau_i.shape)
# print('tau_g_lbw', tau_g_lbw.shape)
# print('tau_g_ubw', tau_g_ubw.shape)
# #
# for elem in g_v_dict:
#     print(f'g_v_dict', elem.shape)
#
# for elem in g_v_lbw:
#     print('g_v_lbw', elem.shape)
# for elem in g_v_ubw:
#     print('g_v_ubw', elem.shape)
#
# for elem in g_no_force_dict:
#     print(f'g_no_force_dict', elem.shape)
#
# for elem in g_no_force_lbw:
#     print('g_no_force_lbw', elem.shape)
# for elem in g_no_force_ubw:
#     print('g_no_force_ubw', elem.shape)
#
# for elem in g_lift_dict:
#     print(f'g_lift_dict', elem.shape)
#
# for elem in g_lift_lbw:
#     print('g_lift_lbw', elem.shape)
# for elem in g_lift_ubw:
#     print('g_lift_ubw', elem.shape)
#
# for elem in g_fc_dict:
#     print(f'g_fc_dict', elem.shape)
#
# for elem in g_fc_lbw:
#     print('g_fc_lbw', elem.shape)
# for elem in g_fc_ubw:
#     print('g_fc_ubw', elem.shape)
#

lbg = np.concatenate((multi_shoot_g_lbw_flat, tau_g_lbw_flat, *g_v_lbw_flat, *g_no_force_lbw_flat, *g_lift_lbw_flat, *g_fc_lbw_flat), axis=1)  #*g_no_force_lbw_flat,  g_land_lbw_flat
ubg = np.concatenate((multi_shoot_g_ubw_flat, tau_g_ubw_flat, *g_v_ubw_flat, *g_no_force_ubw_flat, *g_lift_ubw_flat, *g_fc_ubw_flat), axis=1) #*g_no_force_ubw_flat,  g_land_ubw_flat

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
print('time elapsed loading:', toc - tic)

# print(ubw)
# exit()
tic = time.time()
solution = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
toc = time.time()
print('time elapsed solving:', toc - tic)

ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')
ms.store(dict(a=np.array(solution['x'])))
