# /usr/bin/env python3
import horizon.variables
from horizon import problem
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers import Solver
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
import os
from horizon.ros.replay_trajectory import *
import numpy as np

## ==================
## PREPARE TRAJECTORY
## ==================
transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = '../urdf/spot.urdf'
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

tot_time = 1
dt_hint = 0.02
duration_step = 0.5

n_nodes = int(tot_time / dt_hint)
n_nodes_step = int(duration_step / dt_hint)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

jump_height = 0.1
node_start_step = 15
node_end_step = node_start_step + n_nodes_step

ms = mat_storer.matStorer('../spot/spot_jump_twist.mat')
prev_solution = ms.load()

prev_q = prev_solution['q']
prev_q_dot = prev_solution['q_dot']
prev_q_ddot = prev_solution['q_ddot']

prev_f_list = list()
for i in range(n_c):
    prev_f_list.append(prev_solution[f'f{i}'])

prev_tau = prev_solution['inverse_dynamics']['val'][0][0]
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
prev_contact_map = dict(zip(contacts_name, prev_f_list))

joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

if 'dt' in prev_solution:
    dt_before_res = prev_solution['dt'].flatten()
else:
    dt_before_res = prev_solution['constant_dt'].flatten()[0]

dt_res = 0.001

q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator_with_floating_base(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}
q_res, qdot_res, qddot_res, contact_map_res, tau_sol_res = resampler_trajectory.resample_torques(
    prev_q, prev_q_dot, prev_q_ddot, dt_before_res, dt_res, dae, prev_contact_map,
    kindyn,
    cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

num_samples = tau_sol_res.shape[1]

node_vec = np.zeros([n_nodes + 1])
for i in range(1, n_nodes + 1):
    node_vec[i] = node_vec[i - 1] + dt_before_res[i - 1]

node_vec_res = np.zeros([num_samples + 1])
for i in range(1, num_samples + 1):
    node_vec_res[i] = node_vec_res[i - 1] + dt_res

plot_flag = False

if plot_flag:
    import matplotlib.pyplot as plt

    plt.figure()
    for dim in range(q_res.shape[0]):
        plt.plot(node_vec_res, np.array(q_res[dim, :]))

    for dim in range(prev_q.shape[0]):
        plt.scatter(node_vec, np.array(prev_q[dim, :]))
    plt.title('q')

    plt.figure()
    for dim in range(qdot_res.shape[0]):
        plt.plot(node_vec_res, np.array(qdot_res[dim, :]))

    for dim in range(prev_q_dot.shape[0]):
        plt.scatter(node_vec, np.array(prev_q_dot[dim, :]))
    plt.title('qdot')

    plt.figure()
    for dim in range(qddot_res.shape[0]):
        plt.plot(node_vec_res[:-1], np.array(qddot_res[dim, :]))

    for dim in range(prev_q_ddot.shape[0]):
        plt.scatter(node_vec[:-1], np.array(prev_q_ddot[dim, :]))
    plt.title('q_ddot')

    plt.figure()
    for dim in range(6):
        plt.plot(node_vec_res[:-1], np.array(tau_sol_res[dim, :]))
    for dim in range(6):
        plt.scatter(node_vec[:-1], np.array(prev_tau[dim, :]))
    plt.title('tau on base')

    plt.figure()
    for dim in range(tau_sol_res.shape[0] - 6):
        plt.plot(node_vec_res[:-1], np.array(tau_sol_res[6 + dim, :]))
    for dim in range(prev_tau.shape[0] - 6):
        plt.scatter(node_vec[:-1], np.array(prev_tau[6 + dim, :]))
    plt.title('tau')
    plt.show()

tau_sol_base = tau_sol_res[:6, :]

threshold = 10.
## get index of values greater than a given threshold for each dimension of the vector, and remove all the duplicate values
nodes_exceed = np.unique(np.argwhere(tau_sol_base > threshold)[:, 1])
times_exceed = node_vec_res[nodes_exceed]

## base vector nodes augmented with new nodes + sort
new_nodes_vec = np.concatenate((node_vec, times_exceed))
new_nodes_vec.sort(kind='mergesort')

# new number of nodes
new_n_nodes = new_nodes_vec.shape[0]

## which index where added to base vector nodes
new_indices = np.where(np.in1d(new_nodes_vec, times_exceed))[0]
base_indices = np.where(np.in1d(new_nodes_vec, node_vec))[0]
# =========================================
ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')

old_nodes = 50

old_node_start_step = 20
old_node_end_step = 40
old_node_peak = 30

time_start_step = node_vec[old_node_start_step]
time_end_step = node_vec[old_node_end_step]
n_nodes = new_n_nodes-1

node_start_step = np.where(abs(time_start_step - new_nodes_vec) < dt_res)
if node_start_step:
    node_start_step = int(node_start_step[0][0])
else:
    raise Exception('something is wrong with time_start_step')

node_end_step = np.where(abs(time_end_step - new_nodes_vec) < dt_res)  # dt_res
if node_end_step:
    node_end_step = int(node_end_step[0][0])
else:
    raise Exception('something is wrong with time_start_step')


# SET PROBLEM STATE AND INPUT VARIABLES
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)

f_list = list()
for i in range(n_c):
    f_list.append(prb.createInputVariable(f'f{i}', n_f))

# SET CONTACTS MAP
contact_map = dict(zip(contacts_name, f_list))

# SET DYNAMICS
dt = prb.createParameter("dt", 1)  # variable dt as input

# dt = 0.01
# Computing dynamics
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)

# SET BOUNDS
# q bounds
q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

# q_dot bounds
q_dot_lim = 100. * np.ones(n_v)
# q_ddot bounds
q_ddot_lim = 100. * np.ones(n_v)
# f bounds
f_lim = 10000. * np.ones(n_f)

# set bounds and of q
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
# set bounds of q_dot
q_dot_init = np.zeros(n_v)
q_dot.setBounds(-q_dot_lim, q_dot_lim)
q_dot.setBounds(q_dot_init, q_dot_init, 0)
# set bounds of q_ddot
q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)
# set bounds of f
# for f in f_list:
#     f.setBounds(-f_lim, f_lim)

f_min = [-10000., -10000., -10.]
f_max = [10000., 10000., 10000.]
for f in f_list:
    f.setBounds(f_min, f_max)

# SET INITIAL GUESS
for node in range(prev_q.shape[1]):
    if node in base_indices:
        q.setInitialGuess(prev_q[:, node], node)
for node in range(prev_q_dot.shape[1]):
    if node in base_indices:
        q_dot.setInitialGuess(prev_q_dot[:, node], node)

for node in range(prev_q_ddot.shape[1]):
    if node in base_indices:
        q_ddot.setInitialGuess(prev_q_ddot[:, node], node)

for f, f_ig in zip(f_list, prev_f_list):
    for node in range(f_ig.shape[1]):
        if node in base_indices:
            f.setInitialGuess(f_ig[:, node], node)

# SET TRANSCRIPTION METHOD
th = Transcriptor.make_method(transcription_method, prb, dt, opts=transcription_opts)

# SET INVERSE DYNAMICS CONSTRAINTS
tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
                    1000., 1000., 1000.,  # Contact 1
                    1000., 1000., 1000.,  # Contact 2
                    1000., 1000., 1000.,  # Contact 3
                    1000., 1000., 1000.])  # Contact 4

tau = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot,
                                                                                                             q_ddot,
                                                                                                             contact_map)

prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lim, ub=tau_lim))

# SET FINAL VELOCITY CONSTRAINT
prb.createFinalConstraint('final_velocity', q_dot)

# SET CONTACT POSITION CONSTRAINTS
active_leg = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

mu = 1
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

fb_during_jump = np.array([q_init[0], q_init[1], q_init[2] + jump_height, 0, 0, 0.8509035, 0.525322])
q_final = q_init
q_final[3:7] = [0, 0, 0.8509035, 0.525322]

for frame, f in contact_map.items():
    # 2. velocity of each end effector must be zero
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    p_goal = p_start + [0., 0., jump_height]
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    prb.createConstraint(f"{frame}_vel_before_lift", v, nodes=range(0, node_start_step))
    prb.createConstraint(f"{frame}_vel_after_lift", v, nodes=range(node_end_step, n_nodes + 1))

    # friction cones must be satisfied
    fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
    prb.createIntermediateConstraint(f"{frame}_fc_before_lift", fc, nodes=range(0, node_start_step),
                                     bounds=dict(lb=fc_lb, ub=fc_ub))
    prb.createIntermediateConstraint(f"{frame}_fc_after_lift", fc, nodes=range(node_end_step, n_nodes),
                                     bounds=dict(lb=fc_lb, ub=fc_ub))

    prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_step, node_end_step))

    # prb.createConstraint(f"lift_{frame}_leg", p - p_goal, nodes=node_peak)
    prb.createConstraint(f"land_{frame}_leg", p[2] - p_start[2], nodes=node_end_step)

    # this is useful?
    # prb.createConstraint(f"subterrain_{frame}_leg", p[2] - p_start[2], bounds=dict(lb=-0.0001))

# SET COST FUNCTIONS
# pose of the robot during jump
prb.createCost(f"jump_fb", 1000 * cs.sumsqr(q[0:7] - fb_during_jump),
                       nodes=range(node_start_step, node_end_step))  # nodes_end_step
# minimum joint velocity
prb.createCost("min_q_dot", 1. * cs.sumsqr(q_dot))
# final pose of the robot
prb.createFinalCost(f"final_nominal_pos", 10000 * cs.sumsqr(q - q_final))
# forces
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", 0.01 * cs.sumsqr(f))

# prb.createIntermediateCost('min_dt', 100 * cs.sumsqr(dt))

# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 2000}
# 'ipopt.linear_solver': 'ma57'}

for i in range(len(new_nodes_vec)):
    dt.assign(new_nodes_vec[i], nodes=i)

sol = Solver.make_solver('ipopt', prb, dt, opts)



sol.solve()

solution = sol.getSolutionDict()
solution_constraints = sol.getConstraintSolutionDict()

solution_constraints_dict = dict()
for name, item in prb.getConstraints().items():
    lb, ub = item.getBounds()
    lb_mat = np.reshape(lb, (item.getDim(), len(item.getNodes())), order='F')
    ub_mat = np.reshape(ub, (item.getDim(), len(item.getNodes())), order='F')
    solution_constraints_dict[name] = dict(val=solution_constraints[name], lb=lb_mat, ub=ub_mat, nodes=item.getNodes())

if isinstance(dt, cs.SX):
    ms.store({**solution, **solution_constraints_dict})
else:
    dt_dict = dict(constant_dt=dt)
    ms.store({**solution, **solution_constraints_dict, **dt_dict})
