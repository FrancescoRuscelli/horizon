import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os
from scipy.io import loadmat


def trajectoryInitializer(traj_duration, step_height, traj_len_before=0, traj_len_after=0):
    t = np.linspace(0, 1, np.ceil(traj_duration - (traj_len_after + traj_len_before)))
    traj_z = np.full(traj_len_before, 0.)
    traj_z = np.append(traj_z, (64. * t ** 3. * (1. - t) ** 3.) * step_height)
    traj_z = np.append(traj_z, np.full(traj_len_after, 0.))
    return traj_z

# ========================================
# traj = trajectoryInitializer(100, 10, 60, 10)
# initial_q = -0.5
# mod_q = initial_q + traj
#
# FK = cs.Function.deserialize(kindyn.ik(frame))
# p = FK(q=q)['ee_pos']
# p_start = FK(q=q_init)['ee_pos']

# plt.scatter(range(mod_q.shape[0]), mod_q)
# plt.show()
# exit()

# =========================================
ms = mat_storer.matStorer('spot_step.mat')

transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')



tot_time = 1
dt_hint = 0.02
duration_step = 0.5

n_nodes = int(tot_time / dt_hint)
n_nodes_step = int(duration_step / dt_hint)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

node_start_step = 15
node_end_step = node_start_step + n_nodes_step

print('total nodes:', n_nodes)
print('starting node of step:', node_start_step)
print('last node of step:', node_end_step)

load_initial_guess = True

# SET PROBLEM STATE AND INPUT VARIABLES
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)

f_list = list()
for i in range(n_c):
    f_list.append(prb.createInputVariable(f'f{i}', n_f))

# SET CONTACTS MAP
contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, f_list))


# import initial guess if present
if load_initial_guess:
    prev_solution = ms.load()
    q_ig = prev_solution['q']
    q_dot_ig = prev_solution['q_dot']
    q_ddot_ig = prev_solution['q_ddot']
    f_ig_list = list()
    for f in f_list:
        f_ig_list.append(prev_solution[f'f{i}'])

# SET DYNAMICS
dt = prb.createInputVariable("dt", 1)  # variable dt as input
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

dt_min = 0.01  # [s]
dt_max = 0.15  # [s]

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
for f in f_list:
    f.setBounds(-f_lim, f_lim)

# set bounds of dt
dt.setBounds(dt_min, dt_max)

# SET INITIAL GUESS
if load_initial_guess:
    for node in range(q_ig.shape[1]):
        q.setInitialGuess(q_ig[:, node], node)

    for node in range(q_dot_ig.shape[1]):
        q_dot.setInitialGuess(q_dot_ig[:, node], node)

    for node in range(q_ddot_ig.shape[1]):
        q_ddot.setInitialGuess(q_ddot_ig[:, node], node)

    for f, f_ig in zip(f_list, f_ig_list):
        for node in range(f_ig.shape[1]):
            f.setInitialGuess(f_ig[:, node], node)
else:
    q.setInitialGuess(q_init)

dt.setInitialGuess(dt_min)
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

# SET CONTACT POSITION CONSTRAINTS
active_leg = list()
active_leg = next(iter(contact_map))
# active_leg = ['lf_foot', 'rf_foot']
# active_leg = ['lf_foot', 'rf_foot', 'lh_foot']
# active_leg = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

mu = 0.8
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

for frame, f in contact_map.items():
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']

    p_goal = p_start + [0.1, 0., 0.2]
    # 1. position of each end effector and its initial position must be the same
    # if frame != active_leg:
    #     prb.createConstraint(f"{frame}_fixed", p - p_start)
    # else:
    #     prb.createConstraint(f"{frame}_before_step", p - p_start, nodes=range(0, node_start_step))
    #     prb.createConstraint(f"{frame}_after_step", p - p_goal, nodes=range(node_end_step, n_nodes+1))

    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    # 2. velocity of each end effector must be zero
    if frame not in active_leg:
        prb.createConstraint(f"{frame}_vel_before_step", v)
    else:
        prb.createConstraint(f"{frame}_vel_before_step", v, nodes=range(0, node_start_step))
        prb.createConstraint(f"{frame}_vel_after_step", v, nodes=range(node_end_step, n_nodes + 1))

    # friction cones must be satisfied
    fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
    if frame not in active_leg:
        prb.createIntermediateConstraint(f"{frame}_friction_cone", fc, bounds=dict(lb=fc_lb, ub=fc_ub))
    else:
        prb.createIntermediateConstraint(f"{frame}_friction_cone_before_step", fc, nodes=range(0, node_start_step), bounds=dict(lb=fc_lb, ub=fc_ub))
        prb.createIntermediateConstraint(f"{frame}_friction_cone_after_step", fc, nodes=range(node_end_step, n_nodes), bounds=dict(lb=fc_lb, ub=fc_ub))

    if frame in active_leg:
        prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_step, node_end_step))

        prb.createConstraint(f"lift_{frame}_leg", p - p_goal, nodes=25)
        prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step + 2)

        # prb.createCostFunction(f"lift_{frame}_leg", 100000 * cs.sumsqr(p - p_goal), nodes=range(node_start_step, node_end_step))
        # prb.createCostFunction(f"land_{frame}_leg", 10000000 * cs.sumsqr(p - p_start), nodes=range(node_end_step, n_nodes+1))

# SET COST FUNCTIONS
prb.createCostFunction("min_q_dot", 1000. * cs.sumsqr(q_dot))
# prb.createIntermediateCost("min_q_ddot", 10. * cs.sumsqr(q_ddot))

# don't know why this is not working
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", 0.01 * cs.sumsqr(f))

q_fb_trg = np.array([q_init[0], q_init[1], q_init[2] + 0.1, 0.0, 0.0, 0.0, 1.0])
# prb.createCostFunction("floating_base_orientation", 1000.*cs.sumsqr(q[3:7] - q_fb_trg[3:7]), nodes=list(range(node_start_step, node_end_step)))
# prb.createCostFunction("floating_base_position", 100000.*cs.sumsqr(q[0:7] - q_fb_trg))
# prb.createCostFunction("floating_base_position", 100000.*cs.sumsqr(q[3:7] - q_fb_trg[3:7]))
# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 5000}
# 'ipopt.linear_solver': 'ma57'}

solver = solver.Solver.make_solver('ipopt', prb, dt, opts)
solver.solve()

solution = solver.getSolutionDict()
ms.store(solution)

# ========================================================
plot_all = True
if plot_all:
    hplt = plotter.PlotterHorizon(prb, solution)
    hplt.plotVariables(show_bounds=False, legend=True)
    hplt.plotFunctions(show_bounds=False)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))

    pos_contact_list = list()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']
        plt.figure()
        plt.title(contact)
        for dim in range(n_f):
            plt.plot(np.array([range(pos.shape[1])]), np.array(pos[dim, :]), marker="x", markersize=3, linestyle='dotted')

        plt.vlines([node_start_step, node_end_step], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xy')
        plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xz')
        plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)

    plt.show()
# ======================================================


contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))

# resampling
resampling = True
if resampling:
    dt_res = 0.001
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], solution['dt'].flatten(), dt_res, dae, contact_map, kindyn,
                                                                            cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)



    repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
else:
    # remember to run a robot_state_publisher
    repl = replay_trajectory(dt, joint_names, solution['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

repl.sleep(1.)
repl.replay(is_floating_base=True)

