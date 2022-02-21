import time

import numpy as np

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os
from scipy.io import loadmat

def plotFunction(name, dim=None):
    cnstr_val = solution[name]['val'][0, 0]
    cnstr_lb = solution[name]['lb'][0, 0]
    cnstr_ub = solution[name]['ub'][0, 0]

    var_dim_select = set(range(cnstr_val.shape[0]))

    if dim is not None:
        if not set(dim).issubset(var_dim_select):
            raise Exception('Wrong selected dimension.')
        else:
            var_dim_select = dim

    plt.figure()
    for dim in var_dim_select:
        plt.plot(range(cnstr_val.shape[1]), cnstr_val[dim, :])

        plt.plot(range(cnstr_lb.shape[1]), cnstr_lb[dim, :], marker="x", markersize=3, linestyle='dotted', linewidth=1)
        plt.plot(range(cnstr_ub.shape[1]), cnstr_ub[dim, :], marker="x", markersize=3, linestyle='dotted', linewidth=1)

    plt.title(f'{name}')

def checkBounds(name, tol=0.):
    checkLowerBounds(name, tol)
    checkUpperBounds(name, tol)

def checkLowerBounds(name, tol=0.):
    cnstr_val = solution[name]['val'][0, 0]
    cnstr_lb = solution[name]['lb'][0, 0]
    cnstr_nodes = solution[name]['nodes'][0, 0]


    check_lb = cnstr_lb - cnstr_val

    for dim in range(check_lb.shape[0]):
        for node in range(check_lb.shape[1]):
            if check_lb[dim, node] > tol:
                print(f"{name} violates the lower bounds at dim {dim} and node {cnstr_nodes[0, node]} of {check_lb[dim, node]}" )


def checkUpperBounds(name, tol=0.):
    cnstr_val = solution[name]['val'][0, 0]
    cnstr_ub = solution[name]['ub'][0, 0]
    cnstr_nodes = solution[name]['nodes'][0, 0]

    check_ub = cnstr_val - cnstr_ub

    for dim in range(check_ub.shape[0]):
        for node in range(check_ub.shape[1]):
            if check_ub[dim, node] > tol:
                print(f"{name} violates the upper bounds at dim {dim} and node {cnstr_nodes[0, node]} of {check_ub[dim, node]}")



transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = '../playground/urdf/spot.urdf'
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

ms = mat_storer.matStorer('../playground/spot/spot_step_fd.mat')
solution = ms.load()

# print([name for name in solution])
# exit()
contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))

replay_traj = False
resample = False
plotting = True
check_bounds = False
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

q = cs.SX.sym('q', n_q)
q_dot = cs.SX.sym('q_dot', n_v)

u = cs.SX.sym("actuated_torques", n_v - 6)
tau = cs.vertcat(cs.SX.zeros(6, 1), u)

f_list = list()
for i in range(n_c):
    f_list.append(cs.SX.sym(f'f{i}', n_f))

# ============================== compute qddot ===================================================
# SET CONTACTS MAP
contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, f_list))
fd = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
q_ddot = fd.call(q, q_dot, tau, contact_map)

x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

tau_hist = np.zeros((solution["q_dot"].shape[0], solution["q_dot"].shape[1] - 1))
for i in range(tau_hist.shape[1]):
    tau_hist[6:, i] = solution["actuated_torques"][:, i]

qddot_hist = np.zeros(tau_hist.shape)
FD = kin_dyn.ForwardDynamics(kindyn, contacts_name, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

for i in range(solution['q'].shape[1] - 1):
    contact_map_i = dict(
        zip(contacts_name, [solution['f0'][:, i], solution['f1'][:, i], solution['f2'][:, i], solution['f3'][:, i]]))
    qddot_hist[:, i] = FD.call(solution["q"][:, i], solution["q_dot"][:, i], tau_hist[:, i],
                               contact_map_i).toarray().flatten()

# ================================================================================================
if replay_traj:
    if resample:

        if 'dt' in solution:
            dt_before_res = solution['dt'].flatten()
        else:
            dt_before_res = solution['constant_dt'].flatten()[0]

        dt_res = 0.001
        dae = {'x': x, 'p': cs.vertcat(u, f_list[0], f_list[1], f_list[2], f_list[3]), 'ode': x_dot, 'quad': 1}

        input = np.array(cs.vertcat(solution["actuated_torques"], solution["f0"], solution["f1"], solution["f2"], solution["f3"]))
        # q_res, qdot_res, input_res = resampler_trajectory.second_order_resample_integrator(solution["q"], solution["q_dot"], input, dt_before_res, dt_res, dae)

        state = cs.vertcat(solution["q"], solution["q_dot"])
        state_res = resampler_trajectory.resampler(state, input, dt_before_res, dt_res, dae)

        q_res = state_res[n_q:, :]
        qdot_res = state_res[:n_v, :]

        tau_res = resampler_trajectory.resample_input(solution["actuated_torques"], dt_before_res, dt_res)
        f1_res = resampler_trajectory.resample_input(solution["f0"], dt_before_res, dt_res)
        f2_res = resampler_trajectory.resample_input(solution["f1"], dt_before_res, dt_res)
        f3_res = resampler_trajectory.resample_input(solution["f2"], dt_before_res, dt_res)
        f4_res = resampler_trajectory.resample_input(solution["f3"], dt_before_res, dt_res)

        contact_map = dict(zip(contacts_name, [f1_res, f2_res, f3_res, f4_res]))

        repl = replay_trajectory(dt_res, joint_names, q_res, contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

        repl.sleep(1.)
        repl.replay(is_floating_base=True)
    else:
        repl = replay_trajectory(solution['constant_dt'], joint_names, solution['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

    repl.sleep(1.)
    repl.replay(is_floating_base=True)

if plotting:

    constraint_names = ['collo_x_0', 'collo_x_1', 'collo_x_2', 'collo_continuity', 'collo_dyn_0', 'collo_dyn_1', 'collo_dyn_2', 'final_velocity', 'lf_foot_vel', 'lf_foot_fc', 'lh_foot_vel', 'lh_foot_fc', 'rf_foot_vel', 'rf_foot_fc', 'rh_foot_vel', 'rh_foot_fc', 'constant_dt']
    plt.figure()
    for dim in range(solution['q'].shape[0]):
        plt.plot(range(solution['q'].shape[1]), np.array(solution['q'][dim, :]))
    plt.title('q')

    plt.figure()
    for dim in range(solution['q_dot'].shape[0]):
        plt.plot(range(solution['q_dot'].shape[1]), np.array(solution['q_dot'][dim, :]))
    plt.title('q_dot')

    plt.figure()
    for dim in range(qddot_hist.shape[0]):
        plt.plot(range(qddot_hist.shape[1]), np.array(qddot_hist[dim, :]))
    plt.title('q_ddot')

    pos_contact_list = list()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']
        plt.figure()
        plt.title(contact)
        for dim in range(n_f):
            plt.plot(np.array([range(pos.shape[1])]), np.array(pos[dim, :]), marker="x", markersize=3, linestyle='dotted')

        plt.vlines([node_start_step, node_end_step], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)

    #plane_xy
    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xy')
        plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

    # plane_xz
    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xz')
        plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)

    # forces
    for f in [f'f{i}' for i in range(len(contacts_name))]:
        plt.figure()
        for dim in range(solution[f].shape[0]):
            plt.plot(np.array(range(solution[f].shape[1])), solution[f][dim, :])

        plt.title(f'force {f}')

    # plotFunction('inverse_dynamics')

    # dt
    if 'dt' in solution:
        plt.figure()
        for dim in range(solution['dt'].shape[0]):
            plt.plot(np.array(range(solution['dt'].shape[1])), solution['dt'][dim, :])

            plt.title(f'dt')

        dt_timeline = list()
        dt_timeline.append(0)
        for i in range(solution['dt'].shape[1]):
            dt_timeline.append(dt_timeline[-1] + solution['dt'][0, i])

        for f in [f'f{i}' for i in range(len(contacts_name))]:

            plt.figure()
            for dim in range(solution[f].shape[0]):
                plt.plot(dt_timeline[:-1], solution[f][dim, :])
                plt.scatter(dt_timeline[:-1], solution[f][dim, :])
            plt.title(f'forces {f} with dt')

        # for tau in solution['inverse_dynamics']:
        #     for dim in range(solution['inverse_dynamics'].shape[0]):
        #         plt.plot(np.array(range(solution['inverse_dynamics'].shape[1])), solution['inverse_dynamics'][dim, :])
        #     plt.title(f'inverse_dynamics')

        # for cnstr in constraint_names:
        #     plotFunction(cnstr)


    plt.show()