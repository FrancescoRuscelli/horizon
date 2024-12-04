#!/usr/bin/env python3

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.solvers import solver
import os, argparse
from itertools import filterfalse
import numpy as np
import casadi as cs



action = 'leap'
rviz_replay = False
solver_type = 'ipopt'
codegen = False
warmstart_flag = False
plot_sol = True
resampling = False
load_initial_guess = False

if rviz_replay:
    from horizon.ros.replay_trajectory import replay_trajectory
    import rospy
    plot_sol = False

path_to_examples = os.path.abspath(__file__ + "/../../examples/")

# mat storer
if warmstart_flag:
    file_name = os.path.splitext(os.path.basename(__file__))[0]
    save_dir = path_to_examples + '/mat_files'
    save_file = path_to_examples + f'/mat_files/{file_name}.mat'

    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)

    ms = mat_storer.matStorer(save_file)

    if os.path.isfile(save_file):
        print(f'{file_name}.mat file found. Using previous solution as initial guess.')
        load_initial_guess = True
    else:
        print(f'{file_name}.mat file NOT found. The solution will be saved for future warmstarting.')

# options
transcription_method = 'multiple_shooting'
transcription_opts = dict(integrator='RK4')

tf = 2.5
n_nodes = 50

disp = [0., 0., 0., 0., 0., 0., 1.]

if action == 'jump_forward' or action == 'leap':
    disp[0] = 2  # [m]
if action == 'jump_twist':
    disp[3:7] = [0, 0, 0.8509035, 0.525322]

if action == 'wheelie':
    node_action = (20, n_nodes)
elif action == 'leap':
    node_action = [(15, 35), (30, 45)]
else:
    node_action = (20, 30)

# load urdf
urdffile = os.path.join(path_to_examples, 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

# parameters
n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3
dt = tf / n_nodes

# define dynamics
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)
f_list = [prb.createInputVariable(f'force_{i}', n_f) for i in contacts_name]
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)
# contact map
contact_map = dict(zip(contacts_name, f_list))

# import initial guess if present
if load_initial_guess:
    if os.path.exists(path_to_examples + f'/mat_files/{file_name}.mat'):
        prev_solution = ms.load()
        q_ig = prev_solution['q']
        q_dot_ig = prev_solution['q_dot']
        q_ddot_ig = prev_solution['q_ddot']
        f_ig_list = [prev_solution[f.getName()] for f in f_list]

# initial state and initial guess
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

q.setBounds(q_init, q_init, 0)
q_dot.setBounds(np.zeros(n_v), np.zeros(n_v), 0)

q.setInitialGuess(q_init)

if load_initial_guess:
    q.setInitialGuess(q_ig)
    q_dot.setInitialGuess(q_dot_ig)
    q_ddot.setInitialGuess(q_ddot_ig)
    [f.setInitialGuess(f_ig) for f, f_ig in zip(f_list, f_ig_list)]
else:
    [f.setInitialGuess([0, 0, 55]) for f in f_list]

# transcription
if solver_type != 'ilqr':
    th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# dynamic feasibility
id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
tau = id_fn.call(q, q_dot, q_ddot, contact_map)
prb.createIntermediateConstraint("dynamic_feasibility", tau[:6])

# final velocity is zero
prb.createFinalConstraint('final_velocity', q_dot)

# contact handling
k_all = range(1, n_nodes + 1)
if action == 'leap':
    list_swing = [list(range(*n_range)) for n_range in node_action]
    k_swing = [item for sublist in list_swing for item in sublist]
    k_swing_front = list(range(*[node for node in node_action[0]]))
    k_swing_hind = list(range(*[node for node in node_action[1]]))
    k_stance_front = list(filterfalse(lambda k: k in k_swing_front, k_all))
    k_stance_hind = list(filterfalse(lambda k: k in k_swing_hind, k_all))

else:
    k_swing = list(range(*[node for node in node_action]))

k_stance = list(filterfalse(lambda k: k in k_swing, k_all))

# list of lifted legs
lifted_legs = ['lf_foot', 'rf_foot']

if action != 'wheelie' and action != 'jump_on_wall':
    lifted_legs.extend(['lh_foot', 'rh_foot'])

q_final = q_init
q_final[:3] = q_final[:3] + disp[:3]
q_final[3:7] = disp[3:7]

def barrier(x):
    return cs.sum1(cs.if_else(x > 0, 0, x ** 2))

for frame, f in contact_map.items():
    nodes_stance = k_stance if frame in lifted_legs else k_all

    if action == 'leap':
        nodes_stance = k_stance_front if frame in ['lf_foot', 'rf_foot'] else k_stance_hind
        nodes_swing = k_swing_front if frame in ['lf_foot', 'rf_foot'] else k_swing_hind

    FK = cs.Function.deserialize(kindyn.fk(frame))
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))

    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    prb.createConstraint(f"{frame}_vel", v, nodes=nodes_stance)
    prb.createIntermediateCost(f'{frame}_fn', barrier(f[2] - 25.0))

    if action == 'leap':
        if solver_type == 'ilqr':
            prb.createIntermediateCost(f'{frame}_ground', 1e3 * barrier(p[2] - p_start[2]), nodes=nodes_swing)
        else:
            gc = prb.createConstraint(f'{frame}_ground', p[2], nodes=nodes_swing)
            gc.setLowerBounds(p_start[2])

    if frame in lifted_legs:
        if action == 'jump_on_wall':
            mu = 1
            p_goal = p_start + [0.3, 0., 0.8]
            rot = -np.pi / 2.
            R_wall = np.array([[np.cos(rot), 0, np.sin(rot)],
                               [0, 1, 0],
                               [-np.sin(rot), 0, np.cos(rot)]])

            fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(f, mu, R_wall)
            if solver_type == 'ipopt':
                prb.createIntermediateConstraint(f"{frame}_fc_wall", fc, nodes=range(k_swing[-1], n_nodes),
                                                 bounds=dict(lb=fc_lb, ub=fc_ub))
            else:
                prb.createIntermediateCost(f"{frame}_fc_wall_lb", 1 * barrier(fc - fc_lb),
                                           nodes=range(k_swing[-1], n_nodes))
                prb.createIntermediateCost(f"{frame}_fc_wall_ub", 1 * barrier(fc_ub - fc),
                                           nodes=range(k_swing[-1], n_nodes))

            prb.createFinalConstraint(f"lift_{frame}_leg", p - p_goal)

# swing force is zero
for leg in lifted_legs:
    if action == 'leap':
        nodes = k_swing_front if leg in ['lf_foot', 'rf_foot'] else k_swing_hind
    else:
        nodes = k_swing

    fzero = np.zeros(n_f)
    contact_map[leg].setBounds(fzero, fzero, nodes=nodes)

if action != 'wheelie' and action != 'jump_on_wall':
    if solver_type == 'ilqr':
        prb.createFinalConstraint(f"final_nominal_pos_base", q[:6] - q_final[:6])
        prb.createFinalCost(f"final_nominal_pos_joints", 1e3 * cs.sumsqr(q[7:] - q_final[7:]))
    else:
        prb.createFinalConstraint(f"final_nominal_pos", q - q_final)

prb.createResidual("min_q_dot", q_dot)
# prb.createIntermediateResidual("min_q_ddot", 1e-3* (q_ddot))
for f in f_list:
    prb.createIntermediateResidual(f"min_{f.getName()}", cs.sqrt(3e-3) * f)

# =============
# SOLVE PROBLEM
# =============

opts = dict()

if solver_type == 'ipopt':
    opts['ipopt.tol'] = 0.001
    opts['ipopt.constr_viol_tol'] = n_nodes * 1e-3
    opts['ipopt.max_iter'] = 2000

if solver_type == 'ilqr':
    opts['ilqr.max_iter'] = 200
    opts['ilqr.integrator'] = 'RK4'
    opts['ilqr.closed_loop_forward_pass'] = True
    opts['ilqr.line_search_accept_ratio'] = 1e-9
    opts['ilqr.constraint_violation_threshold'] = 1e-3
    opts['ilqr.step_length_threshold'] = 1e-3
    opts['ilqr.alpha_min'] = 0.2
    opts['ilqr.kkt_decomp_type'] = 'qr'
    opts['ilqr.constr_decomp_type'] = 'qr'
    opts['ilqr.codegen_enabled'] = codegen
    opts['ilqr.codegen_workdir'] = '/tmp/spot_motions'

if solver_type == 'gnsqp':
    qp_solver = 'osqp'
    if qp_solver == 'osqp':
        opts['gnsqp.qp_solver'] = 'osqp'
        opts['warm_start_primal'] = True
        opts['warm_start_dual'] = True
        opts['merit_derivative_tolerance'] = 1e-3
        opts['constraint_violation_tolerance'] = n_nodes * 1e-3
        opts['osqp.polish'] = True  # without this
        opts['osqp.delta'] = 1e-6  # and this, it does not converge!
        opts['osqp.verbose'] = False
        opts['osqp.rho'] = 0.02
        opts['osqp.scaled_termination'] = False
    if qp_solver == 'qpoases':  # does not work!
        opts['gnsqp.qp_solver'] = 'qpoases'
        opts['sparse'] = True
        opts["enableEqualities"] = True
        opts["initialStatusBounds"] = "inactive"
        opts["numRefinementSteps"] = 0
        opts["enableDriftCorrection"] = 0
        opts["terminationTolerance"] = 10e9 * 1e-7
        opts["enableFlippingBounds"] = False
        opts["enableNZCTests"] = False
        opts["enableRamping"] = False
        opts["enableRegularisation"] = True
        opts["numRegularisationSteps"] = 2
        opts["epsRegularisation"] = 5. * 10e3 * 1e-7
        opts['hessian_type'] = 'posdef'

solv = solver.Solver.make_solver(solver_type, prb, opts)

try:
    solv.set_iteration_callback()
except:
    pass

solv.solve()

if solver_type == 'ilqr':
    solv.print_timings()

solution = solv.getSolutionDict()
dt_sol = solv.getDt()
cumulative_dt = np.zeros(len(dt_sol) + 1)
for i in range(len(dt_sol)):
    cumulative_dt[i + 1] = dt_sol[i] + cumulative_dt[i]

solution_constraints_dict = dict()

if warmstart_flag:
    if isinstance(dt, cs.SX):
        ms.store({**solution, **solution_constraints_dict})
    else:
        dt_dict = dict(dt=dt)
        ms.store({**solution, **solution_constraints_dict, **dt_dict})

contact_forces_names =[f'force_{contact}' for contact in contacts_name]
f_list = [solution[contact_force] for contact_force in contact_forces_names]
# ========================================================

if plot_sol:
    import matplotlib.pyplot as plt
    from matplotlib import gridspec
    from matplotlib.ticker import FormatStrFormatter

    save_path = '/home/francesco/Documents/workshop_icra/'
    fig_size = [19.20, 10.80]
    vline_width = 0.7
    contact_forces_name_nice = [r'Left Front', r'Right Front', r'Left Hind', r'Right Hind']
    wanted_time_label = {0, 20, 40, 50}
    # contact forces over time
    gs = gridspec.GridSpec(2, 2)
    gs.hspace = -0.75
    plot_ratio = 0.15
    w = 7195
    h = 3841

    fig = plt.figure(frameon=True)
    fig.set_size_inches(fig_size[0], fig_size[1])
    # manager = plt.get_current_fig_manager()
    # manager.window.showMaximized()
    fig.tight_layout()
    # fig.suptitle(r"Contact forces")
    plot_num = 0
    # contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
    for name, elem in zip(contact_forces_name_nice, f_list):
        ax = fig.add_subplot(gs[plot_num])
        for i in range(elem.shape[0]):  # get i-th dimension
            ax.step(cumulative_dt, np.append(elem[i, :], elem[i, -1]), where='post')

        # ax.axvline(cumulative_dt[node_start_step], linestyle='dashed', color='k', linewidth=vline_width)
        # ax.axvline(cumulative_dt[node_end_step], linestyle='dashed', color='k', linewidth=vline_width)

        # #### stuff for tickz and labels #####
        # y
        ax.yaxis.set_major_locator(plt.MultipleLocator(100))
        # x
        ax.set_xticks(cumulative_dt)

        label_list = list(range(cumulative_dt.shape[0]))
        label_list = [e for e in label_list if e not in wanted_time_label]

        xticks = ax.xaxis.get_major_ticks()
        for i_hide in label_list:
            xticks[i_hide].label1.set_visible(False)





        # plot lims
        x_left, x_right = ax.get_xlim()
        y_low, y_high = ax.get_ylim()
        ax.set_aspect(abs((x_right - x_left) / (y_low - y_high)) * plot_ratio)
        ax.legend(['$f_{x}$', '$f_{y}$', '$f_{z}$'])
        ax.set_title(name)
        ax.set_xlabel(r'time [s]')
        ax.set_ylabel(r'force [N]')
        ax.grid(alpha=0.4)
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        plt.xlim([0, cumulative_dt[-1]])

        if plot_num in [0, 1]:
            ax.set_xticklabels([])
            ax.set_xlabel("")

        plot_num += 1

    plt.savefig(save_path + "/spot_leap_forces", dpi=500, bbox_inches='tight')
    # # contact forces over nodes
    # gs = gridspec.GridSpec(2, 2)
    # fig = plt.figure()
    # fig.suptitle("$\text{Contact Forces}$")
    # plot_num = 0
    # for name, elem in zip(contact_forces_name_nice, f_list):
    #     ax = fig.add_subplot(gs[plot_num])
    #     ax.set_title(name)
    #     ax.grid(axis='x')
    #     for i in range(elem.shape[0]):  # get i-th dimension
    #
    #         ax.step(np.array(range(elem.shape[1])), elem[i, :], where='post')
    #         # ax.plot(np.array(range(elem.shape[1])), elem[i, :], 'o--', color='grey', alpha=0.3)
    #         ax.vlines([node_start_step, node_end_step], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1],
    #                    linestyles='dashed', colors='k', linewidth=0.4)
    #
    #     plot_num += 1

    # # contacts position
    fig = plt.figure(frameon=True)
    fig.set_size_inches(fig_size[0], fig_size[1])

    gs = gridspec.GridSpec(2, 2)
    gs.hspace = -0.75
    i = 0

    for name, contact in zip(contact_forces_name_nice, contacts_name):
        ax = fig.add_subplot(gs[i])

        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        for dim in range(n_f):
            ax.plot(cumulative_dt, pos[dim, :].toarray().flatten(), linestyle='-')

        # ax.axvline(cumulative_dt[node_start_step], linestyle='dashed', color='k', linewidth=vline_width)
        # ax.axvline(cumulative_dt[node_end_step], linestyle='dashed', color='k', linewidth=vline_width)

        # #### stuff for tickz and labels #####
        # y
        ax.yaxis.set_major_locator(plt.MultipleLocator(0.5))
        # x
        ax.set_xticks(cumulative_dt)

        label_list = list(range(cumulative_dt.shape[0]))
        label_list = [e for e in label_list if e not in wanted_time_label]

        xticks = ax.xaxis.get_major_ticks()
        for i_hide in label_list:
            xticks[i_hide].label1.set_visible(False)

        # plot lims
        x_left, x_right = ax.get_xlim()
        y_low, y_high = ax.get_ylim()
        ax.set_aspect(abs((x_right - x_left) / (y_low - y_high)) * plot_ratio)

        ax.legend(['$p_{x}$', '$p_{y}$', '$p_{z}$'], fancybox=True)
        ax.set_title(name)
        ax.set_xlabel(r'time [s]')
        ax.set_ylabel(r'pos [m]')
        ax.grid(alpha=0.4)
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        plt.xlim([cumulative_dt[0], cumulative_dt[-1]])

        if i in [0, 1]:
            ax.set_xticklabels([])
            ax.set_xlabel("")

        i += 1

    plt.savefig(save_path + "/spot_leap_pos", dpi=500, bbox_inches='tight')
    # hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables(show_bounds=True, same_fig=True, legend=False)
    # hplt.plotVariables([elem.getName() for elem in f_list], show_bounds=True, gather=2, legend=False)
    # hplt.plotFunctions(show_bounds=True, same_fig=True)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))


    # plt.rcParams['pgf.texsystem'] = 'pdflatex'
    # plt.rcParams.update({'font.family': 'serif'})
    # pos_contact_list = list()
    # fig = plt.figure()
    # fig.suptitle('Contacts')
    # gs = gridspec.GridSpec(2, 2)
    # i = 0
    # for contact in contacts_name:
    #     ax = fig.add_subplot(gs[i])
    #     ax.set_title('${}$'.format(contact))
    #     i += 1
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #     for dim in range(n_f):
    #         ax.plot(np.atleast_2d(cumulative_dt), np.array(pos[dim, :]), marker="x", markersize=3,
    #                 linestyle='dotted')  # marker="x", markersize=3, linestyle='dotted'
    #
    # plt.figure()
    # for contact in contacts_name:
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #
    #     plt.title(f'$feet position - plane_xy$')
    #     plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)
    #
    # plt.figure()
    # for contact in contacts_name:
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #
    #     plt.title(f'$feet position - plane_xz$')
    #     plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)


plt.show()
# ======================================================


    # pos_contact_list = list()
    # fig = plt.figure()
    # fig.suptitle('Contacts')
    # gs = gridspec.GridSpec(2, 2)
    # i = 0
    # for contact in contacts_name:
    #     ax = fig.add_subplot(gs[i])
    #     ax.set_title('{}'.format(contact))
    #     i += 1
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #     for dim in range(n_f):
    #         ax.plot(np.atleast_2d(cumulative_dt), np.array(pos[dim, :]), marker="x", markersize=3,
    #                 linestyle='dotted')  # marker="x", markersize=3, linestyle='dotted'
    #
    # plt.figure()
    # for contact in contacts_name:
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #
    #     plt.title(f'feet position - plane_xy')
    #     plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)
    #
    # plt.figure()
    # for contact in contacts_name:
    #     FK = cs.Function.deserialize(kindyn.fk(contact))
    #     pos = FK(q=solution['q'])['ee_pos']
    #
    #     plt.title(f'feet position - plane_xz')
    #     plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)
    #
    # plt.show()
# ======================================================
# contact_map = {contacts_name[i]: solution[f_list[i].getName()] for i in range(n_c)}
#
# # resampling
# if resampling:
#
#     if isinstance(dt, cs.SX):
#         dt_before_res = solution['dt'].flatten()
#     else:
#         dt_before_res = dt
#
#     dt_res = 0.001
#     dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
#     q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
#         solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
#         kindyn,
#         cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)


