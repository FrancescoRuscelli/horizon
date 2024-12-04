#!/usr/bin/env python3
import numpy
from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.solvers import solver
import os, argparse
from itertools import filterfalse
import numpy as np
import casadi as cs

def str2bool(v):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

'''
An example of the famous robot Spot (from Boston Dynamics) performing a jump.
'''

# import the necessary modules
# 'problem' is the core of the library, used to build the problem
# 'transcriptions' submodule contains method to specify the transcription of the optimization problem
# 'solvers' submodules contains all the tools to solve the problem
# 'utils' and 'ros' are handy submodules
# 'casadi_kin_dyn' bridges URDF and Horizon through Pinocchio
from horizon import problem
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers.solver import Solver
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import matplotlib.pyplot as plt
from matplotlib import gridspec
import numpy as np
import casadi as cs
import os, math
from itertools import filterfalse

# # flag to activate the replay on rviz
# rviz_replay = True
# # flag to activate the resampling of the solution to an higher frequency
# resampling = True
# # flag to plot the solution
# plot_sol = True
# # flag to load initial guess (if a previous solution is present)
# load_initial_guess = True
#
# # get path to the examples folder and temporary add it to the environment
path_to_examples = os.path.abspath(__file__ + "/../../examples/")
# os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples
#
# # mat storer
# file_name = os.path.splitext(os.path.basename(__file__))[0]
# ms = mat_storer.matStorer(f'{file_name}.mat')
#
# # options for horizon transcription
# transcription_method = 'multiple_shooting' # can choose between 'multiple_shooting' and 'direct_collocation'
# transcription_opts = dict(integrator='RK4') # integrator used by the multiple_shooting
#
#
# # Create CasADi interface to Pinocchio
urdffile = os.path.join(path_to_examples, 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
#
# # joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names:
    joint_names.remove('universe')
if 'floating_base_joint' in joint_names:
    joint_names.remove('floating_base_joint')
#
# # Optimization parameters
n_nodes = 50
node_action = (20, 40)
node_start_step = 20
node_end_step = 40
#
# number of contacts (4 feet of spot)
n_c = 4
# Get dimension of pos and vel from urdf
n_q = kindyn.nq()
n_v = kindyn.nv()
# Dimension of the forces (x,y,z) [no torques]
n_f = 3

# Contact links name
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

# # Create horizon problem
# prb = problem.Problem(n_nodes)
#
# # creates the variables for the problem
# # design choice: position and the velocity as state and acceleration as input
#
# # STATE variables
# q = prb.createStateVariable('q', n_q)
# q_dot = prb.createStateVariable('q_dot', n_v)
# # CONTROL variables
# q_ddot = prb.createInputVariable('q_ddot', n_v)
# # set the dt as a variable: the optimizer can choose the dt in between each node
# dt = prb.createInputVariable("dt", 1)
# f_list = [prb.createInputVariable(f'force_{i}', n_f) for i in contacts_name]
#
# # Set the contact map: couple the link names at each force
# contact_map = dict(zip(contacts_name, f_list))
#
# # logic to load a previous solution as the initial guess of the optimization problem.
# # starting the problem with a "good" guess GREATLY simplify the solver's work, reducing the computation time.
# # the solution is stored in a .mat file. the 'mat_storer' module load the solution and set each variable to it for each node.
# if load_initial_guess:
#     prev_solution = ms.load()
#     q_ig = prev_solution['q']
#     q_dot_ig = prev_solution['q_dot']
#     q_ddot_ig = prev_solution['q_ddot']
#     f_ig_list = [prev_solution[f.getName()] for f in f_list]
#     dt_ig = prev_solution['dt']
#
#
# # Creates double integrator taking care of the floating base part
# x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
#
# # Set dynamics of the system and the relative dt
# prb.setDynamics(x_dot)
# prb.setDt(dt)
#
# # ===================== Set BOUNDS  ===================================
# # joint limits
# q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
# q_min.extend(kindyn.q_min()[7:])
# q_min = np.array(q_min)
#
# q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
# q_max.extend(kindyn.q_max()[7:])
# q_max = np.array(q_max)
#
# q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
#                    0.0, 0.9, -1.52,
#                    0.0, 0.9, -1.52,
#                    0.0, 0.9, -1.52,
#                    0.0, 0.9, -1.52])
#
# # velocity limits
# q_dot_lim = 100. * np.ones(n_v)
# # acceleration limits
# q_ddot_lim = 100. * np.ones(n_v)
# # f bounds
# f_min = [-10000., -10000., -10.]
# f_max = [10000., 10000., 10000.]
#
# # time bounds
# dt_min = 0.02  # [s]
# dt_max = 0.1  # [s]
#
# # set bounds and of q
# q.setBounds(q_min, q_max)
# q.setBounds(q_init, q_init, 0)
#
# # set bounds of q_dot
# q_dot_init = np.zeros(n_v)
# q_dot.setBounds(-q_dot_lim, q_dot_lim)
# q_dot.setBounds(q_dot_init, q_dot_init, 0)
#
# # set bounds of q_ddot
# q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)
#
# # set bounds of f
# [f.setBounds(f_min, f_max) for f in f_list]
#
# # set bounds of dt
# if isinstance(dt, cs.SX):
#     dt.setBounds(dt_min, dt_max)
#
# # ================== Set INITIAL GUESS  ===============================
# if load_initial_guess:
#     q.setInitialGuess(q_ig)
#     q_dot.setInitialGuess(q_dot_ig)
#     q_ddot.setInitialGuess(q_ddot_ig)
#     [f.setInitialGuess(f_ig) for f, f_ig in zip(f_list, f_ig_list)]
#
#     if isinstance(dt, cs.SX):
#         dt.setInitialGuess(dt_ig)
#
# else:
#     q.setInitialGuess(q_init)
#     [f.setInitialGuess([0, 0, 55]) for f in f_list]
#
#     if isinstance(dt, cs.SX):
#         dt.setInitialGuess(dt_min)
#
#
# # ================== Set TRANSCRIPTION METHOD ===============================
# th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)
#
# # ====================== Set CONSTRAINTS ===============================
#
# tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
#                     1000., 1000., 1000.,  # Contact 1
#                     1000., 1000., 1000.,  # Contact 2
#                     1000., 1000., 1000.,  # Contact 3
#                     1000., 1000., 1000.])  # Contact 4
#
#
# # the torques are computed using the inverse dynamics, as the input of the problem are the joint acceleration and the forces
# id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
# tau = id_fn.call(q, q_dot, q_ddot, contact_map)
#
# # dynamic feasiblity constraint:
# # floating base cannot exert torques (+ torque bounds of other joints)
# prb.createIntermediateConstraint("dynamic_feasibility", tau, bounds=dict(lb=-tau_lim, ub=tau_lim))
#
# # Set final velocity constraint: at the last node, the robot is standing still
# prb.createFinalConstraint('final_velocity', q_dot)
#
# # set a final pose of the floating base and robot configuration:
# # rotate the robot orientation on the z-axis
# q_final = q_init.copy()
# q_final[3:7] = [0, 0, 0.8509035, 0.525322] # equivalent to 2/3 * pi
# prb.createFinalConstraint(f"final_nominal_pos", q - q_final)
#
#
# k_all = range(1, n_nodes + 1)
# # gather all the nodes where the robot is NOT touching the ground/jumping
# nodes_swing = list(range(*[node for node in node_action]))
# nodes_stance = list(filterfalse(lambda k: k in nodes_swing, k_all))
#
# # iterate over the four contacts:
# for frame, f in contact_map.items():
#
#     FK = cs.Function.deserialize(kindyn.fk(frame))
#     DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
#     DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
#
#     p = FK(q=q)['ee_pos']
#     p_start = FK(q=q_init)['ee_pos']
#     v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
#     a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']
#
#     # velocity of each end effector must be zero before and after the jump
#     prb.createConstraint(f"{frame}_vel_ground", v, nodes=nodes_stance)
#
#     # friction cones must be satisfied while the robot is touching the ground
#     # parameters of the friction cones:
#     mu = 1  # max friction coefficient
#     R = np.identity(3, dtype=float)  # environment rotation w.r.t. inertial frame
#     fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(f, mu, R)
#     prb.createIntermediateConstraint(f"{frame}_fc_ground", fc, nodes=nodes_stance[:-1], bounds=dict(lb=fc_lb, ub=fc_ub))
#
#     # the robot cannot exert force in this node:
#     # the only possible solution for the solver is a jumping trajectory
#     prb.createConstraint(f"{frame}_no_force_during_jump", f, nodes=nodes_swing)
#
# # ====================== Set COSTS ===============================
#
# # minimize the velocity of the robot
# prb.createCost("min_q_dot", 3 * cs.sumsqr(q_dot))
#
# # regularization term for the forces of system (input)
# for f in f_list:
#     prb.createIntermediateCost(f"min_{f.getName()}", 0.02 * cs.sumsqr(f))
#
# # ==================== BUILD PROBLEM =============================
#
# # the chosen solver is IPOPT.
# # populating the options, it is possible to access the internal option of the solver.
# opts = {'ipopt.tol': 0.001,
#         'ipopt.constr_viol_tol': 0.001,
#         'ipopt.max_iter': 2000}
#
# # the solver class accept different solvers, such as 'ipopt', 'ilqr', 'gnsqp'.
# # Different solver are useful (and feasible) in different situations.
# solv = Solver.make_solver('ipopt', prb, opts)
#
# # ==================== SOLVE PROBLEM =============================
# solv.solve()
#
# # ====================== SAVE SOLUTION =======================
#
# # the solution is retrieved in the form of a dictionary ('variable_name' = values)
# solution = solv.getSolutionDict()
# # the dt is retrieved as a vector of values (size: number of intervals --> n_nodes - 1)
# dt_sol = solv.getDt()
#
# # the value of the constraints functions
# solution_constraints = solv.getConstraintSolutionDict()
#
# # populate what will be the .mat file with the solution
# solution_constraints_dict = dict()
# for name, item in prb.getConstraints().items():
#     lb, ub = item.getBounds()
#     lb_mat = np.reshape(lb, (item.getDim(), len(item.getNodes())), order='F')
#     ub_mat = np.reshape(ub, (item.getDim(), len(item.getNodes())), order='F')
#     solution_constraints_dict[name] = dict(val=solution_constraints[name], lb=lb_mat, ub=ub_mat, nodes=item.getNodes())
#
# info_dict = dict(n_nodes=n_nodes, node_start_step=node_start_step, node_end_step=node_end_step)
#
# if isinstance(dt, cs.SX):
#     ms.store({**solution, **solution_constraints_dict, **info_dict})
# else:
#     dt_dict = dict(dt=dt)
#     ms.store({**solution, **solution_constraints_dict, **info_dict, **dt_dict})
#
# import numpy as np
# import matplotlib.pyplot as plt
#
# x = np.arange(3)
# y = np.array([0, 1 , 2])
#
# plt.step(x, y + 2, label='pre (default)')
# plt.plot(x, y + 2, 'o--', color='grey', alpha=0.3)
#
# plt.step(x, y + 1, where='mid', label='mid')
# plt.plot(x, y + 1, 'o--', color='grey', alpha=0.3)
#
# plt.step(x, y, where='post', label='post')
# plt.plot(x, y, 'o--', color='grey', alpha=0.3)
#
# plt.grid(axis='x', color='0.95')
# plt.legend(title='Parameter where:')
# plt.title('plt.step(where=...)')
# plt.show()
#
# exit()
file_path = os.path.abspath(__file__ + '/..')
ms = mat_storer.matStorer(file_path + '/spot_plots.mat')
solution = ms.load()
contact_forces_names =[f'force_{contact}' for contact in contacts_name]
f_list = [solution[contact_force] for contact_force in contact_forces_names]

dt_sol = solution['dt'].flatten()
cumulative_dt = np.zeros(len(dt_sol) + 1)
for i in range(len(dt_sol)):
    cumulative_dt[i + 1] = dt_sol[i] + cumulative_dt[i]

# ========================================================
plot_sol = True

plt.rcParams['text.usetex'] = True
plt.rcParams["mathtext.fontset"] = 'cm'
plt.rcParams["legend.fontsize"] = 14
plt.rcParams["axes.titlesize"] = 20
plt.rcParams["axes.labelsize"] = 20
plt.rcParams["xtick.labelsize"] = 15
plt.rcParams["ytick.labelsize"] = 15
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42



if plot_sol:
    import matplotlib.pyplot as plt
    from matplotlib import gridspec
    from matplotlib.ticker import FormatStrFormatter

    save_path = '/home/francesco/Documents/all_of_horizon/plots/spot_jump_twist/'
    fig_size = [19.20, 10.80]
    vline_width = 0.7
    contact_forces_name_nice = [r'Left Front', r'Right Front', r'Left Hind', r'Right Hind']
    wanted_time_label = {0, 20, 40, 50}
    # contact forces over time
    gs = gridspec.GridSpec(2, 2)
    gs.hspace = 0.3

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
            ax.step(cumulative_dt, numpy.append(elem[i, :], elem[i, -1]), where='post')

        # ax.axvline(cumulative_dt[node_start_step], linestyle='dashed', color='k', linewidth=vline_width)
        # ax.axvline(cumulative_dt[node_end_step], linestyle='dashed', color='k', linewidth=vline_width)

        # #### stuff for tickz and labels #####
        # y
        ax.yaxis.set_major_locator(plt.MultipleLocator(50))
        # x
        ax.set_xticks(cumulative_dt)

        label_list = list(range(cumulative_dt.shape[0]))
        label_list = [e for e in label_list if e not in wanted_time_label]

        xticks = ax.xaxis.get_major_ticks()
        for i_hide in label_list:
            xticks[i_hide].label1.set_visible(False)

        plot_num += 1

        # plot lims
        ax.legend(['$f_{x}$', '$f_{y}$', '$f_{z}$'])
        ax.set_title(name)
        ax.set_xlabel(r'time [s]')
        ax.set_ylabel(r'force [N]')
        ax.grid(alpha=0.4)
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        plt.xlim([0, cumulative_dt[-1]])


    plt.savefig(save_path + "/spot_jump_twist_forces", dpi=500, bbox_inches='tight')
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
    gs.hspace = 0.3
    i = 0

    for name, contact in zip(contact_forces_name_nice, contacts_name):
        ax = fig.add_subplot(gs[i])

        i += 1
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        for dim in range(n_f):
            ax.plot(cumulative_dt, pos[dim, :].toarray().flatten(), linestyle='-')

        # ax.axvline(cumulative_dt[node_start_step], linestyle='dashed', color='k', linewidth=vline_width)
        # ax.axvline(cumulative_dt[node_end_step], linestyle='dashed', color='k', linewidth=vline_width)

        # #### stuff for tickz and labels #####
        # y
        ax.yaxis.set_major_locator(plt.MultipleLocator(0.25))
        # x
        ax.set_xticks(cumulative_dt)

        label_list = list(range(cumulative_dt.shape[0]))
        label_list = [e for e in label_list if e not in wanted_time_label]

        xticks = ax.xaxis.get_major_ticks()
        for i_hide in label_list:
            xticks[i_hide].label1.set_visible(False)

        # plot lims
        ax.legend(['$p_{x}$', '$p_{y}$', '$p_{z}$'], fancybox=True)
        ax.set_title(name)
        ax.set_xlabel(r'time [s]')
        ax.set_ylabel(r'pos [m]')
        ax.grid(alpha=0.4)
        ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        plt.xlim([cumulative_dt[0], cumulative_dt[-1]])


    plt.savefig(save_path + "/spot_jump_twist_pos", dpi=500, bbox_inches='tight')
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
