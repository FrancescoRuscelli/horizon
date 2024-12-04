#!/usr/bin/env python3
import logging

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
    # susendberg's function
    return v.lower() in ("yes", "true", "t", "1")



action = 'jump_up'
rviz_replay = True
solver_type = 'ipopt'
codegen = False
warmstart_flag = False
plot_sol = False


resampling = False
load_initial_guess = False

if rviz_replay:
    from horizon.ros.replay_trajectory import replay_trajectory
    import rospy
    plot_sol = False

path_to_examples = os.path.abspath(__file__ + "/../../../examples")



# options
transcription_method = 'multiple_shooting'
transcription_opts = dict(integrator='RK4')

tf = 2.5
n_nodes = 50

disp = [0., 0., 0., 0., 0., 0., 1.]


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
prb = problem.Problem(n_nodes, logging_level=logging.DEBUG)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)
f_list = [prb.createInputVariable(f'force_{i}', n_f) for i in contacts_name]
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)
# contact map
contact_map = dict(zip(contacts_name, f_list))

# initial state and initial guess
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

q.setBounds(q_init, q_init, 0)
q_dot.setBounds(np.zeros(n_v), np.zeros(n_v), 0)

q.setInitialGuess(q_init)


[f.setInitialGuess([0, 0, 55]) for f in f_list]

# transcription

th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# dynamic feasibility
id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
tau = id_fn.call(q, q_dot, q_ddot, contact_map)
prb.createIntermediateConstraint("dynamic_feasibility", tau[:6])

# final velocity is zero
prb.createFinalConstraint('final_velocity', q_dot)

# contact handling
k_all = range(1, n_nodes + 1)
k_swing = list(range(*[node for node in node_action]))
k_stance = list(filterfalse(lambda k: k in k_swing, k_all))

# list of lifted legs
lifted_legs = ['lf_foot', 'rf_foot']
lifted_legs.extend(['lh_foot', 'rh_foot'])

q_final = q_init
q_final[:3] = q_final[:3] + disp[:3]
q_final[3:7] = disp[3:7]

def barrier(x):
    return cs.sum1(cs.if_else(x > 0, 0, x ** 2))

for frame, f in contact_map.items():
    nodes_stance = k_stance if frame in lifted_legs else k_all


    FK = cs.Function.deserialize(kindyn.fk(frame))
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))

    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    prb.createConstraint(f"{frame}_vel", v, nodes=nodes_stance)
    prb.createIntermediateCost(f'{frame}_fn', barrier(f[2] - 25.0))



# swing force is zero
for leg in lifted_legs:
    nodes = k_swing
    fzero = np.zeros(n_f)
    contact_map[leg].setBounds(fzero, fzero, nodes=nodes)


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

solv = solver.Solver.make_solver(solver_type, prb, opts)
solv.solve()

solution = solv.getSolutionDict()
dt_sol = solv.getDt()
cumulative_dt = np.zeros(len(dt_sol) + 1)
for i in range(len(dt_sol)):
    cumulative_dt[i + 1] = dt_sol[i] + cumulative_dt[i]

solution_constraints_dict = dict()

# ========================================================
if plot_sol:
    import matplotlib.pyplot as plt
    from matplotlib import gridspec

    hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables(show_bounds=True, same_fig=True, legend=False)
    hplt.plotVariables([elem.getName() for elem in f_list], show_bounds=True, gather=2, legend=False)
    # hplt.plotFunctions(show_bounds=True, same_fig=True)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))

    pos_contact_list = list()
    fig = plt.figure()
    fig.suptitle('Contacts')
    gs = gridspec.GridSpec(2, 2)
    i = 0
    for contact in contacts_name:
        ax = fig.add_subplot(gs[i])
        ax.set_title('{}'.format(contact))
        i += 1
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']
        for dim in range(n_f):
            ax.plot(np.atleast_2d(cumulative_dt), np.array(pos[dim, :]), marker="x", markersize=3,
                    linestyle='dotted')  # marker="x", markersize=3, linestyle='dotted'

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'feet position - plane_xy')
        plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'feet position - plane_xz')
        plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)

    plt.show()
# ======================================================
contact_map = {contacts_name[i]: solution[f_list[i].getName()] for i in range(n_c)}

if rviz_replay:

    try:
        # set ROS stuff and launchfile
        import subprocess
        os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples
        subprocess.Popen(["roslaunch", path_to_examples + "/replay/launch/launcher.launch", 'robot:=spot'])
        rospy.loginfo("'spot' visualization started.")

    except:
        print('Failed to automatically run RVIZ. Launch it manually.')
        # remember to run a robot_state_publisher
    repl = replay_trajectory(dt, joint_names, solution['q'], contact_map,
                             cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

    repl.sleep(1.)
    repl.replay(is_floating_base=True)

else:
    print("To visualize the robot trajectory, start the script with the '--replay")