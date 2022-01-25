#!/usr/bin/env python
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter
from horizon.transcriptions import integrators
from horizon.solvers import solver
from horizon.ros.replay_trajectory import *
import matplotlib.pyplot as plt
import os, argparse
import time
from horizon.ros import utils as horizon_ros_utils


parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('--replay', help='visualize the robot trajectory in rviz', action='store_true')
args = parser.parse_args()
# Switch between suspended and free fall
rviz_replay = False
plot_sol = True
resample = False
rope_mode = 'swing' # 'swing' # 'free_fall' # 'fixed'


if args.replay:
    from horizon.ros.replay_trajectory import *
    import roslaunch, rospkg, rospy
    rviz_replay = True
    resample = True
    plot_sol = False

# Loading URDF model in pinocchio
urdffile = os.path.join(os.getcwd(), 'urdf', 'roped_template.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# OPTIMIZATION PARAMETERS
ns = 75 # number of nodes
nc = 3  # number of contacts
nq = kindyn.nq()  # number of DoFs - NB: 7 DoFs floating base (quaternions)
DoF = nq - 7  # Contacts + anchor_rope + rope
nv = kindyn.nv()  # Velocity DoFs
nf = 3  # 2 feet contacts + rope contact with wall, Force DOfs

# Create horizon problem
prb = problem.Problem(ns)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)
f1 = prb.createInputVariable("f1", nf)
f2 = prb.createInputVariable("f2", nf)
frope = prb.createInputVariable("frope", nf)

# Creates double integrator
x, xdot = utils.double_integrator_with_floating_base(q, qdot, qddot)

if rope_mode == 'swing':
    tf = 2. # [s]
else:
    tf = 1. # [s]

dt = tf / ns

prb.setDynamics(xdot)
prb.setDt(dt)


# Formulate discrete time dynamics
L = 0.5 * cs.sumsqr(qdot)  # Objective term
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
if rope_mode == 'swing':
    F_integrator_LEAPFROG = integrators.LEAPFROG(dae)
    F_integrator = integrators.RK4(dae)
else:
    F_integrator = integrators.RK4(dae)

# limits
q_min = kindyn.q_min()
q_max = kindyn.q_max()
q_min[:3] = [-10.0, -10.0, -10.0]
q_max[:3] = [10.0, 10.0, 10.0]
q_min[3:7] = -np.ones(4)
q_max[3:7] = np.ones(4)

if rope_mode != 'free_fall':
    q_min[-1] = 0.3
    q_max[-1] = 0.3

if rope_mode == 'swing':
    q_min[7:13] = np.zeros(6)
    q_max[7:13] = np.zeros(6)


q.setBounds(q_min, q_max)

qdot_min = -1000.*np.ones(nv)
qdot_max = -qdot_min
qdot.setBounds(qdot_min, qdot_max)

qddot_min = -1000.*np.ones(nv)
qddot_max = -qddot_min
qddot.setBounds(qddot_min, qddot_max)

if rope_mode == 'swing':
    f_min = np.zeros(nf)
    f_max = f_min
else:
    f_min = -10000.*np.ones(nf)
    f_max = -f_min


frope_min = -10000. * np.ones(nf)
frope_max = -frope_min

f1.setBounds(f_min, f_max)
f2.setBounds(f_min, f_max)
frope.setBounds(frope_min, frope_max)

q_init = [0., 0., 0., 0., 0., 0., 1.0,
          0., 0., 0.,
          0., 0., 0.,
          0., 0., 0.,
          0.3]

if rope_mode == 'swing':
    # starting from a tilted position
    q_init[14] = 0.3 # rope_anchor_y

q.setInitialGuess(q_init)

qdot_init = np.zeros(nv)
qdot.setInitialGuess(qdot_init)

qddot_init = np.zeros(nv)
qddot.setInitialGuess(qdot_init)

f_init = np.zeros(nf)
f1.setInitialGuess(f_init)
f2.setInitialGuess(f_init)
frope.setInitialGuess(f_init)

state = prb.getState()
input = prb.getInput()
state_prev = state.getVarOffset(-1)
input_prev = input.getVarOffset(-1)

x_prev, _ = utils.double_integrator_with_floating_base(state_prev[0], state_prev[1], input_prev[0])
x_int = F_integrator(x0=x_prev, p=input_prev[0], time=dt)

if rope_mode == 'swing':
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=1)
    q_pprev = q.getVarOffset(-2)
    qdot_pprev = qdot.getVarOffset(-2)
    qddot_pprev = qddot.getVarOffset(-2)
    x_pprev, _ = utils.double_integrator_with_floating_base(q_pprev, qdot_pprev, qddot_pprev)
    x_int2 = F_integrator_LEAPFROG(x0=x_prev, x0_prev=x_pprev, p=input_prev[0], time=dt)
    prb.createConstraint("multiple_shooting2", x_int2["xf"] - x, nodes=range(2, ns + 1))
else:
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=range(1, ns+1))


# Constraints
prb.createConstraint("q_init", q - q_init, nodes=0)
prb.createConstraint("qdot_init", qdot - qdot_init, nodes=0)


tau_min = np.array([0., 0., 0., 0., 0., 0.,  # floating base
                    -1000., -1000., -1000.,  # contact 1
                    -1000., -1000., -1000.,  # contact 2
                    0., 0., 0.,  # rope anchor point
                    0.])  # rope

tau_max = - tau_min

if rope_mode != 'free_fall':
    tau_min[-1] = -10000.

frame_force_mapping = {'rope_anchor2': frope}
id = kin_dyn.InverseDynamics(kindyn, ['rope_anchor2'], cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
tau = id.call(q, qdot, qddot, frame_force_mapping)

prb.createConstraint("inverse_dynamics", tau, nodes=range(0, ns), bounds=dict(lb=tau_min, ub=tau_max))

FKRope = cs.Function.deserialize(kindyn.fk('rope_anchor2'))

p_rope_init = FKRope(q=q_init)['ee_pos']
p_rope = FKRope(q=q)['ee_pos']
prb.createConstraint("rope_anchor_point", p_rope - p_rope_init)


# Cost function
if rope_mode == 'swing':
    weigth_joint_vel = 1.
else:
    weigth_joint_vel = 100.

prb.createCost("min_joint_vel", weigth_joint_vel*cs.sumsqr(qdot))

if rope_mode != 'swing':
    prb.createCost("min_joint_acc", 1000.*cs.sumsqr(qddot[6:-1]), range(1, ns))
    prb.createCost("min_f1", 1000.*cs.sumsqr(f1), range(1, ns))
    prb.createCost("min_f2", 1000.*cs.sumsqr(f2), range(1, ns))

    frope_prev = frope.getVarOffset(-1)
    prb.createCost("min_dfrope", 1000.*cs.sumsqr(frope-frope_prev), range(1, ns))

# Creates problem
opts = {'ipopt.tol': 1e-3,
        'ipopt.constr_viol_tol': 1e-3,
        'ipopt.max_iter': 2000} #

solver = solver.Solver.make_solver('ipopt', prb, opts)
solver.solve()

solution = solver.getSolutionDict()

# ======================================================================================================================
time = np.arange(0.0, tf + 1e-6, dt)

tau_sol = np.zeros(solution["qddot"].shape)
ID = kin_dyn.InverseDynamics(kindyn, ['Contact1', 'Contact2', 'rope_anchor2'])
for i in range(ns):
    frame_force_mapping_i = {'Contact1': solution["f1"][:, i], 'Contact2': solution["f2"][:, i], 'rope_anchor2':  solution["frope"][:, i]}
    tau_sol[:, i] = ID.call(solution["q"][:, i], solution["qdot"][:, i], solution["qddot"][:, i], frame_force_mapping_i).toarray().flatten()


if resample:
    # resampling
    dt_res = 0.001
    frame_force_hist_mapping = {'Contact1': solution["f1"], 'Contact2': solution["f2"], 'rope_anchor2': solution["frope"]}
    q_res, qdot_res, qddot_res, frame_force_res_mapping, tau_res = resampler_trajectory.resample_torques(solution["q"], solution["qdot"], solution["qddot"], dt, dt_res, dae, frame_force_hist_mapping, kindyn)
    time_res = np.arange(0.0, q_res.shape[1] * dt_res - dt_res, dt_res)

if plot_sol:
    # plots raw solution

    # hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables()
    # hplt.plotFunctions()

    plt.figure()
    for i in range(0, 3):
        plt.plot(time, solution["q"][i,:])
    plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{[m]}$', size = 20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], solution["qddot"][i,:])
    plt.suptitle('$\mathrm{Base \ Acceleration}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{ [m] } /  \mathrm{ [sec^2] } $', size = 20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], solution["f1"][i, :])
        plt.plot(time[:-1], solution["f2"][i, :])
        plt.plot(time[:-1], solution["frope"][i, :])
    plt.suptitle('$\mathrm{force \ feet \ and \ rope}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] } /  \mathrm{ [sec^2] } $', size=20)

    plt.figure()
    for i in range(0, 6):
        plt.plot(time[:-1], tau_sol[i, :])
    plt.suptitle('$\mathrm{base \ force}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] }} $', size=20)

    if resample:

        plt.figure()
        for i in range(0, 3):
            plt.plot(time_res, q_res[i, :])
        plt.suptitle('$\mathrm{Base \ Position \ Resampled}$', size=20)
        plt.xlabel('$\mathrm{[sec]}$', size=20)
        plt.ylabel('$\mathrm{[m]}$', size=20)

        plt.figure()
        for i in range(0, 3):
            plt.plot(time_res[:-1], qddot_res[i, :])
        plt.suptitle('$\mathrm{Base \ Acceleration \ Resampled}$', size=20)
        plt.xlabel('$\mathrm{[sec]}$', size=20)
        plt.ylabel('$\mathrm{ [m] } /  \mathrm{ [sec^2] } $', size=20)

        plt.figure()
        f1_res = frame_force_res_mapping["Contact1"]
        f2_res = frame_force_res_mapping["Contact2"]
        frope_res = frame_force_res_mapping["rope_anchor2"]
        for i in range(0, 3):
            plt.plot(time_res[:-1], f1_res[i, :])
            plt.plot(time_res[:-1], f2_res[i, :])
            plt.plot(time_res[:-1], frope_res[i, :])
        plt.suptitle('$\mathrm{force \ feet \ and \ rope \ resampled}$', size=20)
        plt.xlabel('$\mathrm{[sec]}$', size=20)
        plt.ylabel('$\mathrm{ [N] } /  \mathrm{ [sec^2] } $', size=20)

        plt.figure()
        for i in range(0, 6):
            plt.plot(time_res[:-1], tau_res[i, :], '-x')
        plt.suptitle('$\mathrm{base \ force \ resampled}$', size=20)
        plt.xlabel('$\mathrm{[sec]}$', size=20)
        plt.ylabel('$\mathrm{ [N] }} $', size=20)

    plt.show()


if rviz_replay:

    r = rospkg.RosPack()
    path_to_examples = r.get_path('horizon_examples')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/roped_template.launch"])
    launch.start()
    rospy.loginfo("'cart_pole_fd' visualization started.")

    joint_list = ['Contact1_x', 'Contact1_y', 'Contact1_z',
                  'Contact2_x', 'Contact2_y', 'Contact2_z',
                  'rope_anchor1_1_x', 'rope_anchor1_2_y', 'rope_anchor1_3_z',
                  'rope_joint']

    replay_trajectory(dt_res, joint_list, q_res, frame_force_res_mapping).replay()

else:
    print("To visualize the robot trajectory, start the script with the '--replay")
