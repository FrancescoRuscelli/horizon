#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os

rviz_replay = True
plot_sol = True

# Loading URDF model in pinocchio
path_to_examples = os.path.abspath(__file__ + "/../../../")
os.environ['ROS_PACKAGE_PATH'] += ':' + path_to_examples

urdffile = os.path.join(path_to_examples, 'urdf', 'cart_pole_xy.urdf')
urdf = open(urdffile, 'r').read()

# Create casadi interface to pinocchio
kindyn = pycasadi_kin_dyn.CasadiKinDyn(urdf)
nq = kindyn.nq()
nv = kindyn.nv()

# OPTIMIZATION PARAMETERS
tf = 5.0  # [s]
ns = 80  # number of shooting nodes
dt = tf / ns

transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='EULER')  # integrator used by the multiple_shooting


# Create horizon problem
prb = problem.Problem(ns)

# Creates problem STATE variables
# q1: cart displacement
# q2: pole rotation
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)

q_ref = prb.createParameter('q_ref', 1)
# Creates double integrator
x, xdot = utils.double_integrator(q, qdot, qddot)
prb.setDynamics(xdot)
prb.setDt(dt)

# Limits
q_min = [-0.5, -0.5, -2. * np.pi]
q_max = [0.5, 0.5, 2. * np.pi]
q_init = [0., 0., 0.]

qdot_lims = np.array([100., 100., 100.])
qdot_init = [0., 0., 0.]

qddot_lims = np.array([1000., 1000., 1000.])
qddot_init = [0., 0., 0.]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds((-qdot_lims).tolist(), qdot_lims.tolist())
qdot.setBounds(qdot_init, qdot_init, nodes=0)
qddot.setBounds((-qddot_lims).tolist(), qddot_lims.tolist())

q.setInitialGuess(q_init)
qdot.setInitialGuess(qdot_init)
qddot.setInitialGuess(qddot_init)

# Cost function
prb.createIntermediateCost("qddot", cs.sumsqr(qddot))

th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

prb.createFinalConstraint("up", q[2] - np.pi)
prb.createFinalConstraint("final_qdot", qdot)

cnrst_ref = prb.createConstraint('sinusoidal_ref', q[1] - q_ref, range(10, ns+1))

# setting underactuation
tau_lims = np.array([1000., 1000., 0.])
tau = kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))

# Creates problem
solver = solver.Solver.make_solver('ipopt', prb, opts={'ipopt.tol': 1e-4, 'ipopt.max_iter': 2000})

cos_fun = 1/3 * np.cos(np.linspace(np.pi/2, 4*2*np.pi, ns+1))
cos_fun = np.atleast_2d(cos_fun)
q_ref.assign(cos_fun)


solver.solve()
solution = solver.getSolutionDict()
q_hist = solution['q']

time = np.arange(0.0, tf + 1e-6, tf / ns)
plt.figure()
plt.plot(time, q_hist[0, :])
plt.plot(time, q_hist[1, :])
plt.suptitle('$\mathrm{Base \ Position}$', size=20)
plt.xlabel('$\mathrm{[sec]}$', size=20)
plt.ylabel('$\mathrm{[m]}$', size=20)


if plot_sol:
    hplt = PlotterHorizon(prb, solution)
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()

if rviz_replay:

    from horizon.ros.replay_trajectory import replay_trajectory
    import roslaunch, rospy
    # set ROS stuff and launchfile

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/cart_pole_xy.launch"])
    launch.start()
    rospy.loginfo("'cart_pole_final_time' visualization started.")

    # visualize the robot in RVIZ
    joint_list = ["cart_joint_x", "cart_joint_y", "pole_joint"]
    replay_trajectory(tf/ns, joint_list, solution['q']).replay(is_floating_base=False)

else:
    print("To visualize the robot trajectory, start the script with the '--replay' option.")
