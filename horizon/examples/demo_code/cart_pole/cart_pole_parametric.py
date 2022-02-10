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

# Optimization parameters
tf = 5.0  # [s]
ns = 80  # number of nodes
dt = tf / ns

# options for horizon transcription
transcription_method = 'multiple_shooting' # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='EULER')# integrator used by the multiple_shooting


# Create horizon problem
prb = problem.Problem(ns)

# creates the variables for the problem
# design choice: position and the velocity as state and acceleration as input

# q0: cart x-displacement
# q1: cart y-displacement
# q2: pole rotation

# STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# CONTROL variables
qddot = prb.createInputVariable("qddot", nv)

# create a parameter for a reference trajectory.
# Parameters can be left symbolic when building the problem, and must be set to a specific numerical value before solving.
# Parameters are very useful to avoid hard-coded values or trajectories: once the problem is built, the parameters values
# can be changed at will before solving the problem.
# In this specific case, this parameter is the reference trajectory of the cart pole on the y-axis
q_ref = prb.createParameter('q_ref', 1)

# Create double integrator
x, xdot = utils.double_integrator(q, qdot, qddot)

# Set dynamics of the system and the relative dt
prb.setDynamics(xdot)
prb.setDt(dt)

# ================== Set BOUNDS and INITIAL GUESS  ===============================
# joint limits + initial pos
q_min = [-0.5, -0.5, -2. * np.pi]
q_max = [0.5, 0.5, 2. * np.pi]
q_init = [0., 0., 0.]
# velocity limits + initial vel
qdot_lims = np.array([100., 100., 100.])
qdot_init = [0., 0., 0.]
# acceleration limits
qddot_lims = np.array([1000., 1000., 1000.])
qddot_init = [0., 0., 0.]

# Set bounds
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds((-qdot_lims).tolist(), qdot_lims.tolist())
qdot.setBounds(qdot_init, qdot_init, nodes=0)
qddot.setBounds((-qddot_lims).tolist(), qddot_lims.tolist())

q.setInitialGuess(q_init)
qdot.setInitialGuess(qdot_init)
qddot.setInitialGuess(qddot_init)

# Cost function

# ================== Set TRANSCRIPTION METHOD ===============================
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# ====================== Set CONSTRAINTS ===============================

#
prb.createFinalConstraint("up", q[2] - np.pi)
prb.createFinalConstraint("final_qdot", qdot)

cnrst_ref = prb.createConstraint('sinusoidal_ref', q[1] - q_ref, range(10, ns+1))

# Set dynamic feasibility:
# the cart can apply a torque, the joint connecting the cart to the pendulum is UNACTUATED
# the torques are computed using the inverse dynamics, as the input of the problem is the cart acceleration
tau_lims = np.array([1000., 1000., 0.])
tau = kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)

prb.createIntermediateConstraint("dynamic_feasibility", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))
prb.createIntermediateCost("qddot", cs.sumsqr(qddot))

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