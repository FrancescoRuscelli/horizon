#!/usr/bin/env python3

'''
An example of the cart-pole problem: find the trajectory of the cart so that the pole reaches the upright position.
'''

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, argparse


parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('-replay', help='visualize the robot trajectory in rviz', action='store_true')

args = parser.parse_args()

rviz_replay = False
plot_sol = True

if args.replay:
    from horizon.ros.replay_trajectory import *
    import roslaunch, rospkg, rospy
    rviz_replay = True
    plot_sol = False


# Create CasADi interface to Pinocchio
urdffile = os.path.join(os.getcwd(), 'urdf', 'cart_pole.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# Get dimension of pos and vel
nq = kindyn.nq()
nv = kindyn.nv()

# Optimization parameters
tf = 5.0  # [s]
ns = 100  # number of nodes
dt = tf/ns

# options for horizon transcription
transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='RK4') # integrator used by the multiple_shooting

# Create horizon problem
prb = problem.Problem(ns)

# creates the variables for the problem
# design choice: position and the velocity as state and acceleration as input

# STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)
# CONTROL variables
qddot = prb.createInputVariable("qddot", nv)

# Creates double integrator
x, xdot = utils.double_integrator(q, qdot, qddot)

# Set dynamics of the system and the relative dt
prb.setDynamics(xdot)
prb.setDt(dt)

# Define BOUNDS and INITIAL GUESS
# joint limits + initial pos
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]
# velocity limits + initial vel
qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]
# acceleration limits
qddot_lims = np.array([1000., 1000.])
qddot_init = [0., 0.]

# Set bounds
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
qddot.setBounds(-qddot_lims, qddot_lims)

# Set initial guess
q.setInitialGuess(q_init)
qdot.setInitialGuess(qdot_init)
qddot.setInitialGuess(qddot_init)

# Set transcription method
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# Set dynamic feasibility:
# the cart can apply a torque, the joint connecting the cart to the pendulum is UNACTUATED
# the torques are computed using the inverse dynamics, as the input of the problem is the cart acceleration
tau_lims = np.array([1000., 0.])
tau = kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
iv = prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))

# Set desired constraints
# at the last node, the pendulum is upright
prb.createFinalConstraint("up", q[1] - np.pi)
# at the last node, the system velocity is zero
prb.createFinalConstraint("final_qdot", qdot)

# Set cost functions
# minimize the acceleration of system (regularization of the input)
prb.createIntermediateCost("qddot", cs.sumsqr(qddot))

# Create solver with IPOPT and some desired option
solv = solver.Solver.make_solver('ipopt', prb, opts={'ipopt.tol': 1e-4,'ipopt.max_iter': 2000})

# Solve the problem
solv.solve()

# Get the solution as a dictionary
solution = solv.getSolutionDict()
# Get the array of dt, one for each interval between the nodes
dt_sol = solv.getDt()

########################################################################################################################

if plot_sol:
    time = np.arange(0.0, tf+1e-6, tf/ns)
    plt.figure()
    plt.plot(time, solution['q'][0,:])
    plt.plot(time, solution['q'][1,:])
    plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{[m]}$', size = 20)

    hplt = PlotterHorizon(prb, solution)
    # hplt.plotVariable('q', show_bounds=False)
    # hplt.plotVariable('q_dot', show_bounds=False)
    hplt.plotFunction('inverse_dynamics', dim=[1], show_bounds=True)
    plt.show()

if rviz_replay:

    # set ROS stuff and launchfile
    r = rospkg.RosPack()
    path_to_examples = r.get_path('horizon_examples')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/cart_pole.launch"])
    launch.start()
    rospy.loginfo("'cart_pole' visualization started.")

    # visualize the robot in RVIZ
    joint_list=["cart_joint", "pole_joint"]
    replay_trajectory(tf/ns, joint_list, solution['q']).replay(is_floating_base=False)

else:
    print("To visualize the robot trajectory, start the script with the '--replay' option.")







