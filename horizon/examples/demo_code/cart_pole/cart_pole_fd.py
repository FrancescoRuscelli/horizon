#!/usr/bin/env python3

'''
An example of the cart-pole problem: find the trajectory of the cart so that the pole reaches the upright position.
(difference from 'cart_pole.py': system inputs are the torques, not the accelerations)
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
# cart joint is actuated, while the pole joint is unactuated
tau = prb.createInputVariable("u", 2)

# Set dynamics of the system and the relative dt
fd = kindyn.aba()  # this is the forward dynamics function
xdot = cs.vertcat(qdot, fd(q=q, v=qdot, tau=tau)['a'])
prb.setDynamics(xdot)
prb.setDt(dt)

# Define LIMITS and INITIAL GUESS
# joint limits + initial pos
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]
# velocity limits + initial vel
qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]
# torque limits
tau_lims = np.array([3000, 0])
tau_init = [0, 0]

# Set limits
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
tau.setBounds(-tau_lims, tau_lims)

# Set initial guess
q.setInitialGuess(q_init)
qdot.setInitialGuess(qdot_init)
tau.setInitialGuess(tau_init)

# Set transcription method
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# Set desired constraints
# at the last node, the pendulum is upright
prb.createFinalConstraint("up", q[1] - np.pi)
# at the last node, the system velocity is zero
prb.createFinalConstraint("final_qdot", qdot)

# Set cost functions
# minimize the torque of system (regularization of the input)
prb.createIntermediateCost("tau", cs.sumsqr(tau))

# Create solver with IPOPT and some desired option
solv = solver.Solver.make_solver('ipopt', prb)

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
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()

if rviz_replay:

    # set ROS stuff and launchfile
    r = rospkg.RosPack()
    path_to_examples = r.get_path('horizon_examples')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/cart_pole.launch"])
    launch.start()
    rospy.loginfo("'cart_pole_fd' visualization started.")

    # visualize the robot in RVIZ
    joint_list=["cart_joint", "pole_joint"]
    replay_trajectory(tf/ns, joint_list, solution['q']).replay(is_floating_base=False)

else:
    print("To visualize the robot trajectory, start the script with the '--replay' option.")