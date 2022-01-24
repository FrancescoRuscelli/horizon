#!/usr/bin/env python3

'''
An example of the cart-pole problem: find the trajectory of the cart so that the pole reaches the upright position.

'''

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn, mat_storer, rti
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon
from horizon.solvers import solver, ilqr, blocksqp
import matplotlib.pyplot as plt
import os, argparse, time

parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('-replay', help='visualize the robot trajectory in rviz', action='store_true')

args = parser.parse_args()

rviz_replay = False
plot_sol = True
use_ilqr = True # todo false does not work!

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
ns = 20  # number of nodes
dt = 0.1
tf = ns*dt  # [s]

# options for horizon transcription (NOT USED IF 'use_ilqr' IS True)
transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='RK4') # integrator used by the multiple_shooting


# Create horizon problem
prb = problem.Problem(ns)

# creates the variables for the problem
# design choice: position and the velocity as state and torque as input

# STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)
x = prb.getState().getVars()

# CONTROL variables
u = prb.createInputVariable("u", 1)
tau = cs.vertcat(u, 0)

# Set dynamics of the system and the relative dt
fd = kindyn.aba()  # this is the forward dynamics function
xdot = cs.vertcat(qdot, fd(q=q, v=qdot, tau=tau)['a'])
prb.setDynamics(xdot)
prb.setDt(dt)

# Define LIMITS and INITIAL GUESS
# joint limits + initial pos
q_min = np.array([-1, -2.*np.pi])
q_max = -q_min
q_init = np.array([0., np.pi])
# velocity limits + initial vel
qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]
# torque limits
u_lims = np.array([1000])

# Set limits
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-u_lims, u_lims)

# Set initial guess
q.setInitialGuess(q_init)  # note: this is important!!!

# Final constraint
# at the last node, cart moved forward 0.5m and pendulum is upright
q_tgt = np.array([0.5, np.pi])
prb.createFinalConstraint("qfinal", q - q_tgt)
# at the last node, the system velocity is zero
prb.createFinalConstraint("qdotfinal", qdot)

# Set cost functions
# minimize the velocity
prb.createIntermediateCost("damp", 0.01*cs.sumsqr(qdot))
# minimize the torque of system (regularization of the input)
prb.createIntermediateCost("reg", 1e-6*cs.sumsqr(u))

# Create solver
if use_ilqr:
    # Create solver with 'ilqr' and some desired option
    solver = ilqr.SolverILQR(prb, opts={'realtime_iteration': True})
else:
    # Set transcription method
    th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

    # Create solver with 'blocksqp' and some desired option
    solver = blocksqp.BlockSqpSolver(prb, opts={'realtime_iteration': True})

# the rti loop
rti_dt = 0.01
mpc = rti.RealTimeIteration(prb, solver, rti_dt)
stateread = np.array([0.0, np.pi-0.5, 0.0, 0.0])
states = []
inputs = []
times = []
for i in range(500):
    states.append(stateread.copy())
    tic = time.time()
    input = mpc.run(stateread)
    toc = time.time()
    stateread = mpc.integrate(stateread, input)
    inputs.append(input.copy())
    times.append(toc-tic)

plt.figure()
plt.title('state trajectory')
plt.plot(states)

plt.figure()
plt.title('input trajectory')
plt.plot(inputs)

plt.figure()
plt.title('cpu time')
plt.plot(times)

plt.show()
