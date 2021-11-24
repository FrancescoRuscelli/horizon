#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils import plotter
from horizon.solvers import Solver
from horizon.ros.replay_trajectory import replay_trajectory
from horizon.ros import utils as horizon_ros_utils

import matplotlib.pyplot as plt
import os
import time


horizon_ros_utils.roslaunch("horizon_examples", "cart_pole.launch")
time.sleep(3.)

# Loading URDF model in pinocchio
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'cart_pole.urdf')
urdf = open(urdffile, 'r').read()

# Create casadi interface to pinocchio
kindyn = pycasadi_kin_dyn.CasadiKinDyn(urdf)
nq = kindyn.nq()
nv = kindyn.nv()

# OPTIMIZATION PARAMETERS
ns = 100  # number of shooting nodes
tf = 5.0  # [s]
dt = tf/ns
use_ms = True

# Create horizon problem
prb = problem.Problem(ns)

# Create problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)
x = prb.getState().getVars()

# Create problem CONTROL variables
u = prb.createInputVariable("u", 1)
tau = cs.vertcat(u, 0)

# Create dynamics
fd = kindyn.aba()  # this is the forward dynamics function
xdot = cs.vertcat(qdot, fd(q=q, v=qdot, tau=tau)['a'])
prb.setDynamics(xdot)
prb.setDt(dt)
# Limits
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]

qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

tau_lims = np.array([3000])
tau_init = [0]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-tau_lims, tau_lims)

# Cost function
prb.createIntermediateCost("tau", cs.sumsqr(tau))

# Dynamics
if use_ms:
    th = Transcriptor.make_method('multiple_shooting', prb, opts=dict(integrator='EULER'))
else:
    th = Transcriptor.make_method('direct_collocation', prb) # opts=dict(degree=5)


# Constraints
prb.createFinalConstraint("up", q[1] - np.pi)
prb.createFinalConstraint("final_qdot", qdot)

# Creates problem
solver = Solver.make_solver('ipopt', prb)  #, opts={'max_iter': 10})
solver.solve()
solution = solver.getSolutionDict()
q_hist = solution["q"]

time = np.arange(0.0, tf+1e-6, tf/ns)
plt.figure()
plt.plot(time, q_hist[0,:])
plt.plot(time, q_hist[1,:])
plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
plt.xlabel('$\mathrm{[sec]}$', size = 20)
plt.ylabel('$\mathrm{[m]}$', size = 20)

plot_all = True
if plot_all:
    hplt = plotter.PlotterHorizon(prb, solution)
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()

joint_list=["cart_joint", "pole_joint"]
replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)