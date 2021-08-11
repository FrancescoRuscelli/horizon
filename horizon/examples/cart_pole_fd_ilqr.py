#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.solvers import Solver
from horizon.utils.transcription_methods import TranscriptionsHandler
from horizon.utils import plotter
from horizon.ros.replay_trajectory import replay_trajectory

import matplotlib.pyplot as plt
import os

# Loading URDF model in pinocchio
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'cart_pole.urdf')
urdf = open(urdffile, 'r').read()

# Create casadi interface to pinocchio
kindyn = pycasadi_kin_dyn.CasadiKinDyn(urdf)
nq = kindyn.nq()
nv = kindyn.nv()

# OPTIMIZATION PARAMETERS
ns = 20  # number of shooting nodes
tf = 3.0  # [s]
dt = tf/ns
use_ms = True
solver_type = 'ilqr'

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

# Limits
q_min = [-1, -2.*np.pi]
q_max = [1, 2.*np.pi]
q_init = [0.0, np.pi-0.1]

qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

tau_lims = np.array([3000])
tau_init = [0]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
q.setInitialGuess(q_init)
qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-tau_lims, tau_lims)

# Cost function
qtgt = np.array([0.5, np.pi])
prb.createIntermediateCost("err", cs.sumsqr(q - qtgt))
prb.createIntermediateCost("tau", 1e-6*cs.sumsqr(tau))
# prb.createIntermediateCost("qdot", cs.sumsqr(qdot))

if solver_type != 'ilqr':
    # Dynamics
    th = TranscriptionsHandler(prb, dt)
    if use_ms:
        th.setDefaultIntegrator(type='EULER')
        th.setMultipleShooting()
    else:
        th.setDirectCollocation()

# Constraints
prb.createFinalConstraint("up", q[1] - qtgt[1])
prb.createFinalConstraint("center", q[0] - qtgt[0])
prb.createFinalConstraint("final_qdot", qdot)

# Creates problem
solver = Solver.make_solver(solver_type, prb, dt)  #, opts={'max_iter': 10})
solver.ilqr.setStepLength(1)
solver.solve()
q_hist = solver.x_opt[:2, :]
qdot_hist = solver.x_opt[2:4, :]

time = np.arange(0.0, tf+1e-6, tf/ns)
plt.figure()
plt.plot(time, solver.x_opt[:2,:].T)
plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
plt.xlabel('$\mathrm{[sec]}$', size = 20)
plt.ylabel('$\mathrm{[m]}$', size = 20)

plt.figure()
plt.plot(time, solver.x_opt[2:,:].T)
plt.suptitle('$\mathrm{Base \ Velocity}$', size = 20)
plt.xlabel('$\mathrm{[sec]}$', size = 20)
plt.ylabel('$\mathrm{[ms^-1]}$', size = 20)

plt.figure()
plt.plot(time[:-1], solver.u_opt.T)
plt.suptitle('$\mathrm{Force}$', size = 20)
plt.xlabel('$\mathrm{[sec]}$', size = 20)
plt.ylabel('$\mathrm{[N]}$', size = 20)
plt.show()

# plot_all = True
# if plot_all:
#     hplt = plotter.PlotterHorizon(prb)
#     hplt.plotVariables()
#     hplt.plotFunctions()

# joint_list=["cart_joint", "pole_joint"]
# replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)