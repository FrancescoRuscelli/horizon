#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
import logging
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn, resampler_trajectory, plotter
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
ns = 100  # number of shooting nodes
use_ms = True

# Create horizon problem
prb = problem.Problem(ns)

# Create problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Create problem CONTROL variables
u = prb.createInputVariable("u", 1)
tau = cs.vertcat(u, 0)

# Create dynamics
fd = kindyn.aba()  # this is the forward dynamics function
x = cs.vertcat(q, qdot)
xdot = cs.vertcat(qdot, fd(q=q, v=qdot, tau=tau)['a'])

# Quantities at previous time step (to be removed!)
q_prev = q.getVarOffset(-1)
qdot_prev = qdot.getVarOffset(-1)
u_prev = u.getVarOffset(-1)
x_prev = cs.vertcat(q_prev, qdot_prev)

# Formulate discrete time dynamics (for multiple shooting)
tf = 5.0  # [s]
L = 0.5*cs.dot(u, u)  # Objective term
dae = {'x': x, 'p': u, 'ode': xdot, 'quad': L}
opts = {'tf': tf/ns}
F_integrator = integrators.RK4(dae, opts)

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
prb.createCostFunction("tau", cs.sumsqr(tau), nodes=range(ns))

# Constraints
if use_ms:
    x_int = F_integrator(x0=x_prev, p=u_prev)
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns+1)), bounds=dict(lb=np.zeros(nv+nq).tolist(), ub=np.zeros(nv+nq).tolist()))
else:
    integrators.make_direct_collocation(prob=prb, x=x, x_prev=x_prev, xdot=xdot, degree=3, dt=tf/ns)
prb.createConstraint("up", q[1] - np.pi, nodes=ns, bounds=dict(lb = [0], ub = [0]))
prb.createConstraint("final_qdot", qdot - np.array([0., 0.]), nodes=ns, bounds=dict(lb = np.zeros(2).tolist(), ub = np.zeros(2).tolist()))

# Creates problem
opts = {"nlpsol.ipopt":True}
prb.createProblem(opts)

solution = prb.solveProblem()
q_hist = solution["q"]

time = np.arange(0.0, tf+1e-6, tf/ns)
plt.figure()
plt.plot(time, q_hist[0,:])
plt.plot(time, q_hist[1,:])
plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
plt.xlabel('$\mathrm{[sec]}$', size = 20)
plt.ylabel('$\mathrm{[m]}$', size = 20)

# plot
plt = plotter.PlotterHorizon(sol=solution)
plt.plotVariables()

joint_list=["cart_joint", "pole_joint"]
replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)