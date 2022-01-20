#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os

try:
    from horizon.ros.replay_trajectory import *

    do_replay = True
except ImportError:
    do_replay = False

import os
import time
from horizon.ros import utils as horizon_ros_utils

horizon_ros_utils.roslaunch("horizon_examples", "cart_pole_xy.launch")
time.sleep(3.)

# Loading URDF model in pinocchio
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'cart_pole_xy.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

nq = kindyn.nq()
nv = kindyn.nv()

print("nq: ", nq)
print("nv: ", nv)

# OPTIMIZATION PARAMETERS
tf = 5.0  # [s]
ns = 80  # number of shooting nodes
dt = tf / ns
use_ms = True

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

# Dynamics
if use_ms:
    th = Transcriptor.make_method('multiple_shooting', prb, opts=dict(integrator='EULER'))
else:
    th = Transcriptor.make_method('direct_collocation', prb)  # opts=dict(degree=5)

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

for n in range(ns+1):
    q_ref.assign(cos_fun[n], n)

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

plot_all = False
if plot_all:
    hplt = PlotterHorizon(prb, solution)
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()

if do_replay:
    joint_list = ["cart_joint_x", "cart_joint_y", "pole_joint"]
    replay_trajectory(tf / ns, joint_list, q_hist).replay(is_floating_base=False)
