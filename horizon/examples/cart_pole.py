#!/usr/bin/env python

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, casadi_kin_dyn
from horizon.utils.transcription_methods import TranscriptionsHandler
import matplotlib.pyplot as plt
import os

try:
    from horizon.ros.replay_trajectory import *
    do_replay = True
except ImportError:
    do_replay = False

# Loading URDF model in pinocchio
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'cart_pole.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

nq = kindyn.nq()
nv = kindyn.nv()

print("nq: ", nq)
print("nv: ", nv)

# OPTIMIZATION PARAMETERS
tf = 5.0  # [s]
ns = 100  # number of shooting nodes
dt = tf/ns
use_ms = True

# Create horizon problem
prb = problem.Problem(ns)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)

# Creates double integrator
x, xdot = utils.double_integrator(q, qdot, qddot)
prb.setDynamics(xdot)

# Limits
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]

qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

qddot_lims = np.array([1000., 1000.])
qddot_init = [0., 0.]

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
th = TranscriptionsHandler(prb, dt)
if use_ms:
    th.setDefaultIntegrator(type='EULER')
    th.setMultipleShooting()
else:
    th.setDirectCollocation()

prb.createFinalConstraint("up", q[1] - np.pi)
prb.createFinalConstraint("final_qdot", qdot)


tau_lims = np.array([1000., 0.])
tau = casadi_kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))

# Creates problem
opts = {"nlpsol.ipopt": True}
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
plt.show()

if do_replay:
    joint_list=["cart_joint", "pole_joint"]
    replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)








