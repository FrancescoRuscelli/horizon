#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
import time
from horizon import problem
from horizon.solvers import ilqr, blocksqp
from horizon.utils import rti
from horizon.transcriptions.transcriptor import Transcriptor
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
dt = 0.1
tf = ns*dt  # [s]
use_ms = True
use_ilqr = True

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
q_min = np.array([-1, -2.*np.pi])
q_max = -q_min
qdot_lims = np.array([100., 100.])
u_lims = np.array([1000])

# Initial values
q_init = np.array([0., np.pi])
qdot_init = [0., 0.]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
q.setInitialGuess(q_init)  # note: this is important!!!
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-u_lims, u_lims)

# Cost function
q_tgt = np.array([0.5, np.pi])  # forward 0.5m, and stay up!
prb.createIntermediateCost("damp", 0.01*cs.sumsqr(qdot))
prb.createIntermediateCost("reg", 1e-6*cs.sumsqr(u))

# Final constraint
prb.createFinalConstraint("qfinal", q - q_tgt)
prb.createFinalConstraint("qdotfinal", qdot)

# Create solver
if use_ilqr:
    solver = ilqr.SolverILQR(prb, opts={'realtime_iteration': True})
else:
    # Dynamics
    if use_ms:
        th = Transcriptor.make_method('multiple_shooting', prb, opts=dict(integrator='EULER'))
    else:
        th = Transcriptor.make_method('direct_collocation', prb)  # opts=dict(degree=5)

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
