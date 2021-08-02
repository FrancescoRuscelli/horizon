#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
import time
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn, resampler_trajectory, plotter
from horizon.ros.replay_trajectory import replay_trajectory
from horizon.solvers import sqp

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
tf = ns * dt  # [s]
L = 0.5 * cs.dot(u, u)  # Objective term
dae = {'x': x, 'p': u, 'ode': xdot, 'quad': L}
opts = {'tf': dt}
F_integrator = integrators.EULER(dae, opts)

# Limits
q_min = np.array([-0.5, -2. * np.pi])
q_max = np.array([0.5, 2. * np.pi])
q_init = np.array([0., np.pi - 0.01])

qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

tau_lims = np.array([1000])
tau_init = [0]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
q.setInitialGuess(q_init)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-tau_lims, tau_lims)

# Cost function
prb.createCostFunction("track", 10.*(q - q_init), nodes=range(ns + 1))
prb.createCostFunction("damp", 10. * qdot, nodes=range(ns + 1))
prb.createCostFunction("reg", 0.02 * u, nodes=range(ns))

# Final constraint
prb.createConstraint("qfinal", q - q_init, nodes=ns)
prb.createConstraint("qdotfinal", qdot, nodes=ns)

# Dynamics
if use_ms:
    x_int = F_integrator(x0=x_prev, p=u_prev)
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=range(1, ns + 1))
else:
    integrators.make_direct_collocation(prob=prb, x=x, x_prev=x_prev, xdot=xdot, degree=3, dt=tf / ns)

# Creates problem
prb.createProblem()

problem_dict = prb.getProblem()
problem_dict['f'] = prb.function_container.getCostFList()
d = {'verbose': False}
opts = {'max_iter': 1,
        'osqp.osqp': d}
solver = sqp.sqp('solver', 'osqp', problem_dict, opts)
prb.setSolver(solver)

class RealTimeIteration:

    def __init__(self, dt: float) -> None:
        self.integrator = integrators.RK4(dae, {'tf': dt})

    def run(self, stateread: np.array):
        # set initial state
        qread = stateread[0:2]
        qdotread = stateread[2:4]
        q.setBounds(lb=qread, ub=qread, nodes=0)
        qdot.setBounds(lb=qdotread, ub=qdotread, nodes=0)

        # solve
        solution = prb.solveProblem()

        # get control input to apply
        u_opt = solution['u'][:, 0]

        # integrate input
        stateint = self.integrator(x0=stateread, p=u_opt)['xf'].toarray().flatten()
        return stateint, u_opt


# the rti loop
rti = RealTimeIteration(dt=0.01)
stateread = np.array([0, np.pi - 0.30, 0.0, 0.0])
states = []
inputs = []
times = []
for i in range(300):
    states.append(stateread.copy())
    tic = time.time()
    stateread, input = rti.run(stateread)
    toc = time.time()
    inputs.append(input)
    times.append(toc - tic)

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