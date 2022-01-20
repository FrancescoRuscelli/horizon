#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
import time
from horizon import problem
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.solvers import solver
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

# Dynamics
if use_ms:
    th = Transcriptor.make_method('multiple_shooting', prb, opts=dict(integrator='EULER'))
else:
    th = Transcriptor.make_method('direct_collocation', prb)

blocksqp_opts = {'hess_update': 1,  # 2 = BFGS, 4 = exact
    'warmstart': False,
    'max_iter': 1,
    'print_iteration': False,
    'print_maxit_reached': False,
    'print_header': False,
    'verbose_init': False,
    'print_time': 0,
    'opttol': 1e-4,
    'linsol': 'ma27',
    }

# Creates problem
solver = solver.Solver.make_solver('blocksqp', prb, opts=blocksqp_opts)
# prb.createProblem(solver_plugin='blocksqp', opts=blocksqp_opts)


class RealTimeIteration:

    def __init__(self, prb: problem.Problem, dt: float) -> None:
        # define integrator to simulate the system (should be optional)
        x = prb.getState().getVars()
        u = prb.getInput().getVars()
        xdot = prb.getDynamics()
        dae = {'x': x, 'p': u, 'ode': xdot, 'quad': 0}
        self.integrator = integrators.RK4(dae, {'tf': dt})
        
        self.solution = None
        self.state = prb.getState()

    def run(self, stateread: np.array):
    
        # set initial state
        self.state.setBounds(lb=stateread, ub=stateread, nodes=0)
        
        # solve
        solver.solve()
        self.solution = solver.getSolutionDict()

        # get control input to apply
        u_opt = self.solution['u'][:, 0]

        # integrate input and return
        stateint = self.integrator(x0=stateread, p=u_opt)['xf'].toarray().flatten()
        return stateint, u_opt


# the rti loop
rti = RealTimeIteration(prb=prb, dt=0.01)
stateread = np.array([-0.5, np.pi-0.30, 0.0, 0.0])
states = []
inputs = []
times = []
for i in range(300):
    states.append(stateread.copy())
    tic = time.time()
    stateread, input = rti.run(stateread)
    toc = time.time()
    inputs.append(input)
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