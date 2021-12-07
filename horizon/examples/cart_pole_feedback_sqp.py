#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
import time

from horizon import problem
from horizon.transcriptions import integrators
from horizon.solvers import solver, pysqp
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
prb.setDynamics(xdot)
prb.setDt(dt)

# Quantities at previous time step (to be removed!)
q_prev = q.getVarOffset(-1)
qdot_prev = qdot.getVarOffset(-1)
u_prev = u.getVarOffset(-1)
x_prev = cs.vertcat(q_prev, qdot_prev)

# Formulate discrete time dynamics (for multiple shooting)
tf = ns * dt  # [s]
L = 0.5 * cs.dot(u, u)  # Objective term
dae = {'x': x, 'p': u, 'ode': xdot, 'quad': L}
F_integrator = integrators.EULER(dae)

# Limits
q_min = np.array([-1, -2.*np.pi])
q_max = -q_min
q_init = np.array([0., np.pi])

qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

tau_lims = np.array([1000])
tau_init = [0]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
q.setInitialGuess(q_init)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
u.setBounds(-tau_lims, tau_lims)

q_tgt = np.array([0.5, np.pi])  # forward 0.5m, and stay up!

# Cost function
prb.createIntermediateCost("tau", 1e-2*u)
prb.createIntermediateCost("qdot", 2.*qdot)
prb.createIntermediateCost("qfinal", 1e-3*(q - q_tgt))
prb.createFinalCost("qdot_f", 0.1*qdot)
prb.createFinalCost("qfinal_f", 1.*(q - q_tgt))

# Final constraint
prb.createFinalConstraint("up", q[1] - q_tgt[1])
prb.createFinalConstraint("center", q[0] - q_tgt[0])
prb.createFinalConstraint("final_qdot", qdot)

# Dynamics
if use_ms:
    x_int = F_integrator(x0=x_prev, p=u_prev, time=dt)
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=range(1, ns + 1))
else:
    prb.setDynamics(xdot)
    integrators.make_direct_collocation(prob=prb, degree=3, dt=tf / ns)

# Creates problem


opts = dict()
qp_solver = "osqp"

opts['gnsqp.qp_solver'] = qp_solver

if qp_solver == "qpoases":
    opts['sparse'] = True
    opts['hessian_type'] = 'posdef'
    opts['printLevel'] = 'none'

if qp_solver == "osqp":
    opts['warm_start_primal'] = True
    opts['warm_start_dual'] = True
    opts['osqp.polish'] = False
    opts['osqp.verbose'] = False

opts['max_iter'] = 1


solver = solver.Solver.make_solver('gnsqp', prb, opts)
solver.set_iteration_callback()


class RealTimeIteration:

    def __init__(self, dt: float) -> None:
        self.integrator = integrators.RK4(dae)
        self.dt = dt

    def run(self, stateread: np.array):
        # set initial state
        qread = stateread[0:2]
        qdotread = stateread[2:4]
        q.setBounds(lb=qread, ub=qread, nodes=0)
        qdot.setBounds(lb=qdotread, ub=qdotread, nodes=0)

        # solve
        solver.solve()
        solution = solver.getSolutionDict()

        # get control input to apply
        u_opt = solution['u'][:, 0]

        # integrate input
        stateint = self.integrator(x0=stateread, p=u_opt, time=self.dt)['xf'].toarray().flatten()
        return stateint, u_opt

# the rti loop
rti = RealTimeIteration(dt=0.01)
stateread = np.array([0.0, np.pi-0.5, 0.0, 0.0])
states = []
inputs = []
times = []
time_qp = []
time_hessian = []
for i in range(300):
    states.append(stateread.copy())
    tic = time.time()
    stateread, input = rti.run(stateread)
    toc = time.time()
    inputs.append(input)
    times.append(toc - tic)
    time_qp.append(solver.getQPComputationTime())
    time_hessian.append(solver.getHessianComputationTime())

plt.figure()
plt.title('state trajectory')
plt.plot(states)

plt.figure()
plt.title('input trajectory')
plt.plot(inputs)

plt.figure()
plt.title('cpu time')
plt.plot(times)

plt.figure()
plt.title('qp vs Hessian time')
plt.plot(time_qp, label='QP')
plt.plot(time_hessian, label='Hessian')
plt.legend()

plt.show()
