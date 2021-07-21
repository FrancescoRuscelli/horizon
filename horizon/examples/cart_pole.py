#!/usr/bin/env python

import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
import logging
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn, resampler_trajectory
from horizon.ros.replay_trajectory import *
import matplotlib.pyplot as plt

# Loading URDF model in pinocchio
urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

nq = kindyn.nq()
nv = kindyn.nv()

print("nq: ", nq)
print("nv: ", nv)

# OPTIMIZATION PARAMETERS
ns = 100  # number of shooting nodes

# Create horizon problem
prb = problem.Problem(ns, logging_level=logging.DEBUG)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)

# Creates double integrator
x, xdot = utils.double_integrator(q, qdot, qddot)

q_prev = prb.createStateVariable("q", nq, -1)
qdot_prev = prb.createStateVariable("qdot", nv, -1)
qddot_prev = prb.createInputVariable("qddot", nv, -1)
x_prev, _ = utils.double_integrator(q_prev, qdot_prev, qddot_prev)

# Formulate discrete time dynamics
tf = 5.0  # [s]
L = 0.5*cs.dot(qdot, qdot)  # Objective term
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
opts = {'tf': tf/ns}
F_integrator = integrators.RK4(dae, opts, cs.SX)

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
l = F_integrator(x0=x, p=qddot)
prb.createCostFunction("qddot", cs.dot(qddot, qddot), nodes=0)
prb.createCostFunction("L", l["qf"], nodes=list(range(0, ns)))

# Constraints
x_int = F_integrator(x0=x_prev, p=qddot_prev)
prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns+1)), bounds=dict(lb=np.zeros(nv+nq).tolist(), ub=np.zeros(nv+nq).tolist()))
prb.createConstraint("up", q[1] - np.pi, nodes=ns, bounds=dict(lb = [0], ub = [0]))
prb.createConstraint("final_qdot", qdot - np.array([0., 0.]), nodes=ns, bounds=dict(lb = np.zeros(2).tolist(), ub = np.zeros(2).tolist()))


tau_lims = np.array([1000., 0.])
tau = casadi_kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=(-tau_lims).tolist(), ub=tau_lims.tolist()))

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
plt.show()

joint_list=["cart_joint", "pole_joint"]
replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)








