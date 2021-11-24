#!/usr/bin/env python3

from casadi_kin_dyn import pycasadi_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.solvers import Solver
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils import plotter
from horizon.ros.replay_trajectory import replay_trajectory

import matplotlib.pyplot as plt
import os

# Loading URDF model in pinocchio
urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'lwr.urdf')
urdf = open(urdffile, 'r').read()

# Create casadi interface to pinocchio
kindyn = pycasadi_kin_dyn.CasadiKinDyn(urdf)
nq = kindyn.nq()
nv = kindyn.nv()

# OPTIMIZATION PARAMETERS
ns = 100  # number of shooting nodes
tf = 1.0  # [s]
dt = tf/ns
transcription = 'direct_collocation'
solver_type = 'ilqr'

# Create horizon problem
prb = problem.Problem(ns)

# Create problem STATE and INPUT variables
q = prb.createStateVariable("q", nq)
qdot = prb.createInputVariable("qdot", nv)

# Create dynamics
prb.setDynamics(qdot)
prb.setDt(dt)
# Limits
q_min = np.full((nq), -3.0)
q_max = -q_min
qdot_lims = np.full((nv), 10)
q.setBounds(q_min, q_max)
qdot.setBounds(-qdot_lims, qdot_lims)

# Initial value
q_init = np.array([0, 0.7, 0, -0.9, 0, 0, 0])
qdot_init = np.zeros((nq))
q.setBounds(q_init, q_init, nodes=0)
qdot.setBounds(qdot_init, qdot_init, nodes=0)

# Initial guess
q.setInitialGuess(q_init)

# Cost function (min velocity)
prb.createIntermediateCost("qdot", 1e-3*cs.sumsqr(qdot))

# Cost function (min effort)
id = cs.Function.deserialize(kindyn.rnea())
gcomp = id(q, 0, 0)
prb.createIntermediateCost("min_effort", 1*cs.sumsqr(gcomp))

# Final goal
fk = cs.Function.deserialize(kindyn.fk('lwr_7_link'))
pos = fk(q)[0]
pos_des = fk(q_init)[0]
pos_des[2] += 0.1

if solver_type != 'ilqr':
    # Dynamics
    Transcriptor.make_method(transcription, prb, opts={'integrator': 'EULER'})

# Constraints
prb.createFinalConstraint("goal", pos - pos_des)

# Creates problem
solver = Solver.make_solver(solver_type, prb, opts={'hxx_reg_growth_factor': 10.0})  #, opts={'max_iter': 10})

if solver_type == 'ilqr':
    solver.plot_iter = True
    solver.set_iteration_callback()

# solver.plot_iter = True
# solver.set_iteration_callback()
# solver.max_iter = 400
print('started!')
solver.solve()

plt.plot(solver.x_opt.T)
plt.show()

# joint_list=["cart_joint", "pole_joint"]
# replay_trajectory(tf/ns, joint_list, q_hist).replay(is_floating_base=False)