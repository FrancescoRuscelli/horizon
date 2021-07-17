#!/usr/bin/env python

import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import rospy
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn

# Loading URDF model in pinocchio
urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# Forward Kinematics of interested links
FK_waist = cs.Function.deserialize(kindyn.fk('Waist'))
FKR = cs.Function.deserialize(kindyn.fk('Contact1'))
FKL = cs.Function.deserialize(kindyn.fk('Contact2'))
FKRope = cs.Function.deserialize(kindyn.fk('rope_anchor2'))

# OPTIMIZATION PARAMETERS
ns = 30  # number of shooting nodes
nc = 3  # number of contacts
nq = kindyn.nq()  # number of DoFs - NB: 7 DoFs floating base (quaternions)
DoF = nq - 7  # Contacts + anchor_rope + rope
nv = kindyn.nv()  # Velocity DoFs
nf = 3  # 2 feet contacts + rope contact with wall, Force DOfs

# Create horizon problem
prb = problem.Problem(ns)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)
f1 = prb.createInputVariable("f1", nf)
f2 = prb.createInputVariable("f2", nf)
frope = prb.createInputVariable("frope", nf)

# Creates double integrator
x, xdot = utils.dynamic_model_with_floating_base(q, qdot, qddot)

# Formulate discrete time dynamics
tf = 1.0  # [s]
L = 0.5*cs.dot(qdot, qdot)  # Objective term
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
opts = {'tf': tf/ns}
F_integrator = integrators.RK4(dae, opts, "SX")

# Add bounds to STATE and CONTROL variables
q_min = [-10.0, -10.0, -10.0, -1.0, -1.0, -1.0, -1.0,  # Floating base
                  -0.3, -0.1, -0.1,  # Contact 1
                  -0.3, -0.05, -0.1,  # Contact 2
                  -1.57, -1.57, -3.1415,  # rope_anchor
                  0.0]  # rope
q_max = [10.0,  10.0,  10.0,  1.0,  1.0,  1.0,  1.0,  # Floating base
                  0.3, 0.05, 0.1,  # Contact 1
                  0.3, 0.1, 0.1,  # Contact 2
                  1.57, 1.57, 3.1415,  # rope_anchor
                  10.0]  # rope
q.setBounds(q_min, q_max)

qdot_min = (-100.*np.ones(nv)).tolist()
qdot_max = (100.*np.ones(nv)).tolist()
qdot.setBounds(qdot_min, qdot_max)

qddot_min = (-100.*np.ones(nv)).tolist()
qddot_max = (100.*np.ones(nv)).tolist()
qddot.setBounds(qddot_min, qddot_max)

f_min = (-10000.*np.ones(nf)).tolist()
f_max = (10000.*np.ones(nf)).tolist()
f1.setBounds(f_min, f_max)
f2.setBounds(f_min, f_max)
frope.setBounds(f_min, f_max)

# Add initial guess to STATE and CONTROL variables
q_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0., 0., 0.,
                   0., 0., 0.,
                   0., 0., 0.,
                   0.1]
q.setInitialGuess(q_init)

qdot_init = np.zeros(nv).tolist()
qdot.setInitialGuess(qdot_init)

qddot_init = np.zeros(nv).tolist()
qddot.setInitialGuess(qdot_init)

f_init = np.zeros(nf).tolist()
f1.setInitialGuess(f_init)
f2.setInitialGuess(f_init)
frope.setInitialGuess(f_init)

# Cost function
prb.createCostFunction("min_joint_vel", 100.*cs.dot(qdot[6:-1], qdot[6:-1]))
prb.createCostFunction("min_joint_acc", 1000.*cs.dot(qddot[6:-1], qddot[6:-1]))
prb.createCostFunction("min_f1", 1000.*cs.dot(f1, f1))
prb.createCostFunction("min_f2", 1000.*cs.dot(f2, f2))

frope_prev = prb.createInputVariable("frope", nf, -1)
prb.createCostFunction("min_dfrope", 1000.*cs.dot(frope-frope_prev, frope-frope_prev), [1, ns])

# Constraints
prb.createConstraint("qinit", q, nodes=0, bounds=dict(lb=q_init, ub=q_init))
prb.createConstraint("qdotinit", qdot, nodes=0, bounds=dict(lb=qdot_init, ub=qdot_init))

x_int = F_integrator(x0=x, p=qddot)
prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=[0,ns], bounds=dict(lb=np.zeros(nv+nq).tolist(), ub=np.zeros(nv+nq).tolist()))

tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
                    -1000., -1000., -1000.,  # Contact 1
                    -1000., -1000., -1000.,  # Contact 2
                    0., 0., 0.,  # rope_anchor
                    0.]  # rope
tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
                        1000., 1000., 1000.,  # Contact 1
                        1000., 1000., 1000.,  # Contact 2
                        0., 0., 0.,  # rope_anchor
                        0.0]  # rope

frame_force_mapping = {'rope_anchor2': frope}
tau = casadi_kin_dyn.inverse_dynamics(q, qdot, qddot, frame_force_mapping, kindyn)
prb.createConstraint("inverse_dynamics", tau, nodes=[0,ns], bounds=dict(lb=tau_min, ub=tau_max))
