#!/usr/bin/env python

import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import rospy
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, integrators



# Loading URDF model in pinocchio
urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# Forward Kinematics of interested links
FK_waist = cs.Function.deserialize(kindyn.fk('Waist'))
FKR = cs.Function.deserialize(kindyn.fk('Contact1'))
FKL = cs.Function.deserialize(kindyn.fk('Contact2'))
FKRope = cs.Function.deserialize(kindyn.fk('rope_anchor2'))

# Inverse Dynamics
ID = cs.Function.deserialize(kindyn.rnea())

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
qddot.setBounds(qddot_min, qddot_max, [0,ns-1])

f_min = (-10000.*np.ones(nf)).tolist()
f_max = (10000.*np.ones(nf)).tolist()
f1.setBounds(f_min, f_max, [0,ns-1])
f2.setBounds(f_min, f_max, [0,ns-1])
frope.setBounds(f_min, f_max, [0,ns-1])



