#!/usr/bin/env python
import logging

import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import rospy
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn, resampler_trajectory
from horizon.ros.replay_trajectory import *
import matplotlib.pyplot as plt

urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# OPTIMIZATION PARAMETERS
ns = 30  # number of shooting nodes
nc = 4  # number of contacts
nq = kindyn.nq()  # number of DoFs - NB: 7 DoFs floating base (quaternions)
DoF = nq - 7  # Contacts + anchor_rope + rope
nv = kindyn.nv()  # Velocity DoFs
nf = 3  # Force DOfs

# Create horizon problem
prb = problem.Problem(ns)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)
f1 = prb.createInputVariable("f1", nf)
f2 = prb.createInputVariable("f2", nf)
f3 = prb.createInputVariable("f3", nf)
f4 = prb.createInputVariable("f4", nf)
dt = prb.createInputVariable("dt", 1)

x, xdot = utils.double_integrator_with_floating_base(q, qdot, qddot)

# Formulate discrete time dynamics
L = 0.5*cs.dot(qdot, qdot)  # Objective term
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
F_integrator = integrators.RK4(dae, {}, cs.SX)

# Limits
disp_z = 0.2

q_min = [-10.0, -10.0, -10.0, -1.0, -1.0, -1.0, -1.0,  # Floating base
                  0.1, 0.1, -0.635,
                  0.1, -0.5, -0.635,
                  -0.6, -0.5, -0.635,
                  -0.6, 0.1, -0.635]

q_max = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0,  # Floating base
                  0.6, 0.5, -0.635 + disp_z,
                  0.6, -0.1, -0.635 + disp_z,
                  -0.1, -0.1, -0.635 + disp_z,
                  -0.1, 0.5, -0.635 + disp_z]

q_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.349999, 0.349999, -0.635,
                   0.349999, -0.349999, -0.635,
                   -0.349999, -0.349999, -0.635,
                   -0.349999, 0.349999, -0.635]
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
q.setInitialGuess(q_init)

qdot_min = (-100.*np.ones(nv)).tolist()
qdot_max = (100.*np.ones(nv)).tolist()
qdot_init = np.zeros(nv).tolist()
qdot.setBounds(qdot_min, qdot_max)
qdot.setBounds(qdot_init, qdot_init, 0)
qdot.setInitialGuess(qdot_init)

qddot_min = (-100.*np.ones(nv)).tolist()
qddot_max = (100.*np.ones(nv)).tolist()
qddot_init = np.zeros(nv).tolist()
qddot_init[2] = -9.8
qddot.setBounds(qddot_min, qddot_max)
qddot.setBounds(qddot_init, qddot_init, 0)
qddot.setInitialGuess(qddot_init)

f_min = (-10000.*np.ones(nf)).tolist()
f_max = (10000.*np.ones(nf)).tolist()
f_init = np.zeros(nf).tolist()
f1.setBounds(f_min, f_max)
f1.setInitialGuess(f_init)
f2.setBounds(f_min, f_max)
f2.setInitialGuess(f_init)
f3.setBounds(f_min, f_max)
f3.setInitialGuess(f_init)
f4.setBounds(f_min, f_max)
f4.setInitialGuess(f_init)

dt_min = 0.03 #[s]
dt_max = 0.15 #[s]
dt_init = dt_min
dt.setBounds(dt_min, dt_max)
dt.setInitialGuess(dt_init)

# SET UP COST FUNCTION
lift_node = 10
touch_down_node = 20
q_fb_trg = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]

prb.createCostFunction("jump", 1000.*cs.dot(q[2] - q_fb_trg[2], q[2] - q_fb_trg[2]), nodes = list(range(lift_node, touch_down_node)))



# min_fb_or = lambda k: 100.*dot(Q[k][3:7]-q_fb_trg[3:7], Q[k][3:7]-q_fb_trg[3:7])
# J += cost_function(min_fb_or, 0, ns)
#
# min_qdot = lambda k: 10.*dot(Qdot[k], Qdot[k])
# J += cost_function(min_qdot,  0, ns)
#
# min_jerk = lambda k: 0.0003*dot(Qddot[k]-Qddot[k-1], Qddot[k]-Qddot[k-1])
# J += cost_function(min_jerk, 0, ns-1) # <- this smooths qddot solution
#
# min_deltaFC = lambda k: 0.01*dot((F1[k]-F1[k-1])+(F2[k]-F2[k-1])+(F3[k]-F3[k-1])+(F4[k]-F4[k-1]),
#                                  (F1[k]-F1[k-1])+(F2[k]-F2[k-1])+(F3[k]-F3[k-1])+(F4[k]-F4[k-1]))  # min Fdot
# J += cost_function(min_deltaFC, touch_down_node+1, ns-1)
#
#

