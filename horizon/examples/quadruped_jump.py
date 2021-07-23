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
qddot_init[2] = -9.81
qddot.setBounds(qddot_min, qddot_max)
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

dt_min = [0.15] #[s]
dt_max = [0.15] #[s]
dt_init = [dt_min]
dt.setBounds(dt_min, dt_max)
dt.setInitialGuess(dt_init)

# SET UP COST FUNCTION
lift_node = 10
touch_down_node = 20
q_fb_trg = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.0]

#prb.createCostFunction("jump", 1000.*cs.dot(q[2] - q_fb_trg[2], q[2] - q_fb_trg[2]), nodes = list(range(lift_node, touch_down_node)))
prb.createCostFunction("floating_base_quaternion", 100.*cs.dot(q[3:7] - q_fb_trg[3:7], q[3:7] - q_fb_trg[3:7]))
prb.createCostFunction("min_qdot", 10.*cs.dot(qdot, qdot))

qddot_prev = prb.createInputVariable("qddot", nv, -1)
prb.createCostFunction("min_jerk", 0.0003*cs.dot(qddot-qddot_prev, qddot-qddot_prev), nodes=list(range(1, ns)))

f1_prev = prb.createInputVariable("f1", nf, -1)
f2_prev = prb.createInputVariable("f2", nf, -1)
f3_prev = prb.createInputVariable("f3", nf, -1)
f4_prev = prb.createInputVariable("f4", nf, -1)
prb.createCostFunction("min_deltaforce", 0.01*cs.dot( (f1-f1_prev) + (f2-f2_prev) + (f3-f3_prev) + (f4-f4_prev),
                                                      (f1-f1_prev) + (f2-f2_prev) + (f3-f3_prev) + (f4-f4_prev)), nodes=list(range(1, ns)))

# Constraints
q_prev = prb.createStateVariable("q", nq, -1)
qdot_prev = prb.createStateVariable("qdot", nv, -1)
qddot_prev = prb.createInputVariable("qddot", nv, -1)
dt_prev = prb.createInputVariable("dt", 1, -1)
x_prev, _ = utils.double_integrator_with_floating_base(q_prev, qdot_prev, qddot_prev)
x_int = F_integrator(x0=x_prev, p=qddot_prev, time=dt_prev)
prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns+1)), bounds=dict(lb=np.zeros(nv+nq), ub=np.zeros(nv+nq)))

tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
            -10000., -10000., -10000.,  # Contact 1
            -10000., -10000., -10000.,  # Contact 2
            -10000., -10000., -10000.,  # Contact 3
            -10000., -10000., -10000.]  # Contact 4

tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
            10000., 10000., 10000.,  # Contact 1
            10000., 10000., 10000.,  # Contact 2
            10000., 10000., 10000.,  # Contact 3
            10000., 10000., 10000.]  # Contact 4
dd = {'Contact1': f1, 'Contact2': f2, 'Contact3': f3, 'Contact4': f4}
tau = casadi_kin_dyn.InverseDynamics(kindyn, dd.keys()).call(q, qdot, qddot, dd)
prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=tau_min, ub=tau_max))

# GROUND
mu = 0.8 # friction coefficient
R = np.identity(3, dtype=float) # environment rotation wrt inertial frame

# foot
contact_names = ['Contact1', 'Contact2', 'Contact3', 'Contact4']
forces = [f1, f2, f3, f4]
for frame, f in zip(contact_names, forces):
    # BEFORE AND AFTER FLIGHT PHASE
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=q)['ee_pos']
    pd = FK(q=q_init)['ee_pos']
    prb.createConstraint(f"{frame}_before_jump", p - pd, nodes=list(range(0, lift_node)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))
    prb.createConstraint(f"{frame}_after_jump", p - pd, nodes=list(range(touch_down_node, ns+1)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

    fc, fc_lb, fc_ub = casadi_kin_dyn.linearized_friciton_cone(f, mu, R)
    prb.createConstraint(f"{frame}_friction_cone_before_jump", fc, nodes=list(range(0, lift_node)), bounds=dict(lb=fc_lb, ub=fc_ub))
    prb.createConstraint(f"{frame}_friction_cone_after_jump", fc, nodes=list(range(touch_down_node, ns)), bounds=dict(lb=fc_lb, ub=fc_ub))

    # DURING FLIGHT PHASE
    #prb.createConstraint(f"{frame}_no_force_during_jump", f, nodes=list(range(lift_node, touch_down_node)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

# Create problem
prb.createProblem()

opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 5000,
        'ipopt.linear_solver': 'ma57'}

solver = cs.nlpsol('solver', 'ipopt', prb.getProblem(), opts)
prb.setSolver(solver)

solution = prb.solveProblem()

q_hist = solution["q"]
qdot_hist = solution["qdot"]
qddot_hist = solution["qddot"]
f1_hist = solution["f1"]
f2_hist = solution["f2"]
f3_hist = solution["f3"]
f4_hist = solution["f4"]
dt_hist = solution["dt"]

# resampling
dt = 0.001
frame_force_hist_mapping = {'Contact1': f1_hist, 'Contact2': f2_hist, 'Contact3': f3_hist, 'Contact4': f4_hist}
q_res, qdot_res, qddot_res, frame_force_res_mapping, tau_res = resampler_trajectory.resample_torques(q_hist, qdot_hist, qddot_hist, np.sum(dt_hist), dt, dae, frame_force_hist_mapping, kindyn)

joint_list = ['Contact1_x', 'Contact1_y', 'Contact1_z',
              'Contact2_x', 'Contact2_y', 'Contact2_z',
              'Contact3_x', 'Contact3_y', 'Contact3_z',
              'Contact4_x', 'Contact4_y', 'Contact4_z']

replay_trajectory(dt, joint_list, q_res, frame_force_res_mapping).replay()

