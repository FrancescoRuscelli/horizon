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
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn

# USE LEAPFROG INTEGRATOR
LEAPFROG = True

urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)
FKRope = cs.Function.deserialize(kindyn.fk('rope_anchor2'))

# OPTIMIZATION PARAMETERS
ns = 69  # number of shooting nodes
nq = kindyn.nq()  # number of DoFs - NB: 7 DoFs floating base (quaternions)
DoF = nq - 7  # Contacts + anchor_rope + rope
nv = kindyn.nv()  # Velocity DoFs
nf = 3  # 2 feet contacts + rope contact with wall, Force DOfs

# Create horizon problem
prb = problem.Problem(ns, logging_level=logging.DEBUG)

# Creates problem STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)

# Creates problem CONTROL variables
qddot = prb.createInputVariable("qddot", nv)
f1 = prb.createInputVariable("f1", nf)
f2 = prb.createInputVariable("f2", nf)
frope = prb.createInputVariable("frope", nf)

# Creates double integrator
x, xdot = utils.double_integrator_with_floating_base(q, qdot, qddot)

# Formulate discrete time dynamics
tf = 2.0  # [s]
L = 0.5 * cs.dot(qdot, qdot)  # Objective term
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
opts = {'tf': tf / ns}
if LEAPFROG:
    F_integrator_LEAPFROG = integrators.LEAPFROG(dae, opts, cs.SX)
F_integrator = integrators.RK4(dae, opts, cs.SX)

# Bounds and initial guess
q_min = [-10.0, -10.0, -10.0, -1.0, -1.0, -1.0, -1.0,  # Floating base
         0.0, 0.0, 0.0,  # Contact 1
         0.0, 0.0, 0.0,  # Contact 2
         -1.57, -1.57, -3.1415,  # rope_anchor
         0.3]  # rope
q_max = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0,  # Floating base
         0.0, 0.0, 0.0,  # Contact 1
         0.0, 0.0, 0.0,  # Contact 2
         1.57, 1.57, 3.1415,  # rope_anchor
         0.3]  # rope
q_init = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
          0., 0., 0.,
          0., 0., 0.,
          0., 0.3, 0.,
          0.3]
q.setBounds(q_min, q_max)
q.setInitialGuess(q_init)

qdot_min = (-100. * np.ones(nv)).tolist()
qdot_max = (100. * np.ones(nv)).tolist()
qdot.setBounds(qdot_min, qdot_max)
qdot_init = np.zeros(nv).tolist()
qdot.setInitialGuess(qdot_init)

qddot_min = (-100. * np.ones(nv)).tolist()
qddot_max = (100. * np.ones(nv)).tolist()
qddot.setBounds(qddot_min, qddot_max)
qddot_init = np.zeros(nv).tolist()
qddot.setInitialGuess(qdot_init)

f_min = np.zeros(nf).tolist()
f_max = np.zeros(nf).tolist()
f1.setBounds(f_min, f_max)
f2.setBounds(f_min, f_max)
f_init = np.zeros(nf).tolist()
f1.setInitialGuess(f_init)
f2.setInitialGuess(f_init)

f_minRope = (-10000. * np.ones(nf)).tolist()
f_maxRope = (10000. * np.ones(nf)).tolist()
frope.setBounds(f_minRope, f_maxRope)
frope.setInitialGuess(np.zeros(nf).tolist())

# Cost function
prb.createCostFunction("min_joint_vel", 1. * cs.dot(qdot, qdot))

# Constraints
# Initial State
prb.createConstraint("q_init", q-q_init, nodes=0, bounds=dict(lb=np.zeros(nq), ub=np.zeros(nq)))
prb.createConstraint("qdot_init", qdot-qdot_init, nodes=0, bounds=dict(lb=np.zeros(nv), ub=np.zeros(nv)))

# MULTIPLE SHOOTING
q_prev = prb.createStateVariable("q", nq, -1)
qdot_prev = prb.createStateVariable("qdot", nv, -1)
qddot_prev = prb.createInputVariable("qddot", nv, -1)
x_prev, _ = utils.double_integrator_with_floating_base(q_prev, qdot_prev, qddot_prev)
x_int = F_integrator(x0=x_prev, p=qddot_prev)

if LEAPFROG:
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, 2)),
                         bounds=dict(lb=np.zeros(nv + nq).tolist(), ub=np.zeros(nv + nq).tolist()))

    q_pprev = prb.createStateVariable("q", nq, -2)
    qdot_pprev = prb.createStateVariable("qdot", nv, -2)
    qddot_pprev = prb.createInputVariable("qddot", nv, -2)
    x_pprev, _ = utils.double_integrator_with_floating_base(q_pprev, qdot_pprev, qddot_pprev)
    x_int2 = F_integrator_LEAPFROG(x0=x_prev, x0_prev=x_pprev, p=qddot_prev)
    prb.createConstraint("multiple_shooting2", x_int2["xf"] - x, nodes=list(range(2, ns + 1)),
                         bounds=dict(lb=np.zeros(nv + nq).tolist(), ub=np.zeros(nv + nq).tolist()))
else:
    prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns + 1)),
                         bounds=dict(lb=np.zeros(nv + nq).tolist(), ub=np.zeros(nv + nq).tolist()))

# INVERSE DYNAMICS
tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
           -1000., -1000., -1000.,  # Contact 1
           -1000., -1000., -1000.,  # Contact 2
           0., 0., 0.,  # rope_anchor
           -10000.]  # rope

tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
           1000., 1000., 1000.,  # Contact 1
           1000., 1000., 1000.,  # Contact 2
           0., 0., 0.,  # rope_anchor
           0.0]  # rope

frame_force_mapping = {'rope_anchor2': frope}
tau = casadi_kin_dyn.InverseDynamics(kindyn, ['rope_anchor2'], cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q,
                                                                                                                  qdot,
                                                                                                                  qddot,
                                                                                                                  frame_force_mapping)
prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=tau_min, ub=tau_max))

# ROPE CONTACT CONSTRAINT
p_rope_init = FKRope(q=q_init)['ee_pos']
p_rope = FKRope(q=q)['ee_pos']
prb.createConstraint("rope_anchor_point", p_rope - p_rope_init, bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

prb.createProblem()

# SETUP SOLVER
opts = {
        'ipopt.tol': 1e-3
        ,'ipopt.constr_viol_tol': 1e-3
        ,'ipopt.max_iter': 4000
        ,'ipopt.linear_solver': 'ma57'
        }

solver = cs.nlpsol('solver', 'ipopt', prb.getProblem(), opts)
prb.setSolver(solver)

solution = prb.solveProblem()

q_hist = solution["q"]
qdot_hist = solution["qdot"]
qddot_hist = solution["qddot"]
f1_hist = solution["f1"]
f2_hist = solution["f2"]
frope_hist = solution["frope"]

tau_hist = np.zeros(qddot_hist.shape)
ID = casadi_kin_dyn.InverseDynamics(kindyn, ['Contact1', 'Contact2', 'rope_anchor2'])
for i in range(ns):
    frame_force_mapping_i = {'Contact1': f1_hist[:, i], 'Contact2': f2_hist[:, i], 'rope_anchor2': frope_hist[:, i]}
    tau_hist[:, i] = ID.call(q_hist[:, i], qdot_hist[:, i], qddot_hist[:, i], frame_force_mapping_i).toarray().flatten()

# resampling
dt = 0.001
frame_force_hist_mapping = {'Contact1': f1_hist, 'Contact2': f2_hist, 'rope_anchor2': frope_hist}
q_res, qdot_res, qddot_res, frame_force_res_mapping, tau_res = resampler_trajectory.resample_torques(q_hist, qdot_hist,
                                                                                                     qddot_hist,
                                                                                                     tf / ns, dt, dae,
                                                                                                     frame_force_hist_mapping,
                                                                                                     kindyn)

PRINT = False
if PRINT:
    # plots raw solution
    time = np.arange(0.0, tf + 1e-6, tf / ns)
    time_res = np.arange(0.0, q_res.shape[1] * dt - dt, dt)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time, q_hist[i, :])
    plt.suptitle('$\mathrm{Base \ Position}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{[m]}$', size=20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time_res, q_res[i, :])
    plt.suptitle('$\mathrm{Base \ Position \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{[m]}$', size=20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], qddot_hist[i, :])
    plt.suptitle('$\mathrm{Base \ Acceleration}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [m] } /  \mathrm{ [sec^2] } $', size=20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time_res[:-1], qddot_res[i, :])
    plt.suptitle('$\mathrm{Base \ Acceleration \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [m] } /  \mathrm{ [sec^2] } $', size=20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], f1_hist[i, :])
        plt.plot(time[:-1], f2_hist[i, :])
        plt.plot(time[:-1], frope_hist[i, :])
    plt.suptitle('$\mathrm{force \ feet \ and \ rope}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] } /  \mathrm{ [sec^2] } $', size=20)

    plt.figure()
    f1_res = frame_force_res_mapping["Contact1"]
    f2_res = frame_force_res_mapping["Contact2"]
    frope_res = frame_force_res_mapping["rope_anchor2"]
    for i in range(0, 3):
        plt.plot(time_res[:-1], f1_res[i, :])
        plt.plot(time_res[:-1], f2_res[i, :])
        plt.plot(time_res[:-1], frope_res[i, :])
    plt.suptitle('$\mathrm{force \ feet \ and \ rope \ resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] } /  \mathrm{ [sec^2] } $', size=20)

    plt.figure()
    for i in range(0, 6):
        plt.plot(time[:-1], tau_hist[i, :])
    plt.suptitle('$\mathrm{base \ force}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] }} $', size=20)

    plt.figure()
    for i in range(0, 6):
        plt.plot(time_res[:-1], tau_res[i, :], '-x')
    plt.suptitle('$\mathrm{base \ force \ resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] }} $', size=20)

    plt.show()

# REPLAY TRAJECTORY
joint_list = ['Contact1_x', 'Contact1_y', 'Contact1_z',
              'Contact2_x', 'Contact2_y', 'Contact2_z',
              'rope_anchor1_1_x', 'rope_anchor1_2_y', 'rope_anchor1_3_z',
              'rope_joint']

replay_trajectory(dt, joint_list, q_res, frame_force_res_mapping).replay()
