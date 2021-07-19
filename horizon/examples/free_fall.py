#!/usr/bin/env python
import logging

import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import rospy
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, integrators, casadi_kin_dyn, resampler_trajectory
import matplotlib.pyplot as plt

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
x, xdot = utils.double_integrator_with_floating_base(q, qdot, qddot)

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
prb.createCostFunction("min_joint_acc", 1000.*cs.dot(qddot[6:-1], qddot[6:-1]), list(range(1, ns)))
prb.createCostFunction("min_f1", 1000.*cs.dot(f1, f1), list(range(1, ns)))
prb.createCostFunction("min_f2", 1000.*cs.dot(f2, f2), list(range(1, ns)))

frope_prev = prb.createInputVariable("frope", nf, -1)
prb.createCostFunction("min_dfrope", 1000.*cs.dot(frope-frope_prev, frope-frope_prev), list(range(1, ns)))

# Constraints
prb.createConstraint("qinit", q, nodes=0, bounds=dict(lb=q_init, ub=q_init))
prb.createConstraint("qdotinit", qdot, nodes=0, bounds=dict(lb=qdot_init, ub=qdot_init))

q_prev = prb.createStateVariable("q", nq, -1)
qdot_prev = prb.createStateVariable("qdot", nv, -1)
qddot_prev = prb.createInputVariable("qddot", nv, -1)
x_prev, _ = utils.double_integrator_with_floating_base(q_prev, qdot_prev, qddot_prev)
x_int = F_integrator(x0=x_prev, p=qddot_prev)
prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns+1)), bounds=dict(lb=np.zeros(nv+nq).tolist(), ub=np.zeros(nv+nq).tolist()))

tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
                    -1000., -1000., -1000.,  # Contact 1
                    -1000., -1000., -1000.,  # Contact 2
                    0., 0., 0.,  # rope master point
                    0.]  # rope
tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
                        1000., 1000., 1000.,  # Contact 1
                        1000., 1000., 1000.,  # Contact 2
                        0., 0., 0.,  # rope master point
                        0.0]  # rope

frame_force_mapping = {'rope_anchor2': frope}
tau = casadi_kin_dyn.inverse_dynamics(q, qdot, qddot, frame_force_mapping, kindyn)
prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=tau_min, ub=tau_max))

p_rope_init = FKRope(q=q_init)['ee_pos']
p_rope = FKRope(q=q)['ee_pos']
prb.createConstraint("rope_anchor_point", p_rope-p_rope_init, bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

# Creates problem
prb.createProblem()

# SETUP SOLVER
opts = {'ipopt.tol': 1e-4,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}

solver = cs.nlpsol('solver', 'ipopt', prb.getProblem(), opts)
prb.setSolver(solver)

solution = prb.solveProblem()

q_hist = solution["q"]
qdot_hist = solution["qdot"]
qddot_hist = solution["qddot"]
f1_hist = solution["f1"]
f2_hist = solution["f2"]
frope_hist = solution["frope"]


# resampling
dt = 0.001
q_res, qdot_res, qddot_res = resampler_trajectory.second_order_resample_integrator(q_hist, qdot_hist, qddot_hist, tf, dt, dae, frame_force_mapping, kindyn)


PRINT = False
if PRINT:
    # plots raw solution
    time = np.arange(0.0, tf+1e-6, tf/ns)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time, q_hist[i,:])
    plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{[m]}$', size = 20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], qddot_hist[i,:])
    plt.suptitle('$\mathrm{Base \ Acceleration}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{ [m] } /  \mathrm{ [sec^2] } $', size = 20)

    plt.figure()
    for i in range(0, 3):
        plt.plot(time[:-1], f1_hist[i, :])
        plt.plot(time[:-1], f2_hist[i, :])
        plt.plot(time[:-1], frope_hist[i, :])
    plt.suptitle('$\mathrm{force \ feet \ and \ rope}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{ [N] } /  \mathrm{ [sec^2] } $', size=20)

    plt.show()



# REPLAY TRAJECTORY
joint_list = ['Contact1_x', 'Contact1_y', 'Contact1_z',
              'Contact2_x', 'Contact2_y', 'Contact2_z',
              'rope_anchor1_1_x', 'rope_anchor1_2_y', 'rope_anchor1_3_z',
              'rope_joint']

resampler_trajectory.replay_trajectory(dt, joint_list, q_res).replay()

