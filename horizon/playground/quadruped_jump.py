#!/usr/bin/env python
import logging

import rospy
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory
from horizon.transcriptions import integrators
from horizon.solvers import solver
from horizon.utils.plotter import PlotterHorizon
from horizon.ros.replay_trajectory import *
import matplotlib.pyplot as plt
import os
import time
from horizon.ros import utils as horizon_ros_utils

horizon_ros_utils.roslaunch("horizon_examples", "quadruped_template.launch")
time.sleep(3.)

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'quadruped_template.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# OPTIMIZATION PARAMETERS
ns = 40  # number of shooting nodes
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

prb.setDynamics(xdot)
prb.setDt(dt)
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

dt_min = [0.03] #[s]
dt_max = [0.15] #[s]
dt_init = [dt_min]
dt.setBounds(dt_min, dt_max)
dt.setInitialGuess(dt_init)

# SET UP COST FUNCTION
lift_node = 10
touch_down_node = 30
q_fb_trg = np.array([q_init[0], q_init[1], q_init[2] + 0.9, 0.0, 0.0, 0.0, 1.0]).tolist()

prb.createCostFunction("jump", 10.*cs.dot(q[0:3] - q_fb_trg[0:3], q[0:3] - q_fb_trg[0:3]), nodes=list(range(lift_node, touch_down_node)))
prb.createCostFunction("min_qdot", 10.*cs.dot(qdot, qdot))
#prb.createCostFunction("min_qddot", 1e-4*cs.dot(qddot, qddot), nodes= list(range(0, ns)))
#prb.createCostFunction("min_force", 1e-4*cs.dot( f1 + f2 + f3 + f4, f1 + f2 + f3 + f4), nodes=list(range(0, ns)))


#f1_prev = f1.getVarOffset(-1)
#f2_prev = f2.getVarOffset(-1)
#f3_prev = f3.getVarOffset(-1)
#f4_prev = f4.getVarOffset(-1)
#prb.createCostFunction("min_deltaforce", 1e-4*cs.dot( (f1-f1_prev) + (f2-f2_prev) + (f3-f3_prev) + (f4-f4_prev),
#                                                      (f1-f1_prev) + (f2-f2_prev) + (f3-f3_prev) + (f4-f4_prev)),
#                                                      nodes= list(range(1, ns)))

# Constraints
q_prev = q.getVarOffset(-1)
qdot_prev = qdot.getVarOffset(-1)
qddot_prev = qddot.getVarOffset(-1)
dt_prev = dt.getVarOffset(-1)
x_prev, _ = utils.double_integrator_with_floating_base(q_prev, qdot_prev, qddot_prev)
x_int = F_integrator(x0=x_prev, p=qddot_prev, time=dt_prev)
prb.createConstraint("multiple_shooting", x_int["xf"] - x, nodes=list(range(1, ns+1)), bounds=dict(lb=np.zeros(nv+nq), ub=np.zeros(nv+nq)))

tau_min = [0., 0., 0., 0., 0., 0.,  # Floating base
            -1000., -1000., -1000.,  # Contact 1
            -1000., -1000., -1000.,  # Contact 2
            -1000., -1000., -1000.,  # Contact 3
            -1000., -1000., -1000.]  # Contact 4

tau_max = [0., 0., 0., 0., 0., 0.,  # Floating base
            1000., 1000., 1000.,  # Contact 1
            1000., 1000., 1000.,  # Contact 2
            1000., 1000., 1000.,  # Contact 3
            1000., 1000., 1000.]  # Contact 4
dd = {'Contact1': f1, 'Contact2': f2, 'Contact3': f3, 'Contact4': f4}
tau = kin_dyn.InverseDynamics(kindyn, dd.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, qdot, qddot, dd)
prb.createConstraint("inverse_dynamics", tau, nodes=list(range(0, ns)), bounds=dict(lb=tau_min, ub=tau_max))
prb.createFinalConstraint('final_velocity', qdot)

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

    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=qdot)['ee_vel_linear']
    prb.createConstraint(f"{frame}_vel_before_jump", v, nodes=list(range(0, lift_node)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))
    prb.createConstraint(f"{frame}_vel_after_jump", v, nodes=list(range(touch_down_node, ns + 1)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

    fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
    prb.createConstraint(f"{frame}_friction_cone_before_jump", fc, nodes=list(range(0, lift_node)), bounds=dict(lb=fc_lb, ub=fc_ub))
    prb.createConstraint(f"{frame}_friction_cone_after_jump", fc, nodes=list(range(touch_down_node, ns)), bounds=dict(lb=fc_lb, ub=fc_ub))

    # DURING FLIGHT PHASE
    prb.createConstraint(f"{frame}_no_force_during_jump", f, nodes=list(range(lift_node, touch_down_node)), bounds=dict(lb=[0., 0., 0.], ub=[0., 0., 0.]))

# Create problem
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 5000}#,
        # 'ipopt.linear_solver': 'ma57'}

solver = solver.Solver.make_solver('ipopt', prb, opts)
solver.solve()

solution = solver.getSolutionDict()

plot_all = True
if plot_all:
    hplt = PlotterHorizon(prb, solution)
    hplt.plotVariables(show_bounds=False, legend=True)
    # hplt.plotFunctions(show_bounds=False)
    plt.show()

q_hist = solution["q"]
qdot_hist = solution["qdot"]
qddot_hist = solution["qddot"]
f1_hist = solution["f1"]
f2_hist = solution["f2"]
f3_hist = solution["f3"]
f4_hist = solution["f4"]
dt_hist = solution["dt"]

tau_hist = np.zeros(qddot_hist.shape)
ID = kin_dyn.InverseDynamics(kindyn, ['Contact1', 'Contact2', 'Contact3', 'Contact4'], cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
for i in range(ns):
    frame_force_mapping_i = {'Contact1': f1_hist[:, i], 'Contact2': f2_hist[:, i], 'Contact3': f3_hist[:, i], 'Contact4': f4_hist[:, i]}
    tau_hist[:, i] = ID.call(q_hist[:, i], qdot_hist[:, i], qddot_hist[:, i], frame_force_mapping_i).toarray().flatten()



# resampling
dt = 0.001
frame_force_hist_mapping = {'Contact1': f1_hist, 'Contact2': f2_hist, 'Contact3': f3_hist, 'Contact4': f4_hist}
q_res, qdot_res, qddot_res, frame_force_res_mapping, tau_res = resampler_trajectory.resample_torques(
                                                                        q_hist, qdot_hist, qddot_hist, dt_hist.flatten(),
                                                                        dt, dae, frame_force_hist_mapping, kindyn,
                                                                        cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

PLOTS = True
if PLOTS:
    time = np.arange(0.0, q_res.shape[1]*dt, dt)

    plt.figure()
    for i in range(3):
        plt.plot(time, q_res[i, :])
    plt.suptitle('$\mathrm{Base \ Position \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{[m]}$', size=20)

    plt.figure()
    for i in range(6):
        plt.plot(tau_hist[i, :])
    plt.suptitle('$\mathrm{Base \ Forces}$', size=20)
    plt.xlabel('$\mathrm{sample}$', size=20)
    plt.ylabel('$\mathrm{[N]}$', size=20)

    plt.figure()
    for i in range(6):
        plt.plot(time[:-1], tau_res[i, :])
    plt.suptitle('$\mathrm{Base \ Forces \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{[N]}$', size=20)

    plt.figure()
    for i in range(q_res.shape[0]):
        plt.plot(time, q_res[i, :])
    plt.suptitle('$\mathrm{q \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{q}$', size=20)

    plt.figure()
    for i in range(qdot_res.shape[0]):
        plt.plot(time, qdot_res[i, :])
    plt.suptitle('$\mathrm{\dot{q} \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{\dot{q}}$', size=20)

    plt.figure()
    for i in range(qddot_res.shape[0]):
        plt.plot(time[:-1], qddot_res[i, :])
    plt.suptitle('$\mathrm{\ddot{q} \ Resampled}$', size=20)
    plt.xlabel('$\mathrm{[sec]}$', size=20)
    plt.ylabel('$\mathrm{\ddot{q}}$', size=20)

    plt.show()




joint_list = ['Contact1_x', 'Contact1_y', 'Contact1_z',
              'Contact2_x', 'Contact2_y', 'Contact2_z',
              'Contact3_x', 'Contact3_y', 'Contact3_z',
              'Contact4_x', 'Contact4_y', 'Contact4_z']


repl = replay_trajectory(dt, joint_list, q_res, frame_force_res_mapping, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
repl.sleep(1.)
repl.replay()

