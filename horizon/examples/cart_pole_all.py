#!/usr/bin/env python3

'''
An example of the cart-pole problem: find the trajectory of the cart so that the pole reaches the upright position.
(difference from 'cart_pole.py': system inputs are the torques, not the accelerations)
'''

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
import casadi as cs
import numpy as np
from horizon import problem
from horizon.utils import utils, kin_dyn, mat_storer
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.utils.plotter import PlotterHorizon
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, argparse


parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('-replay', help='visualize the robot trajectory in rviz', action='store_true')

args = parser.parse_args()
# ipopt, gnsqp, ilqr, blocksqp
solver_type = 'gnsqp' # todo fails with ilqr and blocksqp and gnsqp
rviz_replay = False
plot_sol = True
torque_input = False
optimize_final_time = False

if solver_type and optimize_final_time:
    input("'ilqr' solver supports only float and Parameter dt. Press a button to continue.")
    optimize_final_time = False

if args.replay:
    from horizon.ros.replay_trajectory import *
    import roslaunch, rospkg, rospy
    rviz_replay = True
    plot_sol = False


# Create CasADi interface to Pinocchio
urdffile = os.path.join(os.getcwd(), 'urdf', 'cart_pole.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# Get dimension of pos and vel
nq = kindyn.nq()
nv = kindyn.nv()

# Optimization parameters
tf = 5.0  # [s]
ns = 100 # number of nodes
dt = tf/ns

# options for horizon transcription
transcription_method = 'multiple_shooting'  # can choose between 'multiple_shooting' and 'direct_collocation'
transcription_opts = dict(integrator='RK4') # integrator used by the multiple_shooting

# Create horizon problem
prb = problem.Problem(ns)

# creates the variables for the problem
# design choice: position and the velocity as state and acceleration as input

# STATE variables
q = prb.createStateVariable("q", nq)
qdot = prb.createStateVariable("qdot", nv)
# CONTROL variables
if torque_input:
    # cart joint is actuated, while the pole joint is unactuated
    tau = prb.createInputVariable("tau", 2)
else:
    qddot = prb.createInputVariable("qddot", nv)

# Set dynamics of the system and the relative dt
if torque_input:
    fd = kindyn.aba()  # this is the forward dynamics function:
    qddot = fd(q=q, v=qdot, tau=tau)['a']  # qddot = M^-1(tau - h)
    x, xdot = utils.double_integrator(q, qdot, qddot)  # xdot = [qdot, qddot]
else:
    x, xdot = utils.double_integrator(q, qdot, qddot)


if optimize_final_time:
    # Create final time variable
    dt_min = 0.005
    dt_max = 0.1
    dt_init = 0.01
    dt = prb.createSingleVariable("dt", 1)
    dt.setBounds(dt_min, dt_max)
    dt.setInitialGuess(dt_init)


prb.setDynamics(xdot)
prb.setDt(dt)

# Define LIMITS and INITIAL GUESS
# joint limits + initial pos
q_min = [-0.5, -2.*np.pi]
q_max = [0.5, 2.*np.pi]
q_init = [0., 0.]

q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, nodes=0)
q.setInitialGuess(q_init)

# velocity limits + initial vel
qdot_lims = np.array([100., 100.])
qdot_init = [0., 0.]

qdot.setBounds(-qdot_lims, qdot_lims)
qdot.setBounds(qdot_init, qdot_init, nodes=0)
qdot.setInitialGuess(qdot_init)

# input limits
if torque_input:
    tau_lims = np.array([3000, 0])
    tau_init = [0, 0]
    tau.setBounds(-tau_lims, tau_lims)
else:
    qddot_lims = np.array([1000., 1000.])
    qddot_init = [0., 0.]
    qddot.setBounds(-qddot_lims, qddot_lims)

if torque_input:
    tau.setInitialGuess(tau_init)

if solver_type == 'ipopt':
    # Set transcription method
    th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

if not torque_input:
    # Set dynamic feasibility:
    # the cart can apply a torque, the joint connecting the cart to the pendulum is UNACTUATED
    # the torques are computed using the inverse dynamics, as the input of the problem is the cart acceleration
    tau_lims = np.array([1000., 0.])
    tau = kin_dyn.InverseDynamics(kindyn).call(q, qdot, qddot)
    iv = prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lims, ub=tau_lims))

# Set desired constraints
# at the last node, the pendulum is upright
prb.createFinalConstraint("up", q[1] - np.pi)
# at the last node, the system velocity is zero
prb.createFinalConstraint("final_qdot", qdot)

# Set cost functions
# regularization of the input
if torque_input:
    prb.createIntermediateCost("tau", cs.sumsqr(tau))
else:
    prb.createIntermediateCost("qddot", cs.sumsqr(qddot))

# minimize the velocity
# prb.createIntermediateCost("damp", 0.01*cs.sumsqr(qdot))

if optimize_final_time:
    prb.createCost("min_dt", ns * 1e5 * cs.sumsqr(dt))

# ======================================================================================================================
opts = dict()
if solver_type == 'gnsqp':
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

solv = solver.Solver.make_solver(solver_type, prb, opts=opts)

# Solve the problem
if solver_type == 'gnsqp':
    solv.set_iteration_callback()
else:
    solv.solve()

# Get the solution as a dictionary
solution = solv.getSolutionDict()
# Get the array of dt, one for each interval between the nodes
dt_sol = solv.getDt()

total_time = sum(dt_sol)
print(dt_sol)
print(f"total trajectory time: {total_time}")
########################################################################################################################

if plot_sol:
    time = np.arange(0.0, total_time+1e-6, total_time/ns)
    plt.figure()
    plt.plot(time, solution['q'][0,:])
    plt.plot(time, solution['q'][1,:])
    plt.suptitle('$\mathrm{Base \ Position}$', size = 20)
    plt.xlabel('$\mathrm{[sec]}$', size = 20)
    plt.ylabel('$\mathrm{[m]}$', size = 20)

    hplt = PlotterHorizon(prb, solution)
    hplt.plotVariables()
    hplt.plotFunctions()
    plt.show()

if rviz_replay:

    # set ROS stuff and launchfile
    r = rospkg.RosPack()
    path_to_examples = r.get_path('horizon_examples')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/cart_pole.launch"])
    launch.start()
    rospy.loginfo("'cart_pole_fd' visualization started.")

    # visualize the robot in RVIZ
    joint_list=["cart_joint", "pole_joint"]
    replay_trajectory(total_time/ns, joint_list, solution['q']).replay(is_floating_base=False)

else:
    print("To visualize the robot trajectory, start the script with the '--replay")

