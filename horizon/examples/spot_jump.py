import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os

transcription_method = 'multiple_shooting' # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)


# OPTIMIZATION PARAMETERS
n_nodes = 40  # number of shooting nodes
lift_node = 15
touch_down_node = 35
jump_heigth = 0.1

# min and max node length
dt_min = 0.01  # [s]
dt_max = 0.15  # [s]

n_c = 4  # number of contacts
n_q = kindyn.nq()  # dofs NB: 7 DoFs floating base (quaternions)
n_v = kindyn.nv()  # dofs velocity
n_f = 3  # dofs force

# Create horizon problem
prb = problem.Problem(n_nodes)

#  STATE variables
q = prb.createStateVariable("q", n_q)
q_dot = prb.createStateVariable("q_dot", n_v)

# Creates problem CONTROL variables
# create acceleration input
q_ddot = prb.createInputVariable("q_ddot", n_v)  # joint acc as inputs

# create forces input
f_list = list()
for i in range(n_c):
    f_list.append(prb.createInputVariable(f"f{i}", n_f))  # foot 1 force input

contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, f_list))

# dt = prb.createInputVariable("dt", 1)  # variable dt as input
dt = 0.01
# Computing dynamics
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)

q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
          0.0, 0.789798138441726, -1.5238505,
          0.0, 0.789798138441726, -1.5202315,
          0.0, 0.789798138441726, -1.5300265,
          0.0, 0.789798138441726, -1.5253125])

# set bounds and intial guess of q
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
q.setInitialGuess(q_init)

# set bounds of q_dot
q_dot_lim = 100.*np.ones(n_v)
q_dot_init = np.zeros(n_v)
q_dot.setBounds(-q_dot_lim, q_dot_lim)
q_dot.setBounds(q_dot_init, q_dot_init, 0)

# set bounds of q_ddot
q_ddot_lim = 100.*np.ones(n_v)
q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)

# set bounds of f
f_lim = 10000.*np.ones(n_f)
for f in f_list:
    f.setBounds(-f_lim, f_lim)

## set bounds of time
# dt.setBounds(dt_min, dt_max)
# dt.setInitialGuess(dt_min)

q_fb_traj = np.array([q_init[0], q_init[1], q_init[2] + jump_heigth, 0.0, 0.0, 0.0, 1.0])

prb.createCostFunction("jump", 10.*cs.sumsqr(q[0:3] - q_fb_traj[0:3]), nodes=list(range(lift_node, touch_down_node)))
prb.createCostFunction("min_q_dot", 10.*cs.sumsqr(q_dot))
th = Transcriptor.make_method(transcription_method, prb, dt, opts=transcription_opts)

tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
            1000., 1000., 1000.,  # Contact 1
            1000., 1000., 1000.,  # Contact 2
            1000., 1000., 1000.,  # Contact 3
            1000., 1000., 1000.])  # Contact 4

tau = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot, q_ddot, contact_map)

prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lim, ub=tau_lim))
prb.createFinalConstraint('final_velocity', q_dot)

# GROUND
mu = 1 #0.8  # friction coefficient
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

# for frame, f in contact_map.items():
#     # BEFORE AND AFTER FLIGHT PHASE
#     FK = cs.Function.deserialize(kindyn.fk(frame))
#     p = FK(q=q)['ee_pos']
#     pd = FK(q=q_init)['ee_pos']
#     # 1. position of each end effector and its initial position must be the same
#     prb.createConstraint(f"{frame}_pos", p - pd)

for frame, f in contact_map.items():
    # BEFORE AND AFTER FLIGHT PHASE
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=q)['ee_pos']
    pd = FK(q=q_init)['ee_pos']
    # 1. position of each end effector and its initial position must be the same
    prb.createConstraint(f"{frame}_before_jump", p - pd, nodes=range(0, lift_node))
    prb.createConstraint(f"{frame}_after_jump", p - pd, nodes=range(touch_down_node, n_nodes + 1))

    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    # 2. velocity of each end effector must be zero
    prb.createConstraint(f"{frame}_vel_before_jump", v, nodes=range(0, lift_node))
    prb.createConstraint(f"{frame}_vel_after_jump", v, nodes=range(touch_down_node, n_nodes + 1))

    fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
    # 3. friction cones must be satisfied
    prb.createConstraint(f"{frame}_friction_cone_before_jump", fc, nodes=range(0, lift_node), bounds=dict(lb=fc_lb, ub=fc_ub))
    prb.createConstraint(f"{frame}_friction_cone_after_jump", fc, nodes=range(touch_down_node, n_nodes), bounds=dict(lb=fc_lb, ub=fc_ub))

    # DURING FLIGHT PHASE
    # force exerted on the ground must be zero in flight phase
    prb.createConstraint(f"{frame}_no_force_during_jump", f, nodes=range(lift_node, touch_down_node))

# Create problem
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 5000,
        'ipopt.linear_solver': 'ma57'}

solver = solver.Solver.make_solver('ipopt', prb, dt, opts)
solver.solve()

solution = solver.getSolutionDict()

plot_all = True
if plot_all:
    hplt = plotter.PlotterHorizon(prb, solution)
    hplt.plotVariables(show_bounds=False)
    # hplt.plotFunctions(show_bounds=False)
    plt.show()

# repl = replay_trajectory(dt, kindyn.joint_names(), q_res, frame_force_res_mapping, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
# repl.sleep(1.)
# repl.replay()


