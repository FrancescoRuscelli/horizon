import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, rospkg, argparse
from scipy.io import loadmat


parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('--replay', help='visualize the robot trajectory in rviz', action='store_true')
args = parser.parse_args()

rviz_replay = False
resampling = True
plot_sol = True


if args.replay:
    from horizon.ros.replay_trajectory import *
    import roslaunch, rospkg, rospy
    resampling = True
    rviz_replay = True
    plot_sol = False

r = rospkg.RosPack()
path_to_examples = r.get_path('horizon_examples')
# =========================================
# mat storer
file_name = os.path.splitext(os.path.basename(__file__))[0]
ms = mat_storer.matStorer(path_to_examples + f'/mat_files/{file_name}.mat')

transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')



n_nodes = 50

node_start_leap_front = 15
node_end_leap_front = 35

node_start_leap_hind = 25
node_end_leap_hind = 45

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3



# SET PROBLEM STATE AND INPUT VARIABLES
prb = problem.Problem(n_nodes)
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)

f_list = list()
for i in range(n_c):
    f_list.append(prb.createInputVariable(f'f{i}', n_f))

# SET CONTACTS MAP
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, f_list))


load_initial_guess = False
# import initial guess if present
if load_initial_guess:
    prev_solution = ms.load()
    q_ig = prev_solution['q']
    q_dot_ig = prev_solution['q_dot']
    q_ddot_ig = prev_solution['q_ddot']
    f_ig_list = list()
    for f in f_list:
        f_ig_list.append(prev_solution[f'f{i}'])

    dt_ig = prev_solution['dt']

# SET DYNAMICS
dt = prb.createInputVariable("dt", 1)  # variable dt as input
# dt = 0.01
# Computing dynamics
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)
# SET BOUNDS
# q bounds
q_min = [-10., -10., -10., -1., -1., -1., -1.]  # floating base
q_min.extend(kindyn.q_min()[7:])
q_min = np.array(q_min)

q_max = [10., 10., 10., 1., 1., 1., 1.]  # floating base
q_max.extend(kindyn.q_max()[7:])
q_max = np.array(q_max)

q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

# q_dot bounds
q_dot_lim = 100. * np.ones(n_v)
# q_ddot bounds
q_ddot_lim = 100. * np.ones(n_v)
# f bounds
f_lim = 10000. * np.ones(n_f)

dt_min = 0.01  # [s]
dt_max = 0.1  # [s]

# set bounds and of q
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
# set bounds of q_dot
q_dot_init = np.zeros(n_v)
q_dot.setBounds(-q_dot_lim, q_dot_lim)
q_dot.setBounds(q_dot_init, q_dot_init, 0)
# set bounds of q_ddot
q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)
# set bounds of f
# for f in f_list:
#     f.setBounds(-f_lim, f_lim)

f_min = [-10000., -10000., -10.]
f_max = [10000., 10000., 10000.]
for f in f_list:
    f.setBounds(f_min, f_max)
# set bounds of dt
if isinstance(dt, cs.SX):
    dt.setBounds(dt_min, dt_max)

# SET INITIAL GUESS
if load_initial_guess:
    for node in range(q_ig.shape[1]):
        q.setInitialGuess(q_ig[:, node], node)

    for node in range(q_dot_ig.shape[1]):
        q_dot.setInitialGuess(q_dot_ig[:, node], node)

    for node in range(q_ddot_ig.shape[1]):
        q_ddot.setInitialGuess(q_ddot_ig[:, node], node)

    for f, f_ig in zip(f_list, f_ig_list):
        for node in range(f_ig.shape[1]):
            f.setInitialGuess(f_ig[:, node], node)

    if isinstance(dt, cs.SX):
        for node in range(dt_ig.shape[1]):
            dt.setInitialGuess(dt_ig[:, node], node)

else:
    q.setInitialGuess(q_init)
    if isinstance(dt, cs.SX):
        dt.setInitialGuess(dt_min)



# SET TRANSCRIPTION METHOD
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# SET INVERSE DYNAMICS CONSTRAINTS
tau_lim = np.array([0., 0., 0., 0., 0., 0.,  # Floating base
                    1000., 1000., 1000.,  # Contact 1
                    1000., 1000., 1000.,  # Contact 2
                    1000., 1000., 1000.,  # Contact 3
                    1000., 1000., 1000.])  # Contact 4

tau = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot,
                                                                                                             q_ddot,
                                                                                                             contact_map)
prb.createIntermediateConstraint("inverse_dynamics", tau, bounds=dict(lb=-tau_lim, ub=tau_lim))

# SET FINAL VELOCITY CONSTRAINT
# prb.createFinalConstraint('final_velocity', q_dot)

# SET CONTACT POSITION CONSTRAINTS
active_leg = list()
active_leg = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

mu = 1
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

q_flight = q_init.copy()
q_flight[2] = q_flight[2] + 0.3

q_final = q_init.copy()
q_final[0] = q_final[0] + 0.5
q_final[2] = q_final[2] + 0.3

for frame, f in contact_map.items():
    # 2. velocity of each end effector must be zero
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(f, mu, R)

    if frame in ['lf_foot', 'rf_foot']:
        prb.createConstraint(f"{frame}_vel_before_lift", v, nodes=range(0, node_start_leap_front + 1))
        prb.createConstraint(f"{frame}_vel_after_lift", v, nodes=range(node_end_leap_front, n_nodes + 1))

        prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_leap_front, node_end_leap_front))

        # prb.createIntermediateConstraint(f"{frame}_fc_before_lift", fc, nodes=range(0, node_end_leap_front), bounds=dict(lb=fc_lb, ub=fc_ub))
        # prb.createIntermediateConstraint(f"{frame}_fc_after_lift", fc, nodes=range(node_end_leap_front, n_nodes), bounds=dict(lb=fc_lb, ub=fc_ub))

    if frame in ['lh_foot', 'rh_foot']:
        prb.createConstraint(f"{frame}_vel_before_lift", v, nodes=range(0, node_start_leap_hind + 1))
        prb.createConstraint(f"{frame}_vel_after_lift", v, nodes=range(node_end_leap_hind, n_nodes + 1))

        prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_leap_hind, node_end_leap_hind))

        # prb.createIntermediateConstraint(f"{frame}_fc_before_lift", fc, nodes=range(0, node_end_leap_hind), bounds=dict(lb=fc_lb, ub=fc_ub))
        # prb.createIntermediateConstraint(f"{frame}_fc_after_lift", fc, nodes=range(node_end_leap_hind, n_nodes), bounds=dict(lb=fc_lb, ub=fc_ub))


prb.createFinalConstraint(f"final_nominal_fb", q[0:3] - q_final[0:3])

# SET COST FUNCTIONS
prb.createCost("min_q_dot", 1 * cs.sumsqr(q_dot))
prb.createCost("fb", 1 * cs.sumsqr(q[2] - q_flight[2]))
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", 0.001 * cs.sumsqr(f))


# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 2000}
# 'ipopt.linear_solver': 'ma57'}

solver = solver.Solver.make_solver('ipopt', prb, opts)
solver.solve()
sol_dt = solver.getDt()

solution = solver.getSolutionDict()
solution_constraints = solver.getConstraintSolutionDict()

solution_constraints_dict = dict()
for name, item in prb.getConstraints().items():
    lb, ub = item.getBounds()
    lb_mat = np.reshape(lb, (item.getDim(), len(item.getNodes())), order='F')
    ub_mat = np.reshape(ub, (item.getDim(), len(item.getNodes())), order='F')
    solution_constraints_dict[name] = dict(val=solution_constraints[name], lb=lb_mat, ub=ub_mat, nodes=item.getNodes())

info_dict = dict(n_nodes=n_nodes,
                 node_start_leap_front=node_start_leap_front,
                 node_start_leap_hind=node_start_leap_hind,
                 node_end_leap_front=node_end_leap_front,
                 node_end_leap_hind=node_end_leap_hind)

if isinstance(dt, cs.SX):
    ms.store({**solution, **solution_constraints_dict, **info_dict})
else:
    dt_dict = dict(dt=dt)
    ms.store({**solution, **solution_constraints_dict, **info_dict, **dt_dict})

contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))

# resampling
if resampling:

    if isinstance(dt, cs.SX):
        dt_before_res = solution['dt'].flatten()
    else:
        dt_before_res = dt

    dt_res = 0.001
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
        kindyn,
        cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

if rviz_replay:

    # set ROS stuff and launchfile
    r = rospkg.RosPack()
    path_to_examples = r.get_path('horizon_examples')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path_to_examples + "/replay/launch/spot.launch"])
    launch.start()
    rospy.loginfo("'spot' visualization started.")

    if resampling:
        repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res,
                                 cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
    else:
        # remember to run a robot_state_publisher
        repl = replay_trajectory(dt, joint_names, solution['q'], contact_map,
                                 cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

    repl.sleep(1.)
    repl.replay(is_floating_base=True)

else:
    print("To visualize the robot trajectory, start the script with the '--replay")





















