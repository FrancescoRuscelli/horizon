# /usr/bin/env python3
import numpy as np
##### for robot and stuff
import xbot_interface.config_options as xbot_opt
from xbot_interface import xbot_interface as xbot
import rospy
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer

from horizon.ros.replay_trajectory import *

## ==================
## PREPARE TRAJECTORY
## ==================
transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = '../playground/urdf/spot.urdf'
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

n_nodes = 50

node_start_step = 20
node_end_step = 40
node_peak = 30
jump_height = 0.2

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3


ms = mat_storer.matStorer('../playground/spot/spot_jump_refined_local.mat')
solution = ms.load()

tau = solution['inverse_dynamics']['val'][0][0]
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))


joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

if 'dt' in solution:
    dt_before_res = solution['dt'].flatten()
elif 'constant_dt' in solution:
    dt_before_res = solution['constant_dt'].flatten()[0]
elif 'param_dt' in solution:
    dt_before_res = solution['param_dt'].flatten()

dt_res = 0.001

q = cs.SX.sym('q', n_q)
q_dot = cs.SX.sym('q_dot', n_v)
q_ddot = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)

dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
    solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
    kindyn,
    cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

num_samples = tau_res.shape[1]

if 'param_dt' in solution:
    n_nodes = solution['n_nodes'] - 1
    nodes_vec = solution['times'][0]
else:
    nodes_vec = np.zeros([n_nodes + 1])
    for i in range(1, n_nodes + 1):
        nodes_vec[i] = nodes_vec[i - 1] + dt_before_res[i - 1]

node_vec_res = np.zeros([num_samples + 1])
for i in range(1, num_samples + 1):
    node_vec_res[i] = node_vec_res[i - 1] + dt_res

replay_traj = False
if replay_traj:
    repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
    repl.sleep(1.)
    repl.replay(is_floating_base=True)

plot_flag = True
if plot_flag:
    import matplotlib.pyplot as plt

    plt.figure()
    for dim in range(q_res.shape[0]):
        plt.plot(node_vec_res, np.array(q_res[dim, :]))

    for dim in range(solution['q'].shape[0]):
        plt.scatter(nodes_vec, np.array(solution['q'][dim, :]))
    plt.title('q')

    plt.figure()
    for dim in range(qdot_res.shape[0]):
        plt.plot(node_vec_res, np.array(qdot_res[dim, :]))

    for dim in range(solution['q_dot'].shape[0]):
        plt.scatter(nodes_vec, np.array(solution['q_dot'][dim, :]))
    plt.title('qdot')

    plt.figure()
    for dim in range(qddot_res.shape[0]):
        plt.plot(node_vec_res[:-1], np.array(qddot_res[dim, :]))

    for dim in range(solution['q_ddot'].shape[0]):
        plt.scatter(nodes_vec[:-1], np.array(solution['q_ddot'][dim, :]))
    plt.title('q_ddot')

    plt.figure()
    for dim in range(6):
        plt.plot(node_vec_res[:-1], np.array(tau_res[dim, :]))
    for dim in range(6):
        plt.scatter(nodes_vec[:-1], np.array(tau[dim, :]))
    plt.title('tau on base')

    plt.figure()
    for dim in range(tau_res.shape[0]-6):
        plt.plot(node_vec_res[:-1], np.array(tau_res[6+dim, :]))
    for dim in range(tau.shape[0] - 6):
        plt.scatter(nodes_vec[:-1], np.array(tau[6 + dim, :]))
    plt.title('tau')
    plt.show()

## PREPARE ROBOT

rospy.init_node('spot')
opt = xbot_opt.ConfigOptions()

urdf = rospy.get_param('/xbotcore/robot_description')
srdf = rospy.get_param('/xbotcore/robot_description_semantic')

opt.set_urdf(urdf)
opt.set_srdf(srdf)
opt.generate_jidmap()
opt.set_bool_parameter('is_model_floating_base', True)
opt.set_string_parameter('model_type', 'RBDL')
opt.set_string_parameter('framework', 'ROS')
model = xbot.ModelInterface(opt)
robot = xbot.RobotInterface(opt)

robot_state = np.zeros( n_q-7)
robot_state = robot.getJointPosition()


q_robot = q_res[7:, :]
q_dot_robot = qdot_res[6:, :]
tau_robot = tau_res[6:, :]


q_homing = q_robot[:, 0]

robot.sense()
rate = rospy.Rate(1./dt_res)

for i in range(100):
    robot.setPositionReference(q_homing)
    # robot.setStiffness(4 *[1000, 1000, 500]) #[200, 200, 100]
    # robot.setDamping(4 * [50, 50, 30])  # [200, 200, 100]
    robot.setStiffness(4 *[200, 200, 100]) #[200, 200, 100]
    # robot.setDamping(4 * [50, 50, 30])  # [200, 200, 100]
    robot.move()

# crude homing

input('press a button to replay')

for i in range(num_samples):
    robot.setPositionReference(q_robot[:, i])
    robot.setVelocityReference(q_dot_robot[:, i])
    robot.setEffortReference(tau_robot[:, i])
    robot.move()
    rate.sleep()

print("done, if you didn't notice")