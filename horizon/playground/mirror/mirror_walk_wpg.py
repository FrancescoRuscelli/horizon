from horizon.rhc.horizon_wpg import HorizonWpg, Step

import numpy as np

from threading import Lock

import rospy
import rospkg
from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from xbot_msgs.msg import JointState as XBotJointState

"""
An application of mirror walking using the WPG: inside it uses the TaskInterface.
"""

rospy.init_node('mirror_wpg_node')

urdf_path = rospkg.RosPack().get_path('mirror_urdf') + '/urdf/mirror.urdf'
urdf = open(urdf_path, 'r').read()
rospy.set_param('/robot_description', urdf)

# contact frames
contacts = [f'arm_{i + 1}_TCP' for i in range(3)]

# initial config
q_init = {}

for i in range(3):
    q_init[f'arm_{i + 1}_joint_2'] = -1.9
    q_init[f'arm_{i + 1}_joint_3'] = -2.30
    q_init[f'arm_{i + 1}_joint_5'] = -0.4

base_init = np.array([0, 0, 0.72, 0, 0, 0, 1])

try:
    js = rospy.wait_for_message('/xbotcore/joint_states', XBotJointState, timeout=rospy.Duration(1.0))
    for jname, jpos in zip(js.name, js.position_reference):
        q_init[jname] = round(jpos, 2)
    print('set q init from joint state')
except rospy.ROSException as e:
    print(f'no joint state received, using default initial pose ({e})')

wpg = HorizonWpg(urdf=urdf,
                 contacts=contacts,
                 fixed_joints=[],
                 opt={
                     'fmin': 100.0,
                     'tf': 14.0,
                     'solver_type': 'ilqr'
                 },
                 q_init=q_init,
                 base_init=base_init
                 )

# create a gait pattern
k = 0
s = Step(frame='arm_0_joint_2', k_start=k + int(2.0 / wpg.dt), k_goal=k + int(4.0 / wpg.dt),
         goal=[0.6, 0, 0])
s.clearance = 0.20

# create a gait pattern
steps = list()
n_steps = 3
pattern = contacts.copy()
stride_time = 10.0
duty_cycle = 0.80
tinit = 2.0

for i in range(n_steps):
    l = pattern[i % wpg.nc]
    t_start = tinit + i * stride_time / wpg.nc
    t_goal = t_start + stride_time * (1 - duty_cycle)
    s = Step(frame=l, k_start=k + int(t_start / wpg.dt), k_goal=k + int(t_goal / wpg.dt))
    s.clearance = 0.20
    steps.append(s)

# set it to wpg
wpg.steps = steps
wpg.set_mode('base_ctrl')
wpg.set_target_position(x=0.40, y=0.0, rotz=0)
wpg.toggle_base_dof(id=2, enabled=False)
wpg._set_gait_pattern(k0=k)
try:
    wpg.load_solution('/tmp/mirror_wpg_solution.mat')
except FileNotFoundError:
    pass
wpg.bootstrap()
wpg.resample(dt_res=0.01)
wpg.save_solution('/tmp/mirror_wpg_solution.mat')
wpg.replay()

exit()

# TBD


# some publisher
# rospy.init_node('horizon_wpg_node')
pubs = {}
for c in contacts:
    pubs[c] = rospy.Publisher(f'/horizon/wpg/{c}/path', Path, queue_size=1)

pubs['com'] = rospy.Publisher(f'/horizon/wpg/com/path', Path, queue_size=1)

pubs['traj'] = rospy.Publisher(f'/horizon/wpg/joint_trajectory', JointTrajectory, queue_size=3)

# current iteration
k = 0

# lock to sync subscriber thread with main thread
lock = Lock()


def on_step_recv(msg: PointStamped):
    lock.acquire()

    l = str(input('I am sorry to bother you, which frame should I move ?'))
    cl = float(input('I am sorry to bother you, which clearance should I set ?'))

    # create a gait pattern
    s = Step(frame=l, k_start=k + int(2.0 / wpg.dt), k_goal=k + int(4.0 / wpg.dt),
             goal=[msg.point.x, msg.point.y, msg.point.z])
    s.clearance = cl

    # set it to wpg
    wpg.steps = [s]
    wpg.set_mode('step_ctrl')
    wpg.bootstrap()

    lock.release()


def on_goal_recv(msg: PoseStamped):
    lock.acquire()

    # create a gait pattern
    steps = list()
    n_steps = 8
    pattern = [0, 3, 1, 2]
    stride_time = 6.0
    duty_cycle = 0.80
    tinit = 1.0

    for i in range(n_steps):
        l = pattern[i % wpg.nc]
        t_start = tinit + i * stride_time / wpg.nc
        t_goal = t_start + stride_time * (1 - duty_cycle)
        s = Step(frame=l, k_start=k + int(t_start / wpg.dt), k_goal=k + int(t_goal / wpg.dt))
        steps.append(s)

    wpg.steps = steps
    wpg.set_mode('base_ctrl')
    wpg.bootstrap()
    wpg.set_target_position(x=msg.pose.position.x, y=msg.pose.position.y, rotz=0)

    lock.release()


goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, on_goal_recv)
point_sub = rospy.Subscriber('clicked_point', PointStamped, on_step_recv)

# start rti
rate = rospy.Rate(1. / wpg.dt)
while not rospy.is_shutdown():

    lock.acquire()

    wpg.update_initial_guess(dk=1)
    wpg.rti(k=k)
    k += 1

    wpg.publish_solution()

    path_msgs = wpg.compute_path_msg()
    trjmsg = wpg.compute_trj_msg()

    for c, msg in path_msgs.items():
        pubs[c].publish(msg)

    pubs['traj'].publish(trjmsg)

    lock.release()

    rate.sleep()
