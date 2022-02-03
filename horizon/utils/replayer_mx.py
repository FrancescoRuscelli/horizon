import time

import numpy as np

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os
from scipy.io import loadmat


urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../playground/urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

N = 100
N_states = N + 1
N_control = N

dt = 0.02

ms = mat_storer.matStorer('../playground/spot/spot_backflip_manual.mat')
solution = ms.load()

sol = solution['a']
i = 0
q_dim = N_states * n_q
q_dot_dim = N_states * n_v
q_ddot_dim = N_control * n_v
f_dim = N_control * n_c * n_f

i = 0
i_new = q_dim
q_sol = sol[i:i_new]
i = i_new
i_new = i_new + q_dot_dim
q_dot_sol = sol[i:i_new]
i = i_new
i_new = i_new + q_ddot_dim
q_ddot_sol = sol[i:i_new]
i = i_new
i_new = i_new + f_dim
f_sol = sol[i:i_new]

q = np.reshape(q_sol, [n_q, N_states], order='F')
q_dot = np.reshape(q_dot_sol, [n_v, N_states], order='F')
q_ddot = np.reshape(q_ddot_sol, [n_v, N_control], order='F')
f = np.reshape(f_sol, [n_c * n_f, N_control], order='F')
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
f1 = f[0 * n_f:1 * n_f, :]
f2 = f[1 * n_f:2 * n_f, :]
f3 = f[2 * n_f:3 * n_f, :]
f4 = f[3 * n_f:4 * n_f, :]

f_list = [f1, f2, f3, f4]

contact_map = dict(zip(contacts_name, f_list))

# ========
# replaying
# ========
replay_traj = True
if replay_traj:
    joint_names = kindyn.joint_names()
    if 'universe' in joint_names: joint_names.remove('universe')
    if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')
    repl = replay_trajectory(dt, joint_names, q, contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

    repl.sleep(1.)
    repl.replay(is_floating_base=True)
# ========
# plotting
# ========
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

import matplotlib.pyplot as plt

plt.figure()
for dim in range(q.shape[0]):
    plt.plot(range(q.shape[1]), np.array(q[dim, :]))
plt.title('q')

plt.figure()
for dim in range(q_dot.shape[0]):
    plt.plot(range(q_dot.shape[1]), np.array(q_dot[dim, :]))
plt.title('q_dot')

plt.figure()
for dim in range(q_ddot.shape[0]):
    plt.plot(range(q_ddot.shape[1]), np.array(q_ddot[dim, :]))
plt.title('q_ddot')

pos_contact_list = list()
for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=q)['ee_pos']
    plt.figure()
    plt.title(contact)
    for dim in range(n_f):
        plt.plot(np.array([range(pos.shape[1])]), np.array(pos[dim, :]), marker="x", markersize=3, linestyle='dotted')

    plt.vlines([30, N+1], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)

# plane_xy
plt.figure()

for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=q)['ee_pos']

    plt.title(f'plane_xy')
    plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]))
    plt.scatter(np.array(pos[0, 0]), np.array(pos[1, 0]), c='blue')
    plt.scatter(np.array(pos[0, -1]), np.array(pos[1, -1]), c='green')

FK = cs.Function.deserialize(kindyn.centerOfMass())
pos_com = FK(q=q)['com']
plt.scatter(np.array(pos_com[0, :]), np.array(pos_com[1, :]), marker='x')
plt.scatter(np.array(pos_com[0, 0]), np.array(pos_com[1, 0]), marker='x', c='blue')
plt.scatter(np.array(pos_com[0, -1]), np.array(pos_com[1, -1]), marker='x', c='green')

# plane_xz
plt.figure()
for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=q)['ee_pos']

    plt.title(f'plane_xz')
    plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)
    plt.scatter(np.array(pos[0, 0]), np.array(pos[2, 0]),  c='blue')
    plt.scatter(np.array(pos[0, -1]), np.array(pos[2, -1]), c='green')

FK = cs.Function.deserialize(kindyn.centerOfMass())
pos_com = FK(q=q)['com']
plt.scatter(np.array(pos_com[1, :]), np.array(pos_com[2, :]), marker='x')
plt.scatter(np.array(pos_com[0, 0]), np.array(pos_com[2, 0]), marker='x', c='blue')
plt.scatter(np.array(pos_com[0, -1]), np.array(pos_com[2, -1]), marker='x', c='green')

plt.figure()
plt.scatter(np.array(pos_com[1, :]), np.array(pos_com[2, :]), marker='x')
# forces
plt.figure()

for f in f_list:
    plt.plot(np.array(range(f.shape[1])), f[0, :])
    plt.title(f'forces_x')

plt.figure()
for f in f_list:
    plt.plot(np.array(range(f.shape[1])), f[1, :])
    plt.title(f'forces_y')

plt.figure()
for f in f_list:
    plt.plot(np.array(range(f.shape[1])), f[2, :])
    plt.title(f'forces_z')

plt.show()
