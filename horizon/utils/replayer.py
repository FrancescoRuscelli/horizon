import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os
from scipy.io import loadmat


transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = '/home/francesco/hhcm_workspace/src/horizon/horizon/examples/urdf/spot.urdf'
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

tot_time = 1
dt_hint = 0.02
duration_step = 0.5

n_nodes = int(tot_time / dt_hint)
n_nodes_step = int(duration_step / dt_hint)

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

jump_height = 0.1
node_start_step = 15
node_end_step = node_start_step + n_nodes_step

ms = mat_storer.matStorer('/home/francesco/hhcm_workspace/src/horizon/horizon/examples/spot_jump.mat')
solution = ms.load()
contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))



pos_contact_list = list()
for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=solution['q'])['ee_pos']
    plt.figure()
    plt.title(contact)
    for dim in range(n_f):
        plt.plot(np.array([range(pos.shape[1])]), np.array(pos[dim, :]), marker="x", markersize=3, linestyle='dotted')

    plt.vlines([node_start_step, node_end_step], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)

plt.figure()
for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=solution['q'])['ee_pos']

    plt.title(f'plane_xy')
    plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

plt.figure()
for contact in contacts_name:
    FK = cs.Function.deserialize(kindyn.fk(contact))
    pos = FK(q=solution['q'])['ee_pos']

    plt.title(f'plane_xz')
    plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)


for f in [f'f{i}' for i in range(len(contacts_name))]:
    plt.figure()
    for dim in range(solution[f].shape[0]):
        plt.plot(np.array(range(solution[f].shape[1])), solution[f][dim, :])

    plt.title(f'force {f}')

plt.show()
# ======================================================
