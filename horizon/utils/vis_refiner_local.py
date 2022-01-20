from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
import matplotlib.pyplot as plt
import numpy as np
from horizon.ros.replay_trajectory import *

# ms = mat_storer.matStorer('../playground/spot/spot_jump_refined_local.mat')
ms = mat_storer.matStorer('refiner_spot_jump.mat')
solution_refined = ms.load()
nodes_vec_refined = solution_refined['times'][0]
ms = mat_storer.matStorer('../playground/spot/spot_jump.mat')
solution = ms.load()
dt = solution['dt'].flatten()
n_nodes = 50

nodes_vec = np.zeros([n_nodes + 1])
for i in range(1, n_nodes + 1):
    nodes_vec[i] = nodes_vec[i - 1] + dt[i - 1]

added_nodes = [node for node in nodes_vec_refined if node not in nodes_vec]

plt.figure()
for dim in range(solution_refined['q'].shape[0]):
    plt.plot(nodes_vec_refined, np.array(solution_refined['q'][dim, :]), '--')
if added_nodes:
    plt.vlines([added_nodes[0], added_nodes[-1]], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)
plt.title('q_refined')

for dim in range(solution['q'].shape[0]):
    plt.plot(nodes_vec, np.array(solution['q'][dim, :]))
plt.title('q')

plt.figure()
for dim in range(solution_refined['q_dot'].shape[0]):
    plt.plot(nodes_vec_refined, np.array(solution_refined['q_dot'][dim, :]), '--')
if added_nodes:
    plt.vlines([added_nodes[0], added_nodes[-1]], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)
plt.title('q_refined')

for dim in range(solution['q_dot'].shape[0]):
    plt.plot(nodes_vec, np.array(solution['q_dot'][dim, :]))
plt.title('q_dot')

# =============================================================
tau = solution['inverse_dynamics']['val'][0][0]
tau_ref = solution_refined['inverse_dynamics']['val'][0][0]
plt.figure()
for dim in range(6):
    plt.plot(nodes_vec_refined[:-1], np.array(tau_ref[dim, :]))
for dim in range(6):
    plt.scatter(nodes_vec[:-1], np.array(tau[dim, :]))
plt.title('tau on base')

plt.figure()
for dim in range(tau_ref.shape[0] - 6):
    plt.plot(nodes_vec_refined[:-1], np.array(tau_ref[6 + dim, :]))
for dim in range(tau.shape[0] - 6):
    plt.scatter(nodes_vec[:-1], np.array(tau[6 + dim, :]))
plt.title('tau')


contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))



f_list = ['f0', 'f1', 'f2', 'f3']
for f_ind in f_list:
    plt.figure()
    for dim in range(solution_refined[f_ind].shape[0]):
        plt.plot(nodes_vec_refined[:-1], np.array(solution_refined[f_ind][dim, :]), '--')

    for dim in range(solution[f_ind].shape[0]):
        plt.plot(nodes_vec[:-1], solution[f_ind][dim, :])

    plt.title(f_ind)
plt.show()