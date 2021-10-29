from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
import matplotlib.pyplot as plt
import numpy as np

ms = mat_storer.matStorer('../examples/spot/spot_jump_refined.mat')
solution_refined = ms.load()
nodes_vec_refined = solution_refined['times'][0]
ms = mat_storer.matStorer('../examples/spot/spot_jump.mat')
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
plt.vlines([added_nodes[0], added_nodes[-1]], plt.gca().get_ylim()[0], plt.gca().get_ylim()[1], linestyles='dashed', colors='k', linewidth=0.4)
plt.title('q_refined')


for dim in range(solution['q'].shape[0]):
    plt.plot(nodes_vec, np.array(solution['q'][dim, :]))
plt.title('q')

plt.show()