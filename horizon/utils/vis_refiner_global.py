from horizon.utils import utils, resampler_trajectory, mat_storer
from horizon.ros.replay_trajectory import *
import matplotlib.pyplot as plt



ms = mat_storer.matStorer('../playground/spot/spot_jump_refined_global.mat')
solution_refined = ms.load()
dt_res = solution_refined['dt_res'][0][0]
n_nodes_refined = solution_refined['n_nodes'][0][0]

ms = mat_storer.matStorer('../playground/spot/spot_jump.mat')
solution = ms.load()
dt = solution['dt'].flatten()
n_nodes = 50

nodes_vec = np.zeros([n_nodes + 1])
for i in range(1, n_nodes + 1):
    nodes_vec[i] = nodes_vec[i - 1] + dt[i - 1]


nodes_vec_refined = np.zeros([n_nodes_refined + 1])
for i in range(1, n_nodes_refined + 1):
    nodes_vec_refined[i] = nodes_vec_refined[i - 1] + dt_res

plt.figure()
for dim in range(solution_refined['q'].shape[0]):
    plt.plot(nodes_vec_refined, np.array(solution_refined['q'][dim, :]), '--')
plt.title('q_refined')


for dim in range(solution['q'].shape[0]):
    plt.plot(nodes_vec, np.array(solution['q'][dim, :]))
plt.title('q')

plt.show()

replay_traj = True
if replay_traj:

    urdffile = '../playground/urdf/spot.urdf'
    urdf = open(urdffile, 'r').read()
    kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

    n_c = 4
    n_q = kindyn.nq()
    n_v = kindyn.nv()
    n_f = 3

    joint_names = kindyn.joint_names()
    if 'universe' in joint_names: joint_names.remove('universe')
    if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

    contacts_name = {'lf_foot', 'rf_foot', 'lh_foot', 'rh_foot'}
    contact_map = dict(zip(contacts_name, [solution_refined['f0'], solution_refined['f1'], solution_refined['f2'], solution_refined['f3']]))


    repl = replay_trajectory(dt_res, joint_names, solution_refined['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
    repl.sleep(1.)
    repl.replay(is_floating_base=True)