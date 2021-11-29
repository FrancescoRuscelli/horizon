import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os
from horizon.solvers import Solver
from itertools import groupby
from operator import itemgetter


class Refiner:
    def __init__(self, prb: problem.Problem, new_nodes_vec, prev_solution):

        self.prev_solution = prev_solution

        self.prb = prb
        self.old_n_nodes = self.prb.getNNodes()
        self.new_nodes_vec = new_nodes_vec
        # todo
        # dt = prb.getDt()
        # print(dt)
        ms = mat_storer.matStorer('../examples/spot/spot_jump.mat')
        prev_solution = ms.load()
        prev_dt = prev_solution['dt'].flatten()

        self.nodes_vec = self.get_nodes_dt_constant(prev_dt)
        self.new_n_nodes = self.new_nodes_vec.shape[0]
        new_dt_vec = np.diff(self.new_nodes_vec)

        old_values = np.in1d(self.new_nodes_vec, nodes_vec)
        self.new_indices = np.arange(len(self.new_nodes_vec))[~old_values]
        self.base_indices = np.arange(len(self.new_nodes_vec))[old_values]

        # indices_exceed = np.unique(np.argwhere(np.abs(tau_sol_base) > threshold)[:, 1])
        # zip_indices_new = dict(zip(new_indices, indices_exceed))

        # map from base_indices to expanded indices: [0, 1, 2, 3, 4] ---> [0, 1, [2new, 3new, 4new], 2, 3, 4]
        self.old_to_new = dict(zip(range(n_nodes + 1), self.base_indices))
        self.new_to_old = {v: k for k, v in self.old_to_new.items()}
        # group elements
        ranges_base_indices = self.group_elements(self.base_indices)
        ranges_new_indices = self.group_elements(self.new_indices)

        # each first indices in ranges_base_indices
        first_indices = [item[1] for item in ranges_base_indices[:-1]]
        indices_to_expand = [self.new_to_old[elem] for elem in first_indices]

        # zip couples to expand with expanded nodes
        couples_to_inject = list()
        for base_elem, new_elem in zip(first_indices, ranges_new_indices):
            couples_to_inject.append((base_elem, base_elem + 1))

        self.elem_and_expansion = list(zip(indices_to_expand, ranges_new_indices))

        print('elem_and_expansion', self.elem_and_expansion)

    def get_nodes_dt_constant(self, dt):
        nodes_vec = np.zeros([self.old_n_nodes])
        for i in range(1, n_nodes + 1):
            nodes_vec[i] = nodes_vec[i - 1] + dt[i - 1]

        return nodes_vec

    def group_elements(self, vec):
        ranges_vec = list()
        for k, g in groupby(enumerate(vec), lambda x: x[0] - x[1]):
            group = (map(itemgetter(1), g))
            group = list(map(int, group))
            ranges_vec.append((group[0], group[-1]))

        return ranges_vec

    def expand_nodes(self, vec_to_expand):

        # fill new_samples_nodes with corresponding new_nodes
        new_nodes_vec = [self.old_to_new[elem] for elem in vec_to_expand]

        elem_and_expansion_masked = self.find_nodes_to_inject(vec_to_expand)

        for elem in elem_and_expansion_masked:
            if len(list(elem[1])) == 2:
                nodes_to_inject = list(range(elem[1][0], elem[1][1] + 1))
                new_nodes_vec.extend(nodes_to_inject)
            else:
                nodes_to_inject = elem[1][0]
                new_nodes_vec.append(nodes_to_inject)

        new_nodes_vec.sort()

        return new_nodes_vec

    def find_nodes_to_inject(self, vec_to_expand):
        recipe_vec = self.elem_and_expansion
        recipe_vec_masked = list()
        # expand couples of nodes
        for i in range(len(vec_to_expand) - 1):
            for j in range(len(recipe_vec)):
                if vec_to_expand[i] == recipe_vec[j][0]:
                    if vec_to_expand[i + 1] == vec_to_expand[i] + 1:
                        print(f'couple detected: {vec_to_expand[i], vec_to_expand[i] + 1}: injecting {recipe_vec[j][1]}')
                        recipe_vec_masked.append(recipe_vec[j])

        return recipe_vec_masked

    def resetProblem(self):

        self.prb.setNNodes(self.new_n_nodes-1)

        print(self.prb.getNNodes())

        # set constraints
        for name, cnsrt in self.prb.getConstraints().items():
            print(f'========================== constraint {name} =========================================')
            cnsrt_nodes_old = cnsrt.getNodes().copy()
            cnsrt_lb_bounds_old, cnsrt_ub_bounds_old = cnsrt.getBounds()

            print('old nodes:', cnsrt_nodes_old)
            cnsrt_nodes_new = ref.expand_nodes(cnsrt_nodes_old)
            cnsrt.setNodes(cnsrt_nodes_new, erasing=True)
            print('new nodes:', cnsrt.getNodes())

            # manage bounds
            k = 0
            for node in cnsrt.getNodes():
                if node in self.base_indices:
                    cnsrt.setBounds(cnsrt_lb_bounds_old[:, k], cnsrt_ub_bounds_old[:, k], node)
                    k += 1

            elem_and_expansion_masked = self.find_nodes_to_inject(cnsrt_nodes_old)

            for elem in elem_and_expansion_masked:
                if len(list(elem[1])) == 2:
                    nodes_to_inject = list(range(elem[1][0], elem[1][1] + 1))
                else:
                    nodes_to_inject = elem[1][0]

                cnsrt.setBounds(cnsrt_lb_bounds_old[:, elem[0]], cnsrt_ub_bounds_old[:, elem[0]], nodes=nodes_to_inject)


        # set cost functions
        for name, cost in self.prb.getCosts().items():
            print(f'============================ cost {name} =======================================')
            cost_nodes_old = cost.getNodes()
            print('old nodes:', cost_nodes_old)
            cost_nodes_new = ref.expand_nodes(cost_nodes_old)
            cost.setNodes(cost_nodes_new, erasing=True)
            print('new nodes:', cost.getNodes())

    def resetBounds(self):

        plot_ig = False

        # variables
        for name, var in prb.getVariables().items():
            print(f'============================ var {name} =======================================')
            k = 0
            for node in var.getNodes():
                if node in self.base_indices:
                    var.setInitialGuess(self.prev_solution[f'{name}'][:, k], node)
                    k += 1
                if node in self.new_indices:
                    print(f'node {node} requires an interpolated value to be initialized.')
                    # q.setInitialGuess(q_res[:, zip_indices_new[node]], node)

        for name, var in prb.getVariables().items():
            if plot_ig:

                for dim in range(self.prev_solution[f'{name}'].shape[0]):
                    from horizon.variables import InputVariable
                    nodes_vec_vis = self.nodes_vec
                    if isinstance(var, InputVariable):
                        nodes_vec_vis = self.nodes_vec[:-1]

                    plt.scatter(nodes_vec_vis, self.prev_solution[f'{name}'][dim, :], color='red')

                var_to_print = var.getInitialGuess()

                for dim in range(var_to_print.shape[0]):
                    nodes_vec_vis = self.new_nodes_vec
                    if isinstance(var, InputVariable):
                        nodes_vec_vis = self.new_nodes_vec[:-1]
                    plt.scatter(nodes_vec_vis, var_to_print[dim, :], edgecolors='blue', facecolor='none')

                plt.show()

    #
    def solveProblem(self):

        # =============
        # SOLVE PROBLEM
        # =============
        opts = {'ipopt.tol': 0.001,
                'ipopt.constr_viol_tol': 0.001,
                'ipopt.max_iter': 2000,
                'ipopt.linear_solver': 'ma57'}

        # parametric time
        for i in range(len(new_dt_vec)):
            dt.assign(new_dt_vec[i], nodes=i + 1)

        sol = Solver.make_solver('ipopt', prb, opts)
        sol.solve()

        solution = sol.getSolutionDict()





# =========================================
ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')

transcription_method = 'multiple_shooting'  # direct_collocation # multiple_shooting
transcription_opts = dict(integrator='RK4')

# rospack = rospkg.RosPack()
# rospack.get_path('spot_urdf')
urdffile = '../examples/urdf/spot.urdf'
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

n_nodes = 50

node_start_step = 20
node_end_step = 40
node_peak = 30
jump_height = 0.2

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

# ============================================================================================================
# ============================================ PROBLEM =======================================================
# ============================================================================================================

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
prb.createFinalConstraint('final_velocity', q_dot)

# SET CONTACT POSITION CONSTRAINTS
active_leg = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

mu = 1
R = np.identity(3, dtype=float)  # environment rotation wrt inertial frame

fb_during_jump = np.array([q_init[0], q_init[1], q_init[2] + jump_height, 0.0, 0.0, 0.0, 1.0])
q_final = q_init

for frame, f in contact_map.items():
    # 2. velocity of each end effector must be zero
    FK = cs.Function.deserialize(kindyn.fk(frame))
    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    p_goal = p_start + [0., 0., jump_height]
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    prb.createConstraint(f"{frame}_vel_before_lift", v,
                         nodes=list(range(0, node_start_step)) + list(range(node_end_step, n_nodes + 1)))
    # prb.createConstraint(f"{frame}_vel_after_lift", v, nodes=range(node_end_step, n_nodes + 1))

    # friction cones must be satisfied
    fc, fc_lb, fc_ub = kin_dyn.linearized_friciton_cone(f, mu, R)
    prb.createIntermediateConstraint(f"{frame}_fc_before_lift", fc, nodes=range(0, node_start_step),
                                     bounds=dict(lb=fc_lb, ub=fc_ub))
    prb.createIntermediateConstraint(f"{frame}_fc_after_lift", fc, nodes=range(node_end_step, n_nodes),
                                     bounds=dict(lb=fc_lb, ub=fc_ub))

    prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_step, node_end_step))

    prb.createConstraint(f"start_{frame}_leg", p - p_start, nodes=node_start_step)
    prb.createConstraint(f"lift_{frame}_leg", p - p_goal, nodes=node_peak)
    prb.createConstraint(f"land_{frame}_leg", p - p_start, nodes=node_end_step)

# SET COST FUNCTIONS
# prb.createCostFunction(f"jump_fb", 10000 * cs.sumsqr(q[2] - fb_during_jump[2]), nodes=node_start_step)
prb.createCostFunction("min_q_dot", 1. * cs.sumsqr(q_dot))
prb.createFinalCost(f"final_nominal_pos", 1000 * cs.sumsqr(q - q_init))
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", 0.01 * cs.sumsqr(f))

# prb.createIntermediateCost('min_dt', 100 * cs.sumsqr(dt))

# ===================================================================================================================
# =============
# FAKE SOLVE PROBLEM
# =============


ms = mat_storer.matStorer('../examples/spot/spot_jump.mat')
prev_solution = ms.load()

n_nodes = prev_solution['n_nodes'][0][0]

node_start_step = prev_solution['node_start_step'][0][0]
node_end_step = prev_solution['node_end_step'][0][0]
node_peak = prev_solution['node_peak'][0][0]
jump_height = prev_solution['jump_height'][0][0]

prev_q = prev_solution['q']
prev_q_dot = prev_solution['q_dot']
prev_q_ddot = prev_solution['q_ddot']

prev_f_list = list()
for i in range(n_c):
    prev_f_list.append(prev_solution[f'f{i}'])

prev_tau = prev_solution['inverse_dynamics']['val'][0][0]
contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
prev_contact_map = dict(zip(contacts_name, prev_f_list))

joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

if 'dt' in prev_solution:
    prev_dt = prev_solution['dt'].flatten()
elif 'constant_dt' in prev_solution:
    prev_dt = prev_solution['constant_dt'].flatten()[0]
elif 'param_dt' in prev_solution:
    prev_dt = prev_solution['param_dt'].flatten()

dt_res = 0.001

q_sym = cs.SX.sym('q', n_q)
q_dot_sym = cs.SX.sym('q_dot', n_v)
q_ddot_sym = cs.SX.sym('q_ddot', n_v)
x, x_dot = utils.double_integrator_with_floating_base(q_sym, q_dot_sym, q_ddot_sym)

dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}
q_res, qdot_res, qddot_res, contact_map_res, tau_sol_res = resampler_trajectory.resample_torques(
    prev_q, prev_q_dot, prev_q_ddot, prev_dt, dt_res, dae, prev_contact_map,
    kindyn,
    cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)

f_res_list = list()
for f in prev_f_list:
    f_res_list.append(resampler_trajectory.resample_input(f, prev_dt, dt_res))

num_samples = tau_sol_res.shape[1]

nodes_vec = np.zeros([n_nodes + 1])
for i in range(1, n_nodes + 1):
    nodes_vec[i] = nodes_vec[i - 1] + prev_dt[i - 1]

nodes_vec_res = np.zeros([num_samples + 1])
for i in range(1, num_samples + 1):
    nodes_vec_res[i] = nodes_vec_res[i - 1] + dt_res

tau_sol_base = tau_sol_res[:6, :]

threshold = 5
## get index of values greater than a given threshold for each dimension of the vector, and remove all the duplicate values (given by the fact that there are more dimensions)
indices_exceed = np.unique(np.argwhere(np.abs(tau_sol_base) > threshold)[:, 1])
# these indices corresponds to some nodes ..
values_exceed = nodes_vec_res[indices_exceed]

## search for duplicates and remove them, both in indices_exceed and values_exceed
indices_duplicates = np.where(np.in1d(values_exceed, nodes_vec))
value_duplicates = values_exceed[indices_duplicates]

values_exceed = np.delete(values_exceed, np.where(np.in1d(values_exceed, value_duplicates)))
indices_exceed = np.delete(indices_exceed, indices_duplicates)

## base vector nodes augmented with new nodes + sort
nodes_vec_augmented = np.concatenate((nodes_vec, values_exceed))
nodes_vec_augmented.sort(kind='mergesort')

# ===================================================================================================================

# =============
# SOLVE PROBLEM
# =============


# opts = {'ipopt.tol': 0.001,
#         'ipopt.constr_viol_tol': 0.001,
#         'ipopt.max_iter': 2000,
#         'ipopt.linear_solver': 'ma57'}
#
# solver = solver.Solver.make_solver('ipopt', prb, opts)
# solver.solve()
#
# solution = solver.getSolutionDict()
# solution_constraints = solver.getConstraintSolutionDict()
# ===========================================================================================


# ===========================================================================================
# ===========================================================================================
# ms = mat_storer.matStorer('../examples/spot/spot_jump.mat')
# prev_solution = ms.load()

# n_nodes = prev_solution['n_nodes'][0][0]

# node_start_step = prev_solution['node_start_step'][0][0]
# node_end_step = prev_solution['node_end_step'][0][0]
# node_peak = prev_solution['node_peak'][0][0]
# jump_height = prev_solution['jump_height'][0][0]

# prev_q = prev_solution['q']
# prev_q_dot = prev_solution['q_dot']
# prev_q_ddot = prev_solution['q_ddot']
#
# prev_f_list = list()
# for i in range(n_c):
#     prev_f_list.append(prev_solution[f'f{i}'])
#
# # prev_tau = prev_solution['inverse_dynamics']['val'][0][0]
# contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
# prev_contact_map = dict(zip(contacts_name, prev_f_list))
#
# joint_names = kindyn.joint_names()
# if 'universe' in joint_names: joint_names.remove('universe')
# if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')
#
# if 'dt' in prev_solution:
#     prev_dt = prev_solution['dt'].flatten()
# elif 'constant_dt' in prev_solution:
#     prev_dt = prev_solution['constant_dt'].flatten()[0]
# elif 'param_dt' in prev_solution:
#     prev_dt = prev_solution['param_dt'].flatten()
#
# dt_res = 0.001
#
# q_sym = cs.SX.sym('q', n_q)
# q_dot_sym = cs.SX.sym('q_dot', n_v)
# q_ddot_sym = cs.SX.sym('q_ddot', n_v)
# x, x_dot = utils.double_integrator_with_floating_base(q_sym, q_dot_sym, q_ddot_sym)
#
# dae = {'x': x, 'p': q_ddot_sym, 'ode': x_dot, 'quad': 1}
# q_res, qdot_res, qddot_res, contact_map_res, tau_sol_res = resampler_trajectory.resample_torques(
#     prev_q, prev_q_dot, prev_q_ddot, prev_dt, dt_res, dae, prev_contact_map,
#     kindyn,
#     cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
#
# f_res_list = list()
# for f in prev_f_list:
#     f_res_list.append(resampler_trajectory.resample_input(f, prev_dt, dt_res))
#
# num_samples = tau_sol_res.shape[1]
#
# nodes_vec = np.zeros([n_nodes + 1])
# for i in range(1, n_nodes + 1):
#     nodes_vec[i] = nodes_vec[i - 1] + prev_dt[i - 1]
#
# nodes_vec_res = np.zeros([num_samples + 1])
# for i in range(1, num_samples + 1):
#     nodes_vec_res[i] = nodes_vec_res[i - 1] + dt_res
#
# tau_sol_base = tau_sol_res[:6, :]
#
# threshold = 5
# ## get index of values greater than a given threshold for each dimension of the vector, and remove all the duplicate values (given by the fact that there are more dimensions)
# indices_exceed = np.unique(np.argwhere(np.abs(tau_sol_base) > threshold)[:, 1])
# # these indices corresponds to some nodes ..
# values_exceed = nodes_vec_res[indices_exceed]
#
# ## search for duplicates and remove them, both in indices_exceed and values_exceed
# indices_duplicates = np.where(np.in1d(values_exceed, nodes_vec))
# value_duplicates = values_exceed[indices_duplicates]
#
# values_exceed = np.delete(values_exceed, np.where(np.in1d(values_exceed, value_duplicates)))
# indices_exceed = np.delete(indices_exceed, indices_duplicates)
#
# ## base vector nodes augmented with new nodes + sort
# nodes_vec_augmented = np.concatenate((nodes_vec, values_exceed))
# nodes_vec_augmented.sort(kind='mergesort')
#
# print(nodes_vec_augmented)
# ===========================================================================================
# ===========================================================================================

ref = Refiner(prb, nodes_vec_augmented, prev_solution)

plot_nodes = False
if plot_nodes:
    plt.figure()
    # nodes old
    plt.scatter(nodes_vec_augmented, np.zeros([nodes_vec_augmented.shape[0]]), edgecolors='red', facecolor='none')
    plt.scatter(nodes_vec, np.zeros([nodes_vec.shape[0]]), edgecolors='blue', facecolor='none')
    plt.show()

# print('prev_dt_vec', prev_dt)
# print('new_dt_vec', new_dt_vec)

# if dt is variable:
#     make_parametric(dt)

# ======================================================================================================================
# SET PROBLEM STATE AND INPUT VARIABLES
ref.resetProblem()

ref.resetBounds()
ref.solveProblem()
exit()
# dt = prb.createParameter("dt", 1, nodes=range(1, n_nodes+1))  # variable dt as input

# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}

for i in range(len(new_dt_vec)):
    dt.assign(new_dt_vec[i], nodes=i + 1)

sol = Solver.make_solver('ipopt', prb, dt, opts)
sol.solve()

solution = sol.getSolutionDict()
solution_constraints = sol.getConstraintSolutionDict()

solution_constraints_dict = dict()
for name, item in prb.getConstraints().items():
    lb, ub = item.getBounds()
    lb_mat = np.reshape(lb, (item.getDim(), len(item.getNodes())), order='F')
    ub_mat = np.reshape(ub, (item.getDim(), len(item.getNodes())), order='F')
    solution_constraints_dict[name] = dict(val=solution_constraints[name], lb=lb_mat, ub=ub_mat, nodes=item.getNodes())

from horizon.variables import Variable, SingleVariable, Parameter, SingleParameter

info_dict = dict(n_nodes=n_nodes, times=nodes_vec_augmented, node_start_step=node_start_step,
                 node_end_step=node_end_step, node_peak=node_peak, jump_height=jump_height)
if isinstance(dt, Variable) or isinstance(dt, SingleVariable):
    ms.store({**solution, **solution_constraints_dict, **info_dict})
elif isinstance(dt, Parameter) or isinstance(dt, SingleParameter):
    dt_dict = dict(param_dt=new_dt_vec)
    ms.store({**solution, **solution_constraints_dict, **info_dict, **dt_dict})
else:
    dt_dict = dict(constant_dt=dt)
    ms.store({**solution, **solution_constraints_dict, **info_dict, **dt_dict})
