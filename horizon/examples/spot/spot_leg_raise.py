import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os

ms = mat_storer.matStorer(f'{os.path.splitext(os.path.basename(__file__))[0]}.mat')

transcription_method = 'multiple_shooting'  # direct_collocation
transcription_opts = dict(integrator='RK4')

urdffile = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')


n_nodes = 100

n_c = 4
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3

node_start_step = 30
node_end_step = n_nodes

load_initial_guess = False

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


# import initial guess if present
if load_initial_guess:
    prev_solution = ms.load()
    q_ig = prev_solution['q']
    q_dot_ig = prev_solution['q_dot']
    q_ddot_ig = prev_solution['q_ddot']
    f_ig_list = list()
    for f in f_list:
        f_ig_list.append(prev_solution[f'f{i}'])

# SET DYNAMICS
# dt = prb.createInputVariable("dt", 1)  # variable dt as input
dt = 0.01
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
f_lim = 1000. * np.ones(n_f)


# set bounds and of q
q.setBounds(q_min, q_max)
q.setBounds(q_init, q_init, 0)
# set bounds of q_dot
q_dot_init = np.zeros(n_v)
q_dot.setBounds(-q_dot_lim, q_dot_lim)
# zero initial velocity
q_dot.setBounds(q_dot_init, q_dot_init, 0)
# zero final velocity
q_dot.setBounds(q_dot_init, q_dot_init, n_nodes+1)
# set bounds of q_ddot
q_ddot.setBounds(-q_ddot_lim, q_ddot_lim)
# set bounds of f
for f in f_list:
    f.setBounds(-f_lim, f_lim)
    f[2].setBounds(0, 1000)

# set bounds of dt
# dt.setBounds(dt_min, dt_max)


# SET TRANSCRIPTION METHOD
th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

tau = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED).call(q, q_dot,
                                                                                                             q_ddot,
                                                                                                             contact_map)
prb.createIntermediateConstraint("inverse_dynamics", tau[:6])

# SET CONTACT POSITION CONSTRAINTS
active_leg = ['lf_foot']

for frame, f in contact_map.items():
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    if frame not in active_leg:
        prb.createConstraint(f"{frame}_vel_before_step", v)
    else:
        prb.createConstraint(f"{frame}_vel_before_step", v, nodes=range(0, node_start_step))

    if frame in active_leg:
        prb.createConstraint(f"{frame}_no_force_during_lift", f, nodes=range(node_start_step, n_nodes))


# SET COST FUNCTIONS
prb.createCostFunction("min_q_dot", 10. * cs.sumsqr(q_dot))

FK = cs.Function.deserialize(kindyn.fk(active_leg[0]))
p = FK(q=q)['ee_pos']
p_start = FK(q=q_init)['ee_pos']
p_goal = p_start + [0., 0., -0.1]
prb.createFinalCost(f"lift_{frame}_leg", cs.sumsqr(p - p_start))
# prb.createIntermediateCost("min_q_ddot", 10. * cs.sumsqr(q_ddot))

# don't know why this is not working
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", 0.01 * cs.sumsqr(f))

# q_fb_trg = np.array([q_init[0], q_init[1], q_init[2] + 0.1, 0.0, 0.0, 0.0, 1.0])
# prb.createCostFunction("floating_base_orientation", 1000.*cs.sumsqr(q[3:7] - q_fb_trg[3:7]), nodes=list(range(node_start_step, node_end_step)))
# prb.createCostFunction("floating_base_position", 100000.*cs.sumsqr(q[0:7] - q_fb_trg))
# prb.createCostFunction("floating_base_position", 100000.*cs.sumsqr(q[3:7] - q_fb_trg[3:7]))
# =============
# SOLVE PROBLEM
# =============
opts = {'ipopt.tol': 0.001,
        'ipopt.constr_viol_tol': 0.001,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}

tic = time.time()
solver = solver.Solver.make_solver('ipopt', prb, opts)
toc = time.time()
print('time elapsed loading:', toc - tic)

tic = time.time()
solver.solve()
toc = time.time()
print('time elapsed solving:', toc - tic)

solution = solver.getSolutionDict()
solution_constraints = solver.getConstraintSolutionDict()

solution_constraints_dict = dict()
for name, item in prb.getConstraints().items():
    lb, ub = item.getBounds()
    lb_mat = np.reshape(lb, (item.getDim(), len(item.getNodes())), order='F')
    ub_mat = np.reshape(ub, (item.getDim(), len(item.getNodes())), order='F')
    solution_constraints_dict[name] = dict(val=solution_constraints[name], lb=lb_mat, ub=ub_mat, nodes=item.getNodes())

if isinstance(dt, cs.SX):
    ms.store({**solution, **solution_constraints_dict})
else:
    dt_dict = dict(constant_dt=dt)
    ms.store({**solution, **solution_constraints_dict, **dt_dict})

# ========================================================
plot_all = True
if plot_all:
    # hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables(show_bounds=False, legend=True)
    # hplt.plotFunctions(show_bounds=False)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))

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

    plt.show()
# ======================================================


contacts_name = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']
contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))

# resampling
resampling = False
if resampling:
    dt_res = 0.001
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], solution['dt'].flatten(), dt_res, dae, contact_map, kindyn,
                                                                            cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)



    repl = replay_trajectory(dt_res, joint_names, q_res, contact_map_res, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)
else:
    # remember to run a robot_state_publisher
    repl = replay_trajectory(dt, joint_names, solution['q'], contact_map, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED, kindyn)

repl.sleep(1.)
repl.replay(is_floating_base=True)
